/* 
 * Project myProject
 * Author: Your Name
 * Date: 
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "Adafruit_BME280.h"
#include "Adafruit_AS7341.h"
#include "../lib/Adafruit_BME280/src/Adafruit_BME280.h"
#include "../lib/Adafruit_AS7341/src/Adafruit_AS7341.h"


// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(SEMI_AUTOMATIC);


const BleUuid serviceUuid("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");

const BleUuid txUuid("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");
BleCharacteristic txCharacteristic("tx",BleCharacteristicProperty::INDICATE,txUuid,serviceUuid);
BleAdvertisingData data;

const int outgoingMessageSize = 100;
uint8_t outgoingMessage[outgoingMessageSize];
unsigned int lastRead;




struct enviroSensors{
  float tempF;
  int humidity;
  float baroPressure;
  uint16_t readings[12];
  float counts[12];
};
enviroSensors enviroData;

Adafruit_BME280 bme;
Adafruit_AS7341 as7341;


void getBME(enviroSensors bmeData);
void getAS7341(enviroSensors as7341Data);



void sendThenDisconnect();



void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected,5000);

  BLE.on();
  BLE.addCharacteristic(txCharacteristic);
  data.appendServiceUUID(txUuid);
  data.appendLocalName("12");
  BLE.advertise(&data);
  BLE.setTxPower(8);
  pinMode(D7, OUTPUT);
  digitalWrite(D7, LOW);

  if(!bme.begin(0x77)){
    Serial.printf("Could not start BME280\n");
  }
  else{
    Serial.printf("successfully started BME280\n");
  }

  if(!as7341.begin()){
    Serial.printf("Could not start AS7341\n");
  }
  else{
    Serial.printf("successfully started AS7341\n");
  }

  Serial.printf("Photon2 BLE Address: %s\n",BLE.address().toString().c_str());
  delay(5000);
  as7341.setATIME(100);
  as7341.setASTEP(999);
  as7341.setGain(AS7341_GAIN_64X);
}
  
void loop() {
  if(BLE.connected()){
    delay(5000);
    sendThenDisconnect();
  }
  else{
    digitalWrite(D7, LOW);
    if((millis() - lastRead) > 5000){
      lastRead = millis();
      getBME(enviroData);
      getAS7341(enviroData);
    }
  }
}

void sendThenDisconnect(){
 
  float tempF;
  int humidity;
  float baroPressure;
  uint16_t readings[12];
  float counts[12];

  if (!as7341.readAllChannels(readings)){
    Serial.printf("Error reading all channels!\n");
    //return;
  }

  for(uint8_t i = 0; i < 12; i++) {
    if(i == 4 || i == 5){
      continue;
    }
    // we skip the first set of duplicate clear/NIR readings
    // (indices 4 and 5)
    counts[i] = as7341.toBasicCounts(readings[i]);
  }
  Serial.printf("%0.4f:%0.4f:%0.4f:%0.4f:%0.4f:%0.4f:%0.4f:%0.4f\n", counts[0], counts[1], counts[2], counts[3], counts[6], counts[7], counts[8], counts[9]);

  tempF = (bme.readTemperature() * (9.0/5.0)) + 32.0;
  humidity = (int)bme.readHumidity();
  baroPressure = (bme.readPressure() / 3386.39) + 5;
  Serial.printf("Temp: %0.1f\nHumidity: %i\nBarometric Pressure: %0.2f\n", tempF, humidity, baroPressure);





  snprintf((char *)outgoingMessage, outgoingMessageSize, "%0.1f:%i:%0.2f:%0.4f:%0.4f:%0.4f:%0.4f:%0.4f:%0.4f:%0.4f:%0.4f:\n", tempF, humidity, baroPressure, counts[0], counts[1], counts[2], counts[3], counts[6], counts[7], counts[8], counts[9]);
  txCharacteristic.setValue(outgoingMessage, outgoingMessageSize);
  Serial.printf("Temperature: %0.1f\nHumidity: %i\nBarometric Pressure: %0.2f\nnm415: %0.4f\nnm445: %0.4f\nnm480: %0.4f\nnm515: %0.4f\nnm555: %0.4f\nnm590: %0.4f\nnm630: %0.4f\nnm680: %0.4f\n", tempF, humidity, baroPressure, counts[0], counts[1], counts[2], counts[3], counts[6], counts[7], counts[8], counts[9]);
  BLE.disconnect();
  SystemSleepConfiguration config;
  config.mode(SystemSleepMode::STOP).duration(30000);
  SystemSleepResult result = System.sleep(config);
  if(result.wakeupReason() == SystemSleepWakeupReason::BY_RTC){
    delay(30000);
  }
  // BLE.off();
  // delay(30000);
  // BLE.on();
  // delay(15000);
    }

void getBME(enviroSensors bmeData){
  bmeData.tempF = (bme.readTemperature() * (9.0/5.0)) + 32.0;
  bmeData.humidity = (int)bme.readHumidity();
  bmeData.baroPressure = (bme.readPressure() / 3386.39) + 5;
  Serial.printf("Temp: %0.1f\nHumidity: %i\nBarometric Pressure: %0.2f\n", bmeData.tempF, bmeData.humidity, bmeData.baroPressure);
}












void getAS7341(enviroSensors bmeData){
  if (!as7341.readAllChannels(bmeData.readings)){
    Serial.printf("Error reading all channels!\n");
    //return;
  }

  for(uint8_t i = 0; i < 12; i++) {
    if(i == 4 || i == 5){
      continue;
    }
    // we skip the first set of duplicate clear/NIR readings
    // (indices 4 and 5)
    bmeData.counts[i] = as7341.toBasicCounts(bmeData.readings[i]);
  }
  Serial.printf("nm415: %0.4f\nnm445: %0.4f\nnm480: %0.4f\nnm515: %0.4f\nnm555: %0.4f\nnm590: %0.4f\nnm630: %0.4f\nnm680: %0.4f\n", bmeData.counts[0], bmeData.counts[1], bmeData.counts[2], bmeData.counts[3], bmeData.counts[6], bmeData.counts[7], bmeData.counts[8], bmeData.counts[9]);
}

