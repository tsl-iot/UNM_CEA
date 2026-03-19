/* 
 * Project: Controlled Environment Agriculture Base Station
 * Author: CNM TSL
 * Date: 3/5/2026
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 * -----------------------------------------------------------------------
 **Use the device id for cloud flashing**
 * EJ Device Info
 *  Your device id is 0a10aced202194944a051cc4
 *  Your system firmware version is 5.9.0
 * PB Device Info
 * 	Your device id is 0a10aced202194944a0522d0
 *  Your system firmware version is 6.3.4
 */

/*Installed Libraries
 Adafruit_SSD1306 (a/o 3/5/2026)
 Adafruit_BME280 (a/o 3/5/2026)
 Adafruit_AS7341 (a/o 3/5/2026)
 Adafruit_MQTT (a/o 3/5/2026)
 Adafruit_BusIO_Register (a/o 3/5/2026)
 IoTClassroom_CNM (a/o 3/5/2026)
*/

#include "Particle.h"
//BASE STATION//
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT.h" 
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h" 
//#include <UNM_CEA_Credentials.h>
//PERIPHERAL//
#include "Adafruit_BME280.h"
#include "Adafruit_AS7341.h"

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(SEMI_AUTOMATIC);

//BLE-UART Service
const BleUuid serviceUuid("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
const BleUuid txUuid("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");
BleCharacteristic txCharacteristic("tx",BleCharacteristicProperty::INDICATE,txUuid,serviceUuid);
BleAdvertisingData data;

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

unsigned int lastRead;
const int SUBD7PIN=A2;

/******************************************************************/
void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected,5000);

  BLE.on();
  BLE.addCharacteristic(txCharacteristic);
  data.appendServiceUUID(txUuid);
  data.appendLocalName("3"); // change this to your device number
  BLE.advertise(&data);
  BLE.setTxPower(8);

  pinMode(SUBD7PIN,OUTPUT);
  digitalWrite(SUBD7PIN, LOW);

  if(!bme.begin(0x77)){
    Serial.printf("Could not start BME280\n");
  }
  else{
    Serial.printf("Successfully started BME280\n");
  }

  if(!as7341.begin()){
    Serial.printf("Could not start AS7341\n");
  }
  else{
    Serial.printf("Successfully started AS7341\n");
  }

  /* ASK FOR HELP UNDERSTANDING THE "as7341.set" LINES*/
  Serial.printf("Muon BLE Address: %s\n", BLE.address().toString().c_str());
  delay(5000);
  as7341.setATIME(100);
  as7341.setASTEP(999);
  as7341.setGain(AS7341_GAIN_64X);
}


/******************************************************************/
void loop () {
  if(BLE.connected()){
    delay(5000);
    sendThenDisconnect();
  }
  else{
    digitalWrite(SUBD7PIN,LOW);
    if((millis() - lastRead) > 5000){
      lastRead = millis();
      getBME(enviroData);
      getAS7341(enviroData);
    }
  }
}


/******************************************************************/
void sendThenDisconnect() {
  const int outgoingMessageSize = 120;
  uint8_t outgoingMessage[outgoingMessageSize];
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

  snprintf((char *)outgoingMessage, outgoingMessageSize, "%0.1f:%i:%0.2f:%0.5f:%0.5f:%0.5f:%0.5f:%0.5f:%0.5f:%0.5f:%0.5f:\n", tempF, humidity, baroPressure, counts[0], counts[1], counts[2], counts[3], counts[6], counts[7], counts[8], counts[9]);
  txCharacteristic.setValue(outgoingMessage, outgoingMessageSize);
  Serial.printf("Temperature: %0.1f\nHumidity: %i\nBarometric Pressure: %0.2f\nnm415: %0.5f\nnm445: %0.5f\nnm480: %0.5f\nnm515: %0.5f\nnm555: %0.5f\nnm590: %0.5f\nnm630: %0.5f\nnm680: %0.5f\n", tempF, humidity, baroPressure, counts[0], counts[1], counts[2], counts[3], counts[6], counts[7], counts[8], counts[9]);
  BLE.disconnect();
  SystemSleepConfiguration config;
  config.mode(SystemSleepMode::STOP).duration(30000);
  SystemSleepResult result = System.sleep(config);
  if(result.wakeupReason() == SystemSleepWakeupReason::BY_RTC){
    delay(30000);
  }
}


/*****************************************************************/
void getBME(enviroSensors bmeData) {
  bmeData.tempF = (bme.readTemperature() * (9.0/5.0)) + 32.0;
  bmeData.humidity = (int)bme.readHumidity();
  bmeData.baroPressure = (bme.readPressure() / 3386.39) + 5;
  Serial.printf("Temp: %0.1f\nHumidity: %i\nBarometric Pressure: %0.2f\n", bmeData.tempF, bmeData.humidity, bmeData.baroPressure);

}


/*****************************************************************/
void getAS7341(enviroSensors bmeData) {
  if (!as7341.readAllChannels(bmeData.readings)){
    Serial.printf("Error reading all channels!\n");
  }
  
  for(uint8_t i = 0; i < 12; i++) {
    if(i == 4 || i == 5){
      continue;
    }
    // we skip the first set of duplicate clear/NIR readings
    // (indices 4 and 5)
    bmeData.counts[i] = as7341.toBasicCounts(bmeData.readings[i]);
  }
  Serial.printf("nm415Violet: %0.4f\nnm445Indigo: %0.4f\nnm480Blue: %0.4f\nnm515Cyan: %0.4f\nnm555Green: %0.4f\nnm590Yellow: %0.4f\nnm630Orange: %0.4f\nnm680Red: %0.4f\n", bmeData.counts[0], bmeData.counts[1], bmeData.counts[2], bmeData.counts[3], bmeData.counts[6], bmeData.counts[7], bmeData.counts[8], bmeData.counts[9]);
}