/* 
 * Read LUX, Temp in Celsius, and Relative Humidity. Print to Serial monitor and publish to Adafruit.io
 * Author: Edward Ishman @ CNM Ingenuity
 * Date: 09/11/2026
 * 
 * Datasheets below:
 * VEML7700 - https://www.vishay.com/docs/84286/veml7700.pdf
 * BME280 - https://cdn-learn.adafruit.com/assets/assets/000/115/588/original/bst-bme280-ds002.pdf?1664822559
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "Adafruit_VEML7700.h"
#include "Adafruit_BME280.h"
#include "JsonParserGeneratorRK.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT.h" 
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h" 
#include <UNM_CEA_Credentials.h>

SYSTEM_MODE(AUTOMATIC);

// VEML7700 LUX sensor objects
Adafruit_VEML7700 lux_1;
Adafruit_VEML7700 lux_2;
Adafruit_VEML7700 lux_3;
Adafruit_VEML7700 lux_4;

Adafruit_BME280 bme_1;
Adafruit_BME280 bme_2;
Adafruit_BME280 bme_3;
Adafruit_BME280 bme_4;

void initVEML7700();
void initBME280();
//void watchdogHandler();
void pcaselect(uint8_t i);
uint64_t millis64bit();
void getSensorData(float *lux1, float *lux2, float *lux3, float *lux4, float *temp1, float *temp2, float *temp3, float *temp4, float *hum1, float *hum2, float *hum3, float *hum4);
void createEventPayload();

const int MULTIPLEX_ADDR = 0x70;
uint64_t lastDataGrab;
float luxReading_1, luxReading_2, luxReading_3, luxReading_4, bmeTemp_1, bmeTemp_2, bmeTemp_3, bmeTemp_4, bmeHum_1, bmeHum_2, bmeHum_3, bmeHum_4;
float gatheredData[12]; // Stores sensor data
String dataTags[12] = {"LUX_1", "LUX_2", "LUX_3", "LUX_4", "BME_T_1", "BME_T_2", "BME_T_3", "BME_T_4", "BME_H_1", "BME_H_2", "BME_H_3", "BME_H_4" }; // used for Key in JSON object


//MQTT config and Feed(s)
TCPClient TheClient; 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY);
Adafruit_MQTT_Publish dataFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/ceadataobject");



void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected, 5000);
  Wire.begin();
  delay(1000);
  while(!WiFi.ready()){
    Serial.printf(" . ");
    delay(100);
  }
  initVEML7700(); 
  delay(100);
  initBME280();
  delay(2500);


}

void loop() {
  if((millis() - lastDataGrab) > 30000){
    getSensorData(&luxReading_1, &luxReading_2, &luxReading_3, &luxReading_4, &bmeTemp_1, &bmeTemp_2, &bmeTemp_3, &bmeTemp_4, &bmeHum_1, &bmeHum_2, &bmeHum_3, &bmeHum_4);
    //Serial.printf("lux 1: %0.1f\n\nlux 2: %0.1f\n\nlux 3: %0.1f\n\nlux 4: %0.1f\n\ntemp 1: %0.1f\n\ntemp 2: %0.1f\n\ntemp 3: %0.1f\n\ntemp 4: %0.1f\n\nhum 1: %0.1f\n\nhum 2: %0.1f\n\nhum 3: %0.1f\n\nhum 4: %0.1f\n\n", luxReading_1, luxReading_2, luxReading_3, luxReading_4, bmeTemp_1, bmeTemp_2, bmeTemp_3, bmeTemp_4, bmeHum_1, bmeHum_2, bmeHum_3, bmeHum_4);
    createEventPayload();
    delay(500);
    lastDataGrab = millis64bit();
  }

}


//Select which multiplexer port to communicate with
void pcaselect(uint8_t i) {
  if (i > 7){
    return;
  }
  Wire.beginTransmission(MULTIPLEX_ADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

//Initializes each VEML7700 sensor
void initVEML7700(){
//-------------------------
  pcaselect(0);

  if(!lux_1.begin()){
    Serial.printf("\nLux sensor 1 FAILED TO START!\n");
  }
  else{
    Serial.printf("\nLux sensor 1 successfully started\n");
    lux_1.setGain(VEML7700_GAIN_1_8);
    lux_1.setIntegrationTime(VEML7700_IT_100MS);
  }
//-----------------------
  pcaselect(1);

  if(!lux_2.begin()){
    Serial.printf("Lux sensor 2 FAILED TO START!\n");
  }
  else{
    Serial.printf("Lux sensor 2 successfully started\n");
    lux_2.setGain(VEML7700_GAIN_1_8);
    lux_2.setIntegrationTime(VEML7700_IT_100MS);
  }
//---------------------------
  pcaselect(2);

  if(!lux_3.begin()){
    Serial.printf("Lux sensor 3 FAILED TO START!\n");
  }
  else{
    Serial.printf("Lux sensor 3 successfully started\n");
    lux_3.setGain(VEML7700_GAIN_1_8);
    lux_3.setIntegrationTime(VEML7700_IT_100MS);
  }
//----------------------------
  pcaselect(3);

  if(!lux_4.begin()){
    Serial.printf("Lux sensor 4 FAILED TO START!\n");
  }
  else{
    Serial.printf("Lux sensor 4 successfully started\n");
    lux_4.setGain(VEML7700_GAIN_1_8);
    lux_4.setIntegrationTime(VEML7700_IT_100MS);
  }
}

void initBME280(){
  pcaselect(4);

  if(!bme_1.begin(0x76)){
    Serial.printf("BME 1 FAILED TO START!\n");
  }
  else{
    Serial.printf("BME 1 successfully started\n");
  }
//-----------------------
  pcaselect(5);

  if(!bme_2.begin(0x76)){
    Serial.printf("BME 2 FAILED TO START!\n");
  }
  else{
    Serial.printf("BME 2 successfully started\n");
  }
//---------------------------
  pcaselect(6);

  if(!bme_3.begin(0x76)){
    Serial.printf("BME 3 FAILED TO START!\n");
  }
  else{
    Serial.printf("BME 3 successfully started\n");
  }
//----------------------------
  pcaselect(7);

  if(!bme_4.begin(0x76)){
    Serial.printf("BME 4 FAILED TO START!\n");
  }
  else{
    Serial.printf("BME 4 successfully started\n");
  }
}

void getSensorData(float *lux1, float *lux2, float *lux3, float *lux4, float *temp1, float *temp2, float *temp3, float *temp4, float *hum1, float *hum2, float *hum3, float *hum4){
  pcaselect(0);
  *lux1 = (lux_1.readALS() * 0.110779);  // Light level [lx] is: OUTPUT DATA [dec.] / ALS sensitivity) x (10 / IT [ms]) ---The exact integration time is 90 ms, so the factor should not be 0.1 but 0.110779
  gatheredData[0] = *lux1;

  pcaselect(1);
  *lux2 = (lux_2.readALS() * 0.110779);
  gatheredData[1] = *lux2;

  pcaselect(2);
  *lux3 = (lux_3.readALS() * 0.110779);
  gatheredData[2] = *lux3;

  pcaselect(3);
  *lux4 = (lux_4.readALS() * 0.110779);
  gatheredData[3] = *lux4;

  pcaselect(4);
  *temp1 = bme_1.readTemperature();  
  gatheredData[4] = *temp1;

  pcaselect(5);
  *temp2 = bme_2.readTemperature();  
  gatheredData[5] = *temp2;

  pcaselect(6);
  *temp3 = bme_3.readTemperature();  
  gatheredData[6] = *temp3;

  pcaselect(7);
  *temp4 = bme_4.readTemperature();  
  gatheredData[7] = *temp4;

  pcaselect(4);
  *hum1 = bme_1.readHumidity();  
  gatheredData[8] = *hum1;

  pcaselect(5);
  *hum2 = bme_2.readHumidity();  
  gatheredData[9] = *hum2;

  pcaselect(6);
  *hum3 = bme_3.readHumidity();  
  gatheredData[10] = *hum3;

  pcaselect(7);
  *hum4 = bme_4.readHumidity();  
  gatheredData[11] = *hum4;
}


// Creates a json object with 12+ Key:Value pairs
void createEventPayload(){

  JsonWriterStatic<256> jw;
  {
	  JsonWriterAutoObject obj(&jw);

    for(int i = 0; i < 12; i++){
      jw.insertKeyValue(dataTags[i], gatheredData[i]);
    }
  }

  if(mqtt.Update()){
  dataFeed.publish(jw.getBuffer());
  Serial.printf("Published: %s\n\n\n", jw.getBuffer());
  }
}

uint64_t millis64bit() {
    static uint32_t low4bytes, high4bytes;
    uint32_t newMillis;

    newMillis = millis();
    if (newMillis < low4bytes) {    //check if millis has rolled over
       high4bytes++;                //if so, add one to high bytes
    }
    low4bytes = newMillis;
    return (high4bytes << 32 | low4bytes); //return 64-bit (8-byte) millis
}
