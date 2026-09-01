/* 
 * Read LUX from 8 VEML7700 sensors
 * Author: Edward Ishman @ CNM Ingenuity
 * Date: 09/01/2026
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

#include "Particle.h"
#include "Adafruit_VEML7700.h"
SYSTEM_MODE(AUTOMATIC);

// VEML7700 LUX sensor objects
Adafruit_VEML7700 lux_1;
Adafruit_VEML7700 lux_2;
Adafruit_VEML7700 lux_3;
Adafruit_VEML7700 lux_4;
Adafruit_VEML7700 lux_5;
Adafruit_VEML7700 lux_6;
Adafruit_VEML7700 lux_7;
Adafruit_VEML7700 lux_8;

void getLux(float *lux1, float *lux2, float *lux3, float *lux4, float *lux5, float *lux6, float *lux7, float *lux8);
void initVEML7700();
void watchdogHandler();
void pcaselect(uint8_t i);
uint64_t millis64bit();

const int MULTIPLEX_ADDR = 0x70;
uint64_t lastDataGrab;
float luxReading_1, luxReading_2, luxReading_3, luxReading_4, luxReading_5, luxReading_6, luxReading_7, luxReading_8;
float gatheredData[8]; // Stores sensor data
String dataTags[8] = {"LUX_1", "LUX_2", "LUX_3", "LUX_4", "LUX_5", "LUX_6", "LUX_7", "LUX_8"}; // used for Key in JSON object

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
  delay(2500);

}

void loop() {
  if((millis() - lastDataGrab) > 30000){
    getLux(&luxReading_1, &luxReading_2, &luxReading_3, &luxReading_4, &luxReading_5, &luxReading_6, &luxReading_7, &luxReading_8);
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

void initVEML7700(){
//-------------------------
  pcaselect(0);

  if(!lux_1.begin()){
    Serial.printf("Lux sensor 1 FAILED TO START!\n");

    
  }
  else{
    Serial.printf("Lux sensor 1 successfully started\n");
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

  pcaselect(4);

  if(!lux_5.begin()){
    Serial.printf("Lux sensor 5 FAILED TO START!\n");

    
  }
  else{
    Serial.printf("Lux sensor 5 successfully started\n");
    lux_5.setGain(VEML7700_GAIN_1_8);
    lux_5.setIntegrationTime(VEML7700_IT_100MS);

  }
//-----------------------
  pcaselect(5);

  if(!lux_6.begin()){
    Serial.printf("Lux sensor 6 FAILED TO START!\n");

  }
  else{
    Serial.printf("Lux sensor 6 successfully started\n");
    lux_6.setGain(VEML7700_GAIN_1_8);
    lux_6.setIntegrationTime(VEML7700_IT_100MS);

  }
//---------------------------
  pcaselect(6);

  if(!lux_7.begin()){
    Serial.printf("Lux sensor 7 FAILED TO START!\n");

  }
  else{
    Serial.printf("Lux sensor 7 successfully started\n");
    lux_7.setGain(VEML7700_GAIN_1_8);
    lux_7.setIntegrationTime(VEML7700_IT_100MS);

  }
//----------------------------
  pcaselect(7);

  if(!lux_8.begin()){
    Serial.printf("Lux sensor 8 FAILED TO START!\n");

  }
  else{
    Serial.printf("Lux sensor 8 successfully started\n");
    lux_8.setGain(VEML7700_GAIN_1_8);
    lux_8.setIntegrationTime(VEML7700_IT_100MS);

  }
}


// Calculates and returns the LUX values for devices 4 - 7 on the I2C multiplexer
void getLux(float *lux1, float *lux2, float *lux3, float *lux4, float *lux5, float *lux6, float *lux7, float *lux8){
  
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
  *lux5 = (lux_5.readALS() * 0.110779);  // Light level [lx] is: OUTPUT DATA [dec.] / ALS sensitivity) x (10 / IT [ms]) ---The exact integration time is 90 ms, so the factor should not be 0.1 but 0.110779
  gatheredData[4] = *lux5;

  pcaselect(5);
  *lux6 = (lux_6.readALS() * 0.110779);
  gatheredData[5] = *lux6;

  pcaselect(6);
  *lux7 = (lux_7.readALS() * 0.110779);
  gatheredData[6] = *lux7;

  pcaselect(7);
  *lux8 = (lux_8.readALS() * 0.110779);
  gatheredData[7] = *lux8;
  Serial.printf("Lux sensor 1: %0.4f lx\n\nLux sensor 2: %0.4f lx\n\nLux sensor 3: %0.4flx\n\nLux sensor 4: %0.4flx\n\nLux sensor 5: %0.4f lx\n\nLux sensor 6: %0.4f lx\n\nLux sensor 7: %0.4flx\n\nLux sensor 8: %0.4flx\n\n",*lux1, *lux2, *lux3, *lux4, *lux5, *lux6, *lux7, *lux8);
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



void watchdogHandler(){
  System.reset(RESET_NO_WAIT);
}
