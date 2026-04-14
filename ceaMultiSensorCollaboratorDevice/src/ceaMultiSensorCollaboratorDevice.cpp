/* 
 * Controlled Environment Agriculture Collaborator Device
 * Author: Edward Ishman
 * Date: 04/14/2026
 * Collect data from temp/hum, and light(lux) sensors. 8 sensors total. Then wrap in an JSON object to publish to Node-Red Dashboard
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "Adafruit_HDC302x.h"
#include "Adafruit_VEML7700.h"
#include "JsonParserGeneratorRK.h"

SYSTEM_MODE(SEMI_AUTOMATIC);

const int MULTIPLEX_ADDR = 0x70;
unsigned int lastDataGrab;

float tempReading_0, tempReading_1, tempReading_2, tempReading_3;
float luxReading_4, luxReading_5, luxReading_6, luxReading_7;
double humidityReading_0, humidityReading_1, humidityReading_2, humidityReading_3;

String dataTags[12] = {"Temp_0", "Temp_1", "Temp_2", "Temp_3", "Hum_0", "Hum_1", "Hum_2", "Hum_3","Lux_4", "Lux_5", "Lux_6", "Lux_7"};
float gatheredData[12];
//float gatheredData[12] = {tempReading_0, tempReading_1, tempReading_2, tempReading_3, (double)humidityReading_0, (double)humidityReading_1, (double)humidityReading_2, (double)humidityReading_3, luxReading_4, luxReading_5, luxReading_6, luxReading_7 };


Adafruit_HDC302x tempHum_0 = Adafruit_HDC302x();
Adafruit_HDC302x tempHum_1 = Adafruit_HDC302x();
Adafruit_HDC302x tempHum_2 = Adafruit_HDC302x();
Adafruit_HDC302x tempHum_3 = Adafruit_HDC302x();

Adafruit_VEML7700 lux_4;
Adafruit_VEML7700 lux_5;
Adafruit_VEML7700 lux_6;
Adafruit_VEML7700 lux_7;

void get_HDC_T_H(float *temp_0, double *RH_0, float *temp_1, double *RH_1, float *temp_2, double *RH_2, float *temp_3, double *RH_3);
void getLux(float *lux4, float *lux5, float *lux6, float *lux7);
void initHDC320x();
void initVEML7700();
void createEventPayload();
void pcaselect(uint8_t i);

void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected, 5000);
  Wire.begin();
  delay(1000);
  initHDC320x();
  initVEML7700();
  delay(2500);
}

void loop() {
  if((millis() - lastDataGrab) > 30000){
    get_HDC_T_H(&tempReading_0, &humidityReading_0, &tempReading_1, &humidityReading_1, &tempReading_2, &humidityReading_2, &tempReading_3, &humidityReading_3); 
    delay(1000);
    getLux(&luxReading_4, &luxReading_5, &luxReading_6, &luxReading_7);
    delay(1000);
    createEventPayload();
    lastDataGrab = millis();
  }
}

// Selects the I2C device to talk to
void pcaselect(uint8_t i) {
  if (i > 7){
     return;
  }
  Wire.beginTransmission(MULTIPLEX_ADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

// Measures Temperature and relative humidity from sensors 0 - 3 on the I2C multiplexer
void get_HDC_T_H(float *temp_0, double *RH_0, float *temp_1, double *RH_1, float *temp_2, double *RH_2, float *temp_3, double *RH_3){
  double temp0, temp1, temp2, temp3;

  pcaselect(0);
  tempHum_0.readTemperatureHumidityOnDemand(temp0, *RH_0, TRIGGERMODE_LP0);
  pcaselect(1);
  tempHum_1.readTemperatureHumidityOnDemand(temp1, *RH_1, TRIGGERMODE_LP0);
  pcaselect(2);
  tempHum_2.readTemperatureHumidityOnDemand(temp2, *RH_2, TRIGGERMODE_LP0);
  pcaselect(3);
  tempHum_3.readTemperatureHumidityOnDemand(temp3, *RH_3, TRIGGERMODE_LP0);

  *temp_0 = temp0;
  gatheredData[0] = *temp_0;

  *temp_1 = temp1;
  gatheredData[1] = *temp_1;

  *temp_2 = temp2;
  gatheredData[2] = *temp_2;

  *temp_3 = temp3;
  gatheredData[3] = *temp_3;

  gatheredData[4] = *RH_0;
  gatheredData[5] = *RH_1;
  gatheredData[6] = *RH_2;
  gatheredData[7] = *RH_3;
 

  //Serial.printf("Temp_0: %0.1f\nRH_0: %0.1f\n\nTemp_1: %0.1f\nRH_1: %0.1f\n\nTemp_2: %0.1f\nRH_2: %0.1f\n\nTemp_3: %0.1f\nRH_3: %0.1f\n\n", *temp_0, *RH_0, *temp_1, *RH_1,*temp_2, *RH_2, *temp_3, *RH_3);
  
}

// Calculates and returns the LUX values for devices 4 - 7 on the I2C multiplexer
void getLux(float *lux4, float *lux5, float *lux6, float *lux7){
  
  pcaselect(4);
  *lux4 = (lux_4.readALS() * 0.110779);  // Light level [lx] is: OUTPUT DATA [dec.] / ALS sensitivity) x (10 / IT [ms]) ---The exact integration time is 90 ms, so the factor should not be 0.1 but 0.110779
  gatheredData[8] = *lux4;

  pcaselect(5);
  *lux5 = (lux_5.readALS() * 0.110779);
  gatheredData[9] = *lux5;

  pcaselect(6);
  *lux6 = (lux_6.readALS() * 0.110779);
  gatheredData[10] = *lux6;

  pcaselect(7);
  *lux7 = (lux_7.readALS() * 0.110779);
  gatheredData[11] = *lux7;
  //Serial.printf("Lux sensor 4: %0.4f lx\n\nLux sensor 5: %0.4f lx\n\nLux sensor 6: %0.4flx\n\nLux sensor 7: %0.4flx\n\n", *lux4, *lux5, *lux6, *lux7);


}

void initHDC320x(){
//--------------------------
  pcaselect(0);

  if(!tempHum_0.begin(0x44)){
    Serial.printf("Temp/Hum sensor 0 FAILED TO START!\n");
  }
  else{
    Serial.printf("Temp/Hum sensor 0 successfully started\n");
  }
//-----------------------------------
  pcaselect(1);

  if(!tempHum_1.begin(0x44)){
    Serial.printf("Temp/Hum sensor 1 FAILED TO START!\n");
  }
  else{
    Serial.printf("Temp/Hum sensor 1 successfully started\n");
  }
//------------------------------------
  pcaselect(2);

  if(!tempHum_2.begin(0x44)){
    Serial.printf("Temp/Hum sensor 2 FAILED TO START!\n");
  }
  else{
    Serial.printf("Temp/Hum sensor 2 successfully started\n");
  }
//----------------------------------------
  pcaselect(3);

  if(!tempHum_3.begin(0x44)){
    Serial.printf("Temp/Hum sensor 3 FAILED TO START!\n");
  }
  else{
    Serial.printf("Temp/Hum sensor 3 successfully started\n");
  }
}

void initVEML7700(){
//-------------------------
  pcaselect(4);

  if(!lux_4.begin()){
    Serial.printf("Lux sensor 4 FAILED TO START!\n");
  }
  else{
    Serial.printf("Lux sensor 4 successfully started\n");
    lux_4.setGain(VEML7700_GAIN_1_8);
    lux_4.setIntegrationTime(VEML7700_IT_100MS);
  }
//-----------------------
  pcaselect(5);

  if(!lux_5.begin()){
    Serial.printf("Lux sensor 5 FAILED TO START!\n");
  }
  else{
    Serial.printf("Lux sensor 5 successfully started\n");
    lux_5.setGain(VEML7700_GAIN_1_8);
    lux_5.setIntegrationTime(VEML7700_IT_100MS);
  }
//---------------------------
  pcaselect(6);

  if(!lux_6.begin()){
    Serial.printf("Lux sensor 6 FAILED TO START!\n");
  }
  else{
    Serial.printf("Lux sensor 6 successfully started\n");
    lux_6.setGain(VEML7700_GAIN_1_8);
    lux_6.setIntegrationTime(VEML7700_IT_100MS);
  }
//----------------------------
  pcaselect(7);

  if(!lux_7.begin()){
    Serial.printf("Lux sensor 7 FAILED TO START!\n");
  }
  else{
    Serial.printf("Lux sensor 7 successfully started\n");
    lux_7.setGain(VEML7700_GAIN_1_8);
    lux_7.setIntegrationTime(VEML7700_IT_100MS);
  }
}

void createEventPayload(){
  JsonWriterStatic<256> jw;
  {
	  JsonWriterAutoObject obj(&jw);
      for(int i = 0; i < 12; i++){
        jw.insertKeyValue(dataTags[i], gatheredData[i]);
      }
	    Serial.printf("%s\n", jw.getBuffer());
	  }
}


