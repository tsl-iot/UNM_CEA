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
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT.h" 
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h" 
#include <UNM_CEA_Credentials.h>

SYSTEM_MODE(AUTOMATIC);



// VEML7700 LUX sensor objects
Adafruit_VEML7700 lux_4;
Adafruit_VEML7700 lux_5;
Adafruit_VEML7700 lux_6;
Adafruit_VEML7700 lux_7;

// HDC302x Temp_Hum sensor objects
Adafruit_HDC302x tempHum_0 = Adafruit_HDC302x();
Adafruit_HDC302x tempHum_1 = Adafruit_HDC302x();
Adafruit_HDC302x tempHum_2 = Adafruit_HDC302x();
Adafruit_HDC302x tempHum_3 = Adafruit_HDC302x();

//MQTT config and Feed(s)
TCPClient TheClient; 
Adafruit_MQTT_SPARK mqtt(&TheClient,NODE_RED_SERVER,NODE_RED_SERVERPORT,NODE_RED_USERNAME,NODE_RED_KEY);
Adafruit_MQTT_Publish dataFeed = Adafruit_MQTT_Publish(&mqtt, "cea/dataobject");

// Functions
void get_HDC_T_H(float *temp_0, double *RH_0, float *temp_1, double *RH_1, float *temp_2, double *RH_2, float *temp_3, double *RH_3);
void getLux(float *lux4, float *lux5, float *lux6, float *lux7);
void initHDC320x();
void initVEML7700();
void assignDeviceId();
void createEventPayload();
void MQTT_connect();
void watchdogHandler();
void pcaselect(uint8_t i);
uint64_t millis64bit();

// Variables
const int MULTIPLEX_ADDR = 0x70;
const float DEVICE_ID = 1.00 ;
unsigned int lastDataGrab;
float tempReading_0, tempReading_1, tempReading_2, tempReading_3;
float luxReading_4, luxReading_5, luxReading_6, luxReading_7;
double humidityReading_0, humidityReading_1, humidityReading_2, humidityReading_3;
float gatheredData[13]; // Stores sensor data
String dataTags[13] = {"DevId", "Temp0", "Temp1", "Temp2", "Temp3", "Hum0", "Hum1", "Hum2", "Hum3","Lux4", "Lux5", "Lux6", "Lux7"}; // used for Key in JSON object


// And so begins the program
void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected, 5000);
  Wire.begin();
  delay(1000);
  while(!WiFi.ready()){
    Serial.printf(" . ");
    delay(100);
  }
  initHDC320x();
  initVEML7700();
  delay(2500);
}

void loop() {
  MQTT_connect();
  if((millis() - lastDataGrab) > 30000){
    assignDeviceId();
    get_HDC_T_H(&tempReading_0, &humidityReading_0, &tempReading_1, &humidityReading_1, &tempReading_2, &humidityReading_2, &tempReading_3, &humidityReading_3); 
    delay(500);
    getLux(&luxReading_4, &luxReading_5, &luxReading_6, &luxReading_7);
    delay(500);
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

void get_HDC_T_H(float *temp_0, double *RH_0, float *temp_1, double *RH_1, float *temp_2, double *RH_2, float *temp_3, double *RH_3){
  double temp0, temp1, temp2, temp3;
              //                                                                | | | |
  pcaselect(0); // Talking to device connected to SD0 & SC0 on the Multiplexer  V V V V  and so on...
  tempHum_0.readTemperatureHumidityOnDemand(temp0, *RH_0, TRIGGERMODE_LP0);
  pcaselect(1);
  tempHum_1.readTemperatureHumidityOnDemand(temp1, *RH_1, TRIGGERMODE_LP0);
  pcaselect(2);
  tempHum_2.readTemperatureHumidityOnDemand(temp2, *RH_2, TRIGGERMODE_LP0);
  pcaselect(3);
  tempHum_3.readTemperatureHumidityOnDemand(temp3, *RH_3, TRIGGERMODE_LP0);

  *temp_0 = temp0;
  gatheredData[1] = *temp_0; // Fill the data buffer

  *temp_1 = temp1;
  gatheredData[2] = *temp_1;

  *temp_2 = temp2;
  gatheredData[3] = *temp_2;

  *temp_3 = temp3;
  gatheredData[4] = *temp_3;

  gatheredData[5] = *RH_0;
  gatheredData[6] = *RH_1;
  gatheredData[7] = *RH_2;
  gatheredData[8] = *RH_3;
 

  //Serial.printf("Temp_0: %0.1f\nRH_0: %0.1f\n\nTemp_1: %0.1f\nRH_1: %0.1f\n\nTemp_2: %0.1f\nRH_2: %0.1f\n\nTemp_3: %0.1f\nRH_3: %0.1f\n\n", *temp_0, *RH_0, *temp_1, *RH_1,*temp_2, *RH_2, *temp_3, *RH_3);
  
}

// Calculates and returns the LUX values for devices 4 - 7 on the I2C multiplexer
void getLux(float *lux4, float *lux5, float *lux6, float *lux7){
  
  pcaselect(4);
  *lux4 = (lux_4.readALS() * 0.110779);  // Light level [lx] is: OUTPUT DATA [dec.] / ALS sensitivity) x (10 / IT [ms]) ---The exact integration time is 90 ms, so the factor should not be 0.1 but 0.110779
  gatheredData[9] = *lux4;

  pcaselect(5);
  *lux5 = (lux_5.readALS() * 0.110779);
  gatheredData[10] = *lux5;

  pcaselect(6);
  *lux6 = (lux_6.readALS() * 0.110779);
  gatheredData[11] = *lux6;

  pcaselect(7);
  *lux7 = (lux_7.readALS() * 0.110779);
  gatheredData[12] = *lux7;
  //Serial.printf("Lux sensor 4: %0.4f lx\n\nLux sensor 5: %0.4f lx\n\nLux sensor 6: %0.4flx\n\nLux sensor 7: %0.4flx\n\n", *lux4, *lux5, *lux6, *lux7);
}

// Creates a json object with 12+ Key:Value pairs
void createEventPayload(){

  JsonWriterStatic<256> jw;
  {
	  JsonWriterAutoObject obj(&jw);
      for(int i = 0; i < 13; i++){
        jw.insertKeyValue(dataTags[i], gatheredData[i]);
      }
	  }
    if(mqtt.Update()){
    dataFeed.publish(jw.getBuffer());
    Serial.printf("Published: %s\n\n\n", jw.getBuffer());
  }
}

void assignDeviceId(){
  gatheredData[0] = DEVICE_ID;
}

void watchdogHandler(){
  System.reset(RESET_NO_WAIT);
}

void MQTT_connect() {
  int8_t ret;
  if (mqtt.connected()) {
    
    return;
  }
  Serial.print("Connecting to MQTT... ");
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.printf("%s\n",(char *)mqtt.connectErrorString(ret));
    Serial.printf("Retrying MQTT connection in 5 seconds..\n");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
  }
  Serial.printf("MQTT Connected!\n");
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

