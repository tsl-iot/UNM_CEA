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
/* Multiplexer Config
Area 1
  SD0 & SC0 -- Temp/hum
  SD1 & SC1 -- Lux
Area 2
  SD2 & SC2 -- Temp/hum
  SD3 & SC3 -- Lux  
Area 3
  SD4 & SC4 -- Temp/hum
  SD5 & SC5 -- Lux
Area 4
  SD6 & SC6 -- Temp/hum
  SD7 & SC7 -- Lux  
*/


// VEML7700 LUX sensor objects
Adafruit_VEML7700 lux_1;
Adafruit_VEML7700 lux_2;
Adafruit_VEML7700 lux_3;
Adafruit_VEML7700 lux_4;

// HDC302x Temp_Hum sensor objects
Adafruit_HDC302x tempHum_1 = Adafruit_HDC302x();
Adafruit_HDC302x tempHum_2 = Adafruit_HDC302x();
Adafruit_HDC302x tempHum_3 = Adafruit_HDC302x();
Adafruit_HDC302x tempHum_4 = Adafruit_HDC302x();

//MQTT config and Feed(s)
TCPClient TheClient; 
Adafruit_MQTT_SPARK mqtt(&TheClient,NODE_RED_SERVER,NODE_RED_SERVERPORT,NODE_RED_USERNAME,NODE_RED_KEY);
Adafruit_MQTT_Publish dataFeed = Adafruit_MQTT_Publish(&mqtt, "cea/dataobject");

// Functions
void get_HDC_T_H(float *temp_1, double *RH_1, float *temp_2, double *RH_2, float *temp_3, double *RH_3, float *temp_4, double *RH_4);
void getLux(float *lux1, float *lux2, float *lux3, float *lux4);
void initHDC320x();
void initVEML7700();
void assignDeviceId();
void createEventPayload();
void MQTT_connect();
void watchdogHandler();
void bufferElementBypass(int *arr);
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
String dataTags[13] = {"DevId", "Temp1", "Temp2", "Temp3", "Temp4", "Hum1", "Hum2", "Hum3", "Hum4","Lux1", "Lux2", "Lux3", "Lux4"}; // used for Key in JSON object
bool activeArea1, activeArea2, activeArea3, activeArea4;
int bypassArr[12];
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

  if(!tempHum_1.begin(0x44)){
    Serial.printf("Temp/Hum sensor 0 FAILED TO START!\n");
    activeArea1 = false;
  }
  else{
    Serial.printf("Temp/Hum sensor 0 successfully started\n");
    activeArea1 = true;
  }
//-----------------------------------
  pcaselect(2);

  if(!tempHum_2.begin(0x44)){
    Serial.printf("Temp/Hum sensor 1 FAILED TO START!\n");
    activeArea2 = false;
  }
  else{
    Serial.printf("Temp/Hum sensor 1 successfully started\n");
    activeArea2 = true;
  }
//------------------------------------
  pcaselect(4);

  if(!tempHum_3.begin(0x44)){
    Serial.printf("Temp/Hum sensor 2 FAILED TO START!\n");
    activeArea3 = false;
  }
  else{
    Serial.printf("Temp/Hum sensor 2 successfully started\n");
    activeArea3 = true;
  }
//----------------------------------------
  pcaselect(6);

  if(!tempHum_4.begin(0x44)){
    Serial.printf("Temp/Hum sensor 3 FAILED TO START!\n");
    activeArea4 = false;
  }
  else{
    Serial.printf("Temp/Hum sensor 3 successfully started\n");
    activeArea4 = true;
  }
}

void initVEML7700(){
//-------------------------
  pcaselect(1);

  if(!lux_1.begin()){
    Serial.printf("Lux sensor 4 FAILED TO START!\n");
    activeArea1 = false;
    
  }
  else{
    Serial.printf("Lux sensor 4 successfully started\n");
    lux_1.setGain(VEML7700_GAIN_1_8);
    lux_1.setIntegrationTime(VEML7700_IT_100MS);
    activeArea1 = true;
  }
//-----------------------
  pcaselect(3);

  if(!lux_2.begin()){
    Serial.printf("Lux sensor 5 FAILED TO START!\n");
    activeArea2 = false;
  }
  else{
    Serial.printf("Lux sensor 5 successfully started\n");
    lux_2.setGain(VEML7700_GAIN_1_8);
    lux_2.setIntegrationTime(VEML7700_IT_100MS);
    activeArea2 = true;
  }
//---------------------------
  pcaselect(5);

  if(!lux_3.begin()){
    Serial.printf("Lux sensor 6 FAILED TO START!\n");
    activeArea3 = false;
  }
  else{
    Serial.printf("Lux sensor 6 successfully started\n");
    lux_3.setGain(VEML7700_GAIN_1_8);
    lux_3.setIntegrationTime(VEML7700_IT_100MS);
    activeArea3 = true;
  }
//----------------------------
  pcaselect(7);

  if(!lux_4.begin()){
    Serial.printf("Lux sensor 7 FAILED TO START!\n");
    activeArea4 = false;
  }
  else{
    Serial.printf("Lux sensor 7 successfully started\n");
    lux_4.setGain(VEML7700_GAIN_1_8);
    lux_4.setIntegrationTime(VEML7700_IT_100MS);
    activeArea4 = true;
  }
}


void get_HDC_T_H(float *temp_1, double *RH_1, float *temp_2, double *RH_2, float *temp_3, double *RH_3, float *temp_4, double *RH_4){
  double temp0, temp1, temp2, temp3;
              //                                                                | | | |
  pcaselect(0); // Talking to device connected to SD0 & SC0 on the Multiplexer  V V V V  and so on...
  tempHum_1.readTemperatureHumidityOnDemand(temp0, *RH_1, TRIGGERMODE_LP0);
  pcaselect(2);
  tempHum_2.readTemperatureHumidityOnDemand(temp1, *RH_2, TRIGGERMODE_LP0);
  pcaselect(4);
  tempHum_3.readTemperatureHumidityOnDemand(temp2, *RH_3, TRIGGERMODE_LP0);
  pcaselect(6);
  tempHum_4.readTemperatureHumidityOnDemand(temp3, *RH_4, TRIGGERMODE_LP0);

  *temp_1 = temp0;
  gatheredData[1] = *temp_1; // Fill the data buffer

  *temp_2 = temp1;
  gatheredData[2] = *temp_2;

  *temp_3 = temp2;
  gatheredData[3] = *temp_3;

  *temp_4 = temp3;
  gatheredData[4] = *temp_4;

  gatheredData[5] = *RH_1;
  gatheredData[6] = *RH_2;
  gatheredData[7] = *RH_3;
  gatheredData[8] = *RH_4;
 

  //Serial.printf("Temp_0: %0.1f\nRH_0: %0.1f\n\nTemp_1: %0.1f\nRH_1: %0.1f\n\nTemp_2: %0.1f\nRH_2: %0.1f\n\nTemp_3: %0.1f\nRH_3: %0.1f\n\n", *temp_0, *RH_0, *temp_1, *RH_1,*temp_2, *RH_2, *temp_3, *RH_3);
  
}

// Calculates and returns the LUX values for devices 4 - 7 on the I2C multiplexer
void getLux(float *lux1, float *lux2, float *lux3, float *lux4){
  
  pcaselect(1);
  *lux1 = (lux_1.readALS() * 0.110779);  // Light level [lx] is: OUTPUT DATA [dec.] / ALS sensitivity) x (10 / IT [ms]) ---The exact integration time is 90 ms, so the factor should not be 0.1 but 0.110779
  gatheredData[9] = *lux1;

  pcaselect(3);
  *lux2 = (lux_2.readALS() * 0.110779);
  gatheredData[10] = *lux2;

  pcaselect(5);
  *lux3 = (lux_3.readALS() * 0.110779);
  gatheredData[11] = *lux3;

  pcaselect(7);
  *lux4 = (lux_4.readALS() * 0.110779);
  gatheredData[12] = *lux4;
  //Serial.printf("Lux sensor 4: %0.4f lx\n\nLux sensor 5: %0.4f lx\n\nLux sensor 6: %0.4flx\n\nLux sensor 7: %0.4flx\n\n", *lux4, *lux5, *lux6, *lux7);
}

// Creates a json object with 12+ Key:Value pairs
void createEventPayload(){

  JsonWriterStatic<256> jw;
  {
	  JsonWriterAutoObject obj(&jw);

    if(activeArea1 && activeArea2 && activeArea3 && activeArea4){
      for(int i = 0; i < 13; i++){
        jw.insertKeyValue(dataTags[i], gatheredData[i]);
      }
	  }
    else{
     bufferElementBypass(bypassArr);
     for(int i = 0; i < 12; i++){
        if(i+1 == bypassArr[i]){
          continue;
        }
        jw.insertKeyValue(dataTags[i], gatheredData[i]);
      }
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


void bufferElementBypass(int *arr){
  
  if(!activeArea1){
    arr[0] = 1;
    arr[1] = 5;
    arr[2] = 9;
  }
  if(!activeArea2){
    arr[3] = 2;
    arr[4] = 6;
    arr[5] = 10;
  }
  if(!activeArea3){
    arr[6] = 3;
    arr[7] = 7;
    arr[8] = 11;
  }
  if(!activeArea4){
    arr[9] = 4;
    arr[10] = 8;
    arr[11] = 12;
  }

  for(int n = 0; n <= 10; n++){
    for(int j = 0; j < 12; j++){
      if(arr[j + 1] < arr[j]){
        int shuffleSpace = arr[j + 1];
        arr[j + 1] = arr[j];
        arr[j] = shuffleSpace;
      }
    }
  }

  

}