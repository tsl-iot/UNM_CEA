/* 
 * Project myProject
 * Author: Your Name
 * Date: 
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
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

SYSTEM_MODE(SEMI_AUTOMATIC);



//Beginning of MQTT objects and feed for publishing
TCPClient TheClient; 
Adafruit_MQTT_SPARK mqtt(&TheClient,NODE_RED_SERVER,NODE_RED_SERVERPORT,NODE_RED_USERNAME,NODE_RED_KEY);
Adafruit_MQTT_Publish dataFeed = Adafruit_MQTT_Publish(&mqtt, "cea/dataobject");
//End

//Beginning of data Structure
struct sensorData{
  String deviceID;
  float temperature;
  double relativeHumidity;
  float luxIntensity;
};
sensorData measuredReadings;
//End

//Beginning of sensor objects
Adafruit_VEML7700 luxSense;
Adafruit_HDC302x tempHumSense = Adafruit_HDC302x();
//End

//Beginning of variables, constants, etc.
bool readyToPublish;
const String deviceNumber = "PAR_XX";                 //REPLACE THE XX IN "PAR_XX" WITH A 2-DIGIT NUMBER RANGING FROM 00 - 14 BEFORE FLASHING THE PARTICLE DEVICE
//End

//Beginning of Functions
void initHDC320x();
void initVEML7700();
void createEventPayload(sensorData dataBuffer);
void grabAllSensorData();
void MQTT_connect();
void quitWifiGetCellular();
Timer dataGrab(30000, grabAllSensorData);
//End

void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected, 5000);
  Wire.begin();
  delay(1000);
  WiFi.on();
  WiFi.connect();
  while(!WiFi.ready()){
    Serial.printf(" . ");
    delay(100);
  }
  
  initHDC320x();
  initVEML7700();
  dataGrab.start();         //Start the data collection timer
}

void loop() {
  MQTT_connect();         
  if(readyToPublish){
    createEventPayload(measuredReadings);         
  }
}

//Initializes and configures the HDC320X
void initHDC320x(){
  if(!tempHumSense.begin(0x44)){
    Serial.printf("Temp/Hum sensor 1 FAILED TO START!\n");
  }
  else{
    Serial.printf("Temp/Hum sensor 1 successfully started\n");
  }
}

//Initializes and configures the VEML7700 
void initVEML7700(){
  if(!luxSense.begin()){
    Serial.printf("Lux sensor 4 FAILED TO START!\n");
  }
  else{
    Serial.printf("Lux sensor 4 successfully started\n");
    luxSense.setGain(VEML7700_GAIN_1_8); //Set gain
    luxSense.setIntegrationTime(VEML7700_IT_100MS); // set amount of time to read the light intensity. Sort of like shutter speed on a camera
  }
}

// Measures the temperature and relative humidity
void get_HDC_T_H(){
  double temp0, RH_1;

  tempHumSense.readTemperatureHumidityOnDemand(temp0, RH_1, TRIGGERMODE_LP0);
  measuredReadings.temperature = temp0;
  measuredReadings.relativeHumidity = RH_1;
  //Serial.printf("Temp_0: %0.1f\nRH_0: %0.1f\n\nTemp_1: %0.1f\nRH_1: %0.1f\n\nTemp_2: %0.1f\nRH_2: %0.1f\n\nTemp_3: %0.1f\nRH_3: %0.1f\n\n", *temp_0, *RH_0, *temp_1, *RH_1,*temp_2, *RH_2, *temp_3, *RH_3);
}

// Measures light intensity then converts the raw data into LUX
void getLux(){
  measuredReadings.luxIntensity = (luxSense.readALS() * 0.110779);  // Light level [lx] is: OUTPUT DATA [dec.] / ALS sensitivity) x (10 / IT [ms]) ---The exact integration time is 90 ms, so the factor should not be 0.1 but 0.110779
}

// Creates a JSON object containing sensor data then publishes using the MQTT feed in the header
void createEventPayload(sensorData dataBuffer){

  JsonWriterStatic<256> jw;
  {
	  JsonWriterAutoObject obj(&jw);

    jw.insertKeyValue("Device_ID", dataBuffer.deviceID);
    jw.insertKeyValue("Temperature_C", dataBuffer.temperature);
    jw.insertKeyValue("Relative_Humidity", dataBuffer.relativeHumidity);
    jw.insertKeyValue("Lux", dataBuffer.luxIntensity);
  }
  if(mqtt.Update()){
    dataFeed.publish(jw.getBuffer());
    Serial.printf("Published: %s\n\n\n", jw.getBuffer());
  }
  readyToPublish = false;
}

// Calls functions for getting sensor datapoints and device ID
void grabAllSensorData(){
  static bool SwitchNum;
  SwitchNum = !SwitchNum;
  get_HDC_T_H();
  getLux();
  measuredReadings.deviceID = deviceNumber;
  readyToPublish = true;
}

// Connects to MQTT
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

// Turns Wifi off and Cyellular on
void quitWifiGetCellular(){
  static bool incrementPrint;
                      
  Particle.disconnect();          // Disconnect from the particle cloud 
  while(!Particle.disconnected()){
    if(incrementPrint == 0){
      Serial.printf("Turning off WiFi radio\n");
      incrementPrint = !incrementPrint;
    }
    else{
      Serial.printf(" . ");
      delay(100);
    }
  }
  incrementPrint = 0;
  WiFi.disconnect();          //Disconnect from Wifi
  WiFi.off();         //Turn the Wifi radio OFF
  delay(5000);

  Cellular.on();          
  waitFor(Cellular.isOn, 30000);          //30 seconds for the Cellular radio to fully turn on
  Cellular.connect();
  while(!Cellular.ready()){
    if(incrementPrint == 0){
      Serial.printf("Connecting the Cellular radio \n");
      incrementPrint = !incrementPrint;
    }
    else{
      Serial.printf(" . ");
      delay(100);
    }
  }
}