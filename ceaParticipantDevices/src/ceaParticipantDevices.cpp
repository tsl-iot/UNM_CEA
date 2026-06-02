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

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(SEMI_AUTOMATIC);


Adafruit_VEML7700 luxSense;
Adafruit_HDC302x tempHumSense = Adafruit_HDC302x();

TCPClient TheClient; 
Adafruit_MQTT_SPARK mqtt(&TheClient,NODE_RED_SERVER,NODE_RED_SERVERPORT,NODE_RED_USERNAME,NODE_RED_KEY);
Adafruit_MQTT_Publish dataFeed = Adafruit_MQTT_Publish(&mqtt, "cea/dataobject");




struct sensorData{
  String deviceID;
  float temperature;
  double relativeHumidity;
  float luxIntensity;
};
sensorData measuredReadings;

bool incrementPrint;
bool readyToPublish;
String deviceNumbers[2] = {"PAR_00","PAR_01"};

void initHDC320x();
void initVEML7700();
void createEventPayload(sensorData dataBuffer);
void grabAllSensorData();
void MQTT_connect();

Timer dataGrab(30000, grabAllSensorData);

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

  // Particle.disconnect();
  // while(!Particle.disconnected()){
  //   if(incrementPrint == 0){
  //     Serial.printf("Turning off WiFi radio\n");
  //     incrementPrint++;
  //   }
  //   else{
  //     Serial.printf(" . ");
  //     delay(100);
  //   }
  // }
  // incrementPrint = 0;

  // WiFi.disconnect();
  // WiFi.off();
  // delay(8000);

  // Cellular.on();
  // waitFor(Cellular.isOn, 30000);
  // Cellular.connect();
  // while(!Cellular.ready()){
  //   if(incrementPrint == 0){
  //     Serial.printf("Connecting the Cellular radio \n");
  //     incrementPrint++;
  //   }
  //   else{
  //     Serial.printf(" . ");
  //     delay(100);
  //   }
  // }
  dataGrab.start();

  SystemPowerConfiguration powerConfig = System.getPowerConfiguration();
powerConfig.auxiliaryPowerControlPin(D7).interruptPin(A7);
System.setPowerConfiguration(powerConfig);
}

void loop() {
  MQTT_connect();
  if(readyToPublish){
    createEventPayload(measuredReadings);
  }
}


// Measures Temperature and relative humidity from sensors 0 - 3 on the I2C multiplexer
void initHDC320x(){
  if(!tempHumSense.begin(0x44)){
    Serial.printf("Temp/Hum sensor 1 FAILED TO START!\n");
  }
  else{
    Serial.printf("Temp/Hum sensor 1 successfully started\n");
  }
}

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


void get_HDC_T_H(){
  double temp0, RH_1;
  tempHumSense.readTemperatureHumidityOnDemand(temp0, RH_1, TRIGGERMODE_LP0);

  measuredReadings.temperature = temp0;
  measuredReadings.relativeHumidity = RH_1;
  

  //Serial.printf("Temp_0: %0.1f\nRH_0: %0.1f\n\nTemp_1: %0.1f\nRH_1: %0.1f\n\nTemp_2: %0.1f\nRH_2: %0.1f\n\nTemp_3: %0.1f\nRH_3: %0.1f\n\n", *temp_0, *RH_0, *temp_1, *RH_1,*temp_2, *RH_2, *temp_3, *RH_3);
  
}

// Calculates and returns the LUX values for devices 4 - 7 on the I2C multiplexer
void getLux(){
  
  measuredReadings.luxIntensity = (luxSense.readALS() * 0.110779);  // Light level [lx] is: OUTPUT DATA [dec.] / ALS sensitivity) x (10 / IT [ms]) ---The exact integration time is 90 ms, so the factor should not be 0.1 but 0.110779
  

 
}

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

void grabAllSensorData(){
  static bool SwitchNum;
  SwitchNum = !SwitchNum;
  get_HDC_T_H();
  getLux();
  measuredReadings.deviceID = deviceNumbers[SwitchNum];
  readyToPublish = true;
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