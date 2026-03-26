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
 IoTClassroom_CNM (a/o 3/5/2026)
 Adafruit_VEML7700 (a/o 3/32/2026)
 */

#include "Particle.h"
//BASE STATION//
#include <Adafruit_MQTT.h>
// #include "Adafruit_MQTT/Adafruit_MQTT.h" 
// #include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h" 
//#include <UNM_CEA_Credentials.h>
//PERIPHERAL//
#include "Adafruit_BME280.h"
#include "Adafruit_VEML7700.h"



// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(SEMI_AUTOMATIC);

struct enviroSensors{
  float tempF;
  int humidity;
  float baroPressure;
  float luxVal;
};

enviroSensors enviroData;
enviroSensors lightData;

Adafruit_BME280 bme;
Adafruit_VEML7700 luxSensor;

void getBME(enviroSensors bmeData);
void getLux(enviroSensors luxData);

unsigned int lastRead;

/******************************************************************/
void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected,5000);

//BME Setup
  if(!bme.begin(0x77)){
    Serial.printf("Could not start BME280\n");
  }
  else{
    Serial.printf("Successfully started BME280\n");
  }

//Lux Sensor: the Lux has a default address so don't need to add an address here
  if(!luxSensor.begin()){ 
    Serial.printf("VEML7700 not recognized\n");
  }
  else{
    Serial.printf("VEML up and running!\n");
  }
  luxSensor.setGain(VEML7700_GAIN_1_8); //want a higher gain in a darker environment, can find out other options by right clicking set gain, go to ...
  luxSensor.setIntegrationTime(VEML7700_IT_100MS); //integration time is how long the sensor gathers light info
   
  delay(5000);
}


/******************************************************************/
void loop () {
 if((millis() - lastRead) > 6000){
  getBME(enviroData);
  getLux(lightData);
  lastRead = millis();
  //publish data
 }
  
}


/*****************************************************************
void sendThenDisconnect() {
  
}
*/

/*****************************************************************/
void getBME(enviroSensors bmeData) {
  bmeData.tempF = (bme.readTemperature() * (9.0/5.0)) + 32.0;
  bmeData.humidity = (int)bme.readHumidity();
  bmeData.baroPressure = (bme.readPressure() / 3386.39) + 5;
  Serial.printf("Temp: %0.1f\nHumidity: %i\nBarometric Pressure: %0.2f\n", bmeData.tempF, bmeData.humidity, bmeData.baroPressure);
//get data every minute, publish the data using sprintf to fill a buffer, publish to Node Red (may use JSON?)
}


/*****************************************************************/
void getLux(enviroSensors luxData) {
  luxData.luxVal = luxSensor.readLux();
  Serial.printf("Lux: %0.2f\n", luxData.luxVal);
  delay(5000);
  //when dark, should be close to 0. when super bright (like with a flashlight) should be ~65K
}