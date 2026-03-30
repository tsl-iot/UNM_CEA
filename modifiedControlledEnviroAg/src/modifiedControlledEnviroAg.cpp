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

/*To do:
*figure out if we need to keep both "pressure" and "baroPressure" variables or if this is a remnant from multiple files (comparing to the humidity variable that's the same throughout)
*figure out if we need to keep both "tempF" and "temperature" variables or if this is from multiple files
*Node red (NR): Will use node red to id the device (use a device ID variable), parse out the data, and then send the data to the right feed
***To do from CEA Goals doc: 
*"Can connect and send and save data independently" -- do we need to add code for saving data independently?
*Workshop devices only: "includes 1 BME280 and 1 light sensor, but code can manage array of x10 each sensor"-- does this mean we keep the array code from the base station code?
*Workshop device only: "code to connect either DO or pH sensor (and 1-2 DO or pH sensors that can be used in class)"
*Collaborator device only: "Includes networks of 10x light and BME280 sensors"-- does this use array code too? should the collaborator version of the code be a separate file from the workshop version?
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

// MQTT Broker and Feeds
TCPClient TheClient; 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY);

// Device Feeds
Adafruit_MQTT_Publish tempFeed = Adafruit_MQTT_Publish(&mqtt, "cea/temperature");
Adafruit_MQTT_Publish humidityFeed = Adafruit_MQTT_Publish(&mqtt, "cea/humidity");
Adafruit_MQTT_Publish pressureFeed = Adafruit_MQTT_Publish(&mqtt, "cea/pressure");
Adafruit_MQTT_Publish luxFeed = Adafruit_MQTT_Publish(&mqtt, "cea/lux");
Adafruit_MQTT_Publish device_01_Feed = Adafruit_MQTT_Publish(&mqtt, "cea/devicenumber01");

struct enviroSensors{
  float tempF;
  int humidity;
  float baroPressure;
  float luxVal;
};

// Watchdog
ApplicationWatchdog *wd;
unsigned int lastPrint;
int startingDevice;

//Devices device_1;

enviroSensors enviroData;
enviroSensors lightData;

Adafruit_BME280 bme;
Adafruit_VEML7700 luxSensor;

void getBME(enviroSensors bmeData);
void getLux(enviroSensors luxData);

unsigned int lastRead;

void MQTT_connect();
void pingBroker();
uint64_t millis64bit();
void watchdogHandler();
void parseIncomingData(const uint8_t *data, Devices deviceNum, int deviceID, Adafruit_MQTT_Publish feedName[14][12]);


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

//Lux Setup (note: the Lux has a default address so don't need to add an address here)
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
  MQTT_connect();
  if((millis() - lastRead) > 6000){
    getBME(enviroData);
    getLux(lightData);
    lastRead = millis();
  //publish data
  }
  
}

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

/*****************************************************************/
void watchdogHandler(){
  System.reset(RESET_NO_WAIT);
}

/*****************************************************************/
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

/*****************************************************************/
void pingBroker(){
  static uint64_t last = millis64bit();
  if ((millis()-last)>120000) {
    Serial.printf("Pinging MQTT \n");
    if(! mqtt.ping()) {
      Serial.printf("Disconnecting \n");
      mqtt.disconnect();
    }
      last = millis();
  }
}

/*****************************************************************/
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

/*****************************************************************/
void parseIncomingData(const uint8_t *data, Devices deviceNum, int deviceID, Adafruit_MQTT_Publish feedName[14][12]){

  String incomingMessage;
  int newDelimiter;
  incomingMessage = (String)(char *)data;
  // Serial.printf("%s\n", incomingMessage.c_str());

  // Parse Temperature
  newDelimiter = incomingMessage.indexOf(':');
  //Serial.printf("%i\n", newDelimiter);
  String temperature = incomingMessage.substring(0,newDelimiter);
  incomingMessage.remove(0,newDelimiter+1);
  deviceNum.tempF = atof(temperature.c_str());

  // Parse humidity
  newDelimiter = incomingMessage.indexOf(':');
  //Serial.printf("%i\n", newDelimiter);
  String humidity = incomingMessage.substring(0,newDelimiter);
  incomingMessage.remove(0,newDelimiter+1);
  deviceNum.humidity = atoi(humidity.c_str());

  // Parse barometric pressure
  newDelimiter = incomingMessage.indexOf(':');
  //Serial.printf("%i\n", newDelimiter);
  String pressure = incomingMessage.substring(0,newDelimiter);
  incomingMessage.remove(0,newDelimiter+1);
  deviceNum.pressure = atof(pressure.c_str());

  // Parse Lux reading
  newDelimiter = incomingMessage.indexOf(':');
  //Serial.printf("%i\n", newDelimiter);
  String luxVal = incomingMessage.substring(0, newDelimiter);
  incomingMessage.remove(0,newDelimiter+1);
  deviceNum.luxReading = atof(luxVal.c_str()); 

  Serial.printf("Readings from device ID: %i\nTemperature: %0.1f\nHumidity %i\nBarometric pressure: %0.1f\nLux: %0.4f\n", deviceID, deviceNum.tempF ,deviceNum.humidity, deviceNum.pressure, deviceNum.luxReading;
  if(mqtt.Update()){
    feedName[deviceID-1][0].publish(deviceNum.tempF);
    feedName[deviceID-1][1].publish(deviceNum.humidity);
    feedName[deviceID-1][2].publish(deviceNum.pressure);
    feedName[deviceID-1][3].publish(deviceNum.luxReading);
    feedName[deviceID-1][11].publish(deviceID);
  }
}
