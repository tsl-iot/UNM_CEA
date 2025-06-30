/* 
 * Project: Controlled Environment Agriculture Base Station
 * Author: Edward Ishman
 * Date: 04/21/2025
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT.h" 
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h" 
#include <UNM_CEA_Credentials.h>
#include "../lib/Adafruit_MQTT/src/Adafruit_MQTT_SPARK.h"

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(SEMI_AUTOMATIC);

// MQTT Broker and Feeds
TCPClient TheClient; 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY);

// Device 1 Feeds
Adafruit_MQTT_Publish tempFeed_1 = Adafruit_MQTT_Publish(&mqtt, "cea/temperature0x01");
Adafruit_MQTT_Publish humidityFeed_1 = Adafruit_MQTT_Publish(&mqtt, "cea/humidity0x01");
Adafruit_MQTT_Publish pressureFeed_1 = Adafruit_MQTT_Publish(&mqtt, "cea/pressure0x01");
Adafruit_MQTT_Publish nm415Feed_1 = Adafruit_MQTT_Publish(&mqtt, "cea/nm4150x01");
Adafruit_MQTT_Publish nm445Feed_1 = Adafruit_MQTT_Publish(&mqtt, "cea/nm4450x01");
Adafruit_MQTT_Publish nm480Feed_1 = Adafruit_MQTT_Publish(&mqtt, "cea/nm4800x01");
Adafruit_MQTT_Publish nm515Feed_1 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5150x01");
Adafruit_MQTT_Publish nm555Feed_1 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5550x01");
Adafruit_MQTT_Publish nm615Feed_1 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5900x01");
Adafruit_MQTT_Publish nm630Feed_1 = Adafruit_MQTT_Publish(&mqtt, "cea/nm6300x01");
Adafruit_MQTT_Publish nm680Feed_1 = Adafruit_MQTT_Publish(&mqtt, "cea/nm6800x01");
Adafruit_MQTT_Publish deviceNumberFeed = Adafruit_MQTT_Publish(&mqtt, "cea/devicenumber");
// Device 1 Feeds
Adafruit_MQTT_Publish tempFeed_2 = Adafruit_MQTT_Publish(&mqtt, "cea/temperature0x02");
Adafruit_MQTT_Publish humidityFeed_2 = Adafruit_MQTT_Publish(&mqtt, "cea/humidity0x02");
Adafruit_MQTT_Publish pressureFeed_2 = Adafruit_MQTT_Publish(&mqtt, "cea/pressure0x02");
Adafruit_MQTT_Publish nm415Feed_2 = Adafruit_MQTT_Publish(&mqtt, "cea/nm4150x02");
Adafruit_MQTT_Publish nm445Feed_2 = Adafruit_MQTT_Publish(&mqtt, "cea/nm4450x02");
Adafruit_MQTT_Publish nm480Feed_2 = Adafruit_MQTT_Publish(&mqtt, "cea/nm4800x02");
Adafruit_MQTT_Publish nm515Feed_2 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5150x02");
Adafruit_MQTT_Publish nm555Feed_2 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5550x02");
Adafruit_MQTT_Publish nm615Feed_2 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5900x02");
Adafruit_MQTT_Publish nm630Feed_2 = Adafruit_MQTT_Publish(&mqtt, "cea/nm6300x02");
Adafruit_MQTT_Publish nm680Feed_2 = Adafruit_MQTT_Publish(&mqtt, "cea/nm6800x02");

Adafruit_MQTT_Publish tempFeed_3 = Adafruit_MQTT_Publish(&mqtt, "cea/temperature0x03");
Adafruit_MQTT_Publish humidityFeed_3 = Adafruit_MQTT_Publish(&mqtt, "cea/humidity0x03");
Adafruit_MQTT_Publish pressureFeed_3 = Adafruit_MQTT_Publish(&mqtt, "cea/pressure0x03");
Adafruit_MQTT_Publish nm415Feed_3 = Adafruit_MQTT_Publish(&mqtt, "cea/nm4150x03");
Adafruit_MQTT_Publish nm445Feed_3 = Adafruit_MQTT_Publish(&mqtt, "cea/nm4450x03");
Adafruit_MQTT_Publish nm480Feed_3 = Adafruit_MQTT_Publish(&mqtt, "cea/nm4800x03");
Adafruit_MQTT_Publish nm515Feed_3 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5150x03");
Adafruit_MQTT_Publish nm555Feed_3 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5550x03");
Adafruit_MQTT_Publish nm615Feed_3 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5900x03");
Adafruit_MQTT_Publish nm630Feed_3 = Adafruit_MQTT_Publish(&mqtt, "cea/nm6300x03");
Adafruit_MQTT_Publish nm680Feed_3 = Adafruit_MQTT_Publish(&mqtt, "cea/nm6800x03");

Adafruit_MQTT_Publish tempFeed_4 = Adafruit_MQTT_Publish(&mqtt, "cea/temperature0x04");
Adafruit_MQTT_Publish humidityFeed_4 = Adafruit_MQTT_Publish(&mqtt, "cea/humidity0x04");
Adafruit_MQTT_Publish pressureFeed_4 = Adafruit_MQTT_Publish(&mqtt, "cea/pressure0x04");
Adafruit_MQTT_Publish nm415Feed_4 = Adafruit_MQTT_Publish(&mqtt, "cea/nm4150x04");
Adafruit_MQTT_Publish nm445Feed_4 = Adafruit_MQTT_Publish(&mqtt, "cea/nm4450x04");
Adafruit_MQTT_Publish nm480Feed_4 = Adafruit_MQTT_Publish(&mqtt, "cea/nm4800x04");
Adafruit_MQTT_Publish nm515Feed_4 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5150x04");
Adafruit_MQTT_Publish nm555Feed_4 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5550x04");
Adafruit_MQTT_Publish nm615Feed_4 = Adafruit_MQTT_Publish(&mqtt, "cea/nm6150x04");
Adafruit_MQTT_Publish nm630Feed_4 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5900x04");
Adafruit_MQTT_Publish nm680Feed_4 = Adafruit_MQTT_Publish(&mqtt, "cea/nm6800x04");

Adafruit_MQTT_Publish tempFeed_5 = Adafruit_MQTT_Publish(&mqtt, "cea/temperature0x05");
Adafruit_MQTT_Publish humidityFeed_5 = Adafruit_MQTT_Publish(&mqtt, "cea/humidity0x05");
Adafruit_MQTT_Publish pressureFeed_5 = Adafruit_MQTT_Publish(&mqtt, "cea/pressure0x05");
Adafruit_MQTT_Publish nm415Feed_5 = Adafruit_MQTT_Publish(&mqtt, "cea/nm4150x05");
Adafruit_MQTT_Publish nm445Feed_5 = Adafruit_MQTT_Publish(&mqtt, "cea/nm4450x05");
Adafruit_MQTT_Publish nm480Feed_5= Adafruit_MQTT_Publish(&mqtt, "cea/nm4800x05");
Adafruit_MQTT_Publish nm515Feed_5 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5150x05");
Adafruit_MQTT_Publish nm555Feed_5 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5550x05");
Adafruit_MQTT_Publish nm615Feed_5 = Adafruit_MQTT_Publish(&mqtt, "cea/nm6150x05");
Adafruit_MQTT_Publish nm630Feed_5 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5900x05");
Adafruit_MQTT_Publish nm680Feed_5 = Adafruit_MQTT_Publish(&mqtt, "cea/nm6800x05");

Adafruit_MQTT_Publish tempFeed_6 = Adafruit_MQTT_Publish(&mqtt, "cea/temperature0x06");
Adafruit_MQTT_Publish humidityFeed_6 = Adafruit_MQTT_Publish(&mqtt, "cea/humidity0x06");
Adafruit_MQTT_Publish pressureFeed_6 = Adafruit_MQTT_Publish(&mqtt, "cea/pressure0x06");
Adafruit_MQTT_Publish nm415Feed_6 = Adafruit_MQTT_Publish(&mqtt, "cea/nm4150x06");
Adafruit_MQTT_Publish nm445Feed_6 = Adafruit_MQTT_Publish(&mqtt, "cea/nm4450x06");
Adafruit_MQTT_Publish nm480Feed_6= Adafruit_MQTT_Publish(&mqtt, "cea/nm4800x06");
Adafruit_MQTT_Publish nm515Feed_6 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5150x06");
Adafruit_MQTT_Publish nm555Feed_6 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5550x06");
Adafruit_MQTT_Publish nm615Feed_6 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5900x06");
Adafruit_MQTT_Publish nm630Feed_6 = Adafruit_MQTT_Publish(&mqtt, "cea/nm6300x06");
Adafruit_MQTT_Publish nm680Feed_6 = Adafruit_MQTT_Publish(&mqtt, "cea/nm6800x06");

Adafruit_MQTT_Publish tempFeed_7 = Adafruit_MQTT_Publish(&mqtt, "cea/temperature0x07");
Adafruit_MQTT_Publish humidityFeed_7 = Adafruit_MQTT_Publish(&mqtt, "cea/humidity0x07");
Adafruit_MQTT_Publish pressureFeed_7 = Adafruit_MQTT_Publish(&mqtt, "cea/pressure0x07");
Adafruit_MQTT_Publish nm415Feed_7 = Adafruit_MQTT_Publish(&mqtt, "cea/nm4150x07");
Adafruit_MQTT_Publish nm445Feed_7 = Adafruit_MQTT_Publish(&mqtt, "cea/nm4450x07");
Adafruit_MQTT_Publish nm480Feed_7= Adafruit_MQTT_Publish(&mqtt, "cea/nm4800x07");
Adafruit_MQTT_Publish nm515Feed_7 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5150x07");
Adafruit_MQTT_Publish nm555Feed_7 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5550x07");
Adafruit_MQTT_Publish nm615Feed_7 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5900x07");
Adafruit_MQTT_Publish nm630Feed_7 = Adafruit_MQTT_Publish(&mqtt, "cea/nm6300x07");
Adafruit_MQTT_Publish nm680Feed_7 = Adafruit_MQTT_Publish(&mqtt, "cea/nm6800x07");

Adafruit_MQTT_Publish tempFeed_8 = Adafruit_MQTT_Publish(&mqtt, "cea/temperature0x08");
Adafruit_MQTT_Publish humidityFeed_8 = Adafruit_MQTT_Publish(&mqtt, "cea/humidity0x08");
Adafruit_MQTT_Publish pressureFeed_8 = Adafruit_MQTT_Publish(&mqtt, "cea/pressure0x08");
Adafruit_MQTT_Publish nm415Feed_8 = Adafruit_MQTT_Publish(&mqtt, "cea/nm4150x08");
Adafruit_MQTT_Publish nm445Feed_8 = Adafruit_MQTT_Publish(&mqtt, "cea/nm4450x08");
Adafruit_MQTT_Publish nm480Feed_8= Adafruit_MQTT_Publish(&mqtt, "cea/nm4800x08");
Adafruit_MQTT_Publish nm515Feed_8 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5150x08");
Adafruit_MQTT_Publish nm555Feed_8 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5550x08");
Adafruit_MQTT_Publish nm615Feed_8 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5900x08");
Adafruit_MQTT_Publish nm630Feed_8 = Adafruit_MQTT_Publish(&mqtt, "cea/nm6300x08");
Adafruit_MQTT_Publish nm680Feed_8 = Adafruit_MQTT_Publish(&mqtt, "cea/nm6800x08");

Adafruit_MQTT_Publish tempFeed_9 = Adafruit_MQTT_Publish(&mqtt, "cea/temperature0x09");
Adafruit_MQTT_Publish humidityFeed_9 = Adafruit_MQTT_Publish(&mqtt, "cea/humidity0x09");
Adafruit_MQTT_Publish pressureFeed_9 = Adafruit_MQTT_Publish(&mqtt, "cea/pressure0x09");
Adafruit_MQTT_Publish nm415Feed_9 = Adafruit_MQTT_Publish(&mqtt, "cea/nm4150x09");
Adafruit_MQTT_Publish nm445Feed_9 = Adafruit_MQTT_Publish(&mqtt, "cea/nm4450x09");
Adafruit_MQTT_Publish nm480Feed_9= Adafruit_MQTT_Publish(&mqtt, "cea/nm4800x09");
Adafruit_MQTT_Publish nm515Feed_9 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5150x09");
Adafruit_MQTT_Publish nm555Feed_9 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5550x09");
Adafruit_MQTT_Publish nm615Feed_9 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5900x09");
Adafruit_MQTT_Publish nm630Feed_9 = Adafruit_MQTT_Publish(&mqtt, "cea/nm6300x09");
Adafruit_MQTT_Publish nm680Feed_9 = Adafruit_MQTT_Publish(&mqtt, "cea/nm6800x09");

Adafruit_MQTT_Publish tempFeed_10 = Adafruit_MQTT_Publish(&mqtt, "cea/temperature0x10");
Adafruit_MQTT_Publish humidityFeed_10 = Adafruit_MQTT_Publish(&mqtt, "cea/humidity0x10");
Adafruit_MQTT_Publish pressureFeed_10 = Adafruit_MQTT_Publish(&mqtt, "cea/pressure0x10");
Adafruit_MQTT_Publish nm415Feed_10 = Adafruit_MQTT_Publish(&mqtt, "cea/nm4150x10");
Adafruit_MQTT_Publish nm445Feed_10 = Adafruit_MQTT_Publish(&mqtt, "cea/nm4450x10");
Adafruit_MQTT_Publish nm480Feed_10= Adafruit_MQTT_Publish(&mqtt, "cea/nm4800x10");
Adafruit_MQTT_Publish nm515Feed_10 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5150x10");
Adafruit_MQTT_Publish nm555Feed_10 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5550x10");
Adafruit_MQTT_Publish nm615Feed_10 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5900x10");
Adafruit_MQTT_Publish nm630Feed_10 = Adafruit_MQTT_Publish(&mqtt, "cea/nm6300x10");
Adafruit_MQTT_Publish nm680Feed_10 = Adafruit_MQTT_Publish(&mqtt, "cea/nm6800x10");

Adafruit_MQTT_Publish tempFeed_11 = Adafruit_MQTT_Publish(&mqtt, "cea/temperature0x11");
Adafruit_MQTT_Publish humidityFeed_11 = Adafruit_MQTT_Publish(&mqtt, "cea/humidity0x11");
Adafruit_MQTT_Publish pressureFeed_11 = Adafruit_MQTT_Publish(&mqtt, "cea/pressure0x11");
Adafruit_MQTT_Publish nm415Feed_11 = Adafruit_MQTT_Publish(&mqtt, "cea/nm4150x11");
Adafruit_MQTT_Publish nm445Feed_11 = Adafruit_MQTT_Publish(&mqtt, "cea/nm4450x11");
Adafruit_MQTT_Publish nm480Feed_11= Adafruit_MQTT_Publish(&mqtt, "cea/nm4800x11");
Adafruit_MQTT_Publish nm515Feed_11 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5150x11");
Adafruit_MQTT_Publish nm555Feed_11 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5550x11");
Adafruit_MQTT_Publish nm615Feed_11 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5900x11");
Adafruit_MQTT_Publish nm630Feed_11 = Adafruit_MQTT_Publish(&mqtt, "cea/nm6300x11");
Adafruit_MQTT_Publish nm680Feed_11 = Adafruit_MQTT_Publish(&mqtt, "cea/nm6800x11");

Adafruit_MQTT_Publish tempFeed_12 = Adafruit_MQTT_Publish(&mqtt, "cea/temperature0x12");
Adafruit_MQTT_Publish humidityFeed_12 = Adafruit_MQTT_Publish(&mqtt, "cea/humidity0x12");
Adafruit_MQTT_Publish pressureFeed_12 = Adafruit_MQTT_Publish(&mqtt, "cea/pressure0x12");
Adafruit_MQTT_Publish nm415Feed_12 = Adafruit_MQTT_Publish(&mqtt, "cea/nm4150x12");
Adafruit_MQTT_Publish nm445Feed_12 = Adafruit_MQTT_Publish(&mqtt, "cea/nm4450x12");
Adafruit_MQTT_Publish nm480Feed_12= Adafruit_MQTT_Publish(&mqtt, "cea/nm4800x12");
Adafruit_MQTT_Publish nm515Feed_12 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5150x12");
Adafruit_MQTT_Publish nm555Feed_12 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5550x12");
Adafruit_MQTT_Publish nm615Feed_12 = Adafruit_MQTT_Publish(&mqtt, "cea/nm5900x12");
Adafruit_MQTT_Publish nm630Feed_12 = Adafruit_MQTT_Publish(&mqtt, "cea/nm6300x12");
Adafruit_MQTT_Publish nm680Feed_12 = Adafruit_MQTT_Publish(&mqtt, "cea/nm6800x12");






// BLE Service UUID's -- Imitates UART communication (RX - TX)
void onDataReceived(const uint8_t *data, size_t len, const BlePeerDevice &peer, void *context);
const BleUuid serviceUuid("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
const BleUuid rxUuid("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
const BleUuid txUuid("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");

// BLE Objects
BlePeerDevice peer;
BleCharacteristic peerTxCharacteristic;
BleCharacteristic rxCharacteristic ("rx",BleCharacteristicProperty::WRITE_WO_RSP,rxUuid,serviceUuid,onDataReceived,NULL);

// Watchdog
ApplicationWatchdog *wd;
unsigned int lastPrint;
int startingDevice;



// Structs
struct Devices{
  float tempF;
  int humidity;
  float pressure;
  float lightColorReadings[8];
};
Devices device_1;
Devices device_2;
Devices device_3;
Devices device_4;
Devices device_5;
Devices device_6;
Devices device_7;
Devices device_8;
Devices device_9;
Devices device_10;
Devices device_11;
Devices device_12;

Adafruit_MQTT_Publish feedArray[12][12] = {
{tempFeed_1, humidityFeed_1, pressureFeed_1, nm415Feed_1, nm445Feed_1, nm480Feed_1, nm515Feed_1, nm555Feed_1, nm615Feed_1, nm630Feed_1, nm680Feed_1, deviceNumberFeed},
{tempFeed_2, humidityFeed_2, pressureFeed_2, nm415Feed_2, nm445Feed_2, nm480Feed_2, nm515Feed_2, nm555Feed_2, nm615Feed_2, nm630Feed_2, nm680Feed_2, deviceNumberFeed},
{tempFeed_3, humidityFeed_3, pressureFeed_3, nm415Feed_3, nm445Feed_3, nm480Feed_3, nm515Feed_3, nm555Feed_3, nm615Feed_3, nm630Feed_3, nm680Feed_3, deviceNumberFeed},
{tempFeed_4, humidityFeed_4, pressureFeed_4, nm415Feed_4, nm445Feed_4, nm480Feed_4, nm515Feed_4, nm555Feed_4, nm615Feed_4, nm630Feed_4, nm680Feed_4, deviceNumberFeed},
{tempFeed_5, humidityFeed_5, pressureFeed_5, nm415Feed_5, nm445Feed_5, nm480Feed_5, nm515Feed_5, nm555Feed_5, nm615Feed_5, nm630Feed_5, nm680Feed_5, deviceNumberFeed},
{tempFeed_6, humidityFeed_6, pressureFeed_6, nm415Feed_6, nm445Feed_6, nm480Feed_6, nm515Feed_6, nm555Feed_6, nm615Feed_6, nm630Feed_6, nm680Feed_6, deviceNumberFeed},
{tempFeed_7, humidityFeed_7, pressureFeed_7, nm415Feed_7, nm445Feed_7, nm480Feed_7, nm515Feed_7, nm555Feed_7, nm615Feed_7, nm630Feed_7, nm680Feed_7, deviceNumberFeed},
{tempFeed_8, humidityFeed_8, pressureFeed_8, nm415Feed_8, nm445Feed_8, nm480Feed_8, nm515Feed_8, nm555Feed_8, nm615Feed_8, nm630Feed_8, nm680Feed_8, deviceNumberFeed},
{tempFeed_9, humidityFeed_9, pressureFeed_9, nm415Feed_9, nm445Feed_9, nm480Feed_9, nm515Feed_9, nm555Feed_9, nm615Feed_9, nm630Feed_9, nm680Feed_9, deviceNumberFeed},
{tempFeed_10, humidityFeed_10, pressureFeed_10, nm415Feed_10, nm445Feed_10, nm480Feed_10, nm515Feed_10, nm555Feed_10, nm615Feed_10, nm630Feed_10, nm680Feed_10, deviceNumberFeed},
{tempFeed_11, humidityFeed_11, pressureFeed_11, nm415Feed_11, nm445Feed_11, nm480Feed_11, nm515Feed_11, nm555Feed_11, nm615Feed_11, nm630Feed_11, nm680Feed_11, deviceNumberFeed},
{tempFeed_12, humidityFeed_12, pressureFeed_12, nm415Feed_12, nm445Feed_12, nm480Feed_12, nm515Feed_12, nm555Feed_12, nm615Feed_12, nm630Feed_12, nm680Feed_12, deviceNumberFeed}



};

//Functions
void bleConnect();
void MQTT_connect();
void pingBroker();
uint64_t millis64bit();
void watchdogHandler();
void parseIncomingData(const uint8_t *data, Devices deviceNum, int deviceID, Adafruit_MQTT_Publish feedName[12][12]);
// void connectToNetwork();
// void getWiFi();

void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected,2500);
  Cellular.on();
  Cellular.connect();
  delay(10000);
  Particle.connect();
  //getWiFi();
  //connectToNetwork();
  BLE.on();
  peerTxCharacteristic.onDataReceived(onDataReceived, &peerTxCharacteristic);
  BLE.setTxPower(8);
  Serial.printf("M-SOM BLE Address: %s\n",BLE.address().toString().c_str());
  //wd = new ApplicationWatchdog(180000, watchdogHandler, 1536);
  pinMode(D7,OUTPUT);
  digitalWrite(D7, LOW);
  startingDevice = 1;
  delay(2000);
}

void loop() {
  //MQTT_connect();
  //pingBroker();
  digitalWrite(D7, LOW);
  if(!BLE.connected()){
    bleConnect();
    if((millis() - lastPrint) > 5000){
      Serial.printf("Looking for devices...\n");
      lastPrint = millis();
    }
  }
}
  
    
  
  
 


// Scan BLE devices then connect to the appropriate one
void bleConnect(){
  static int deviceCount;
  static int scanCount;
  String _deviceName;
  BleAdvertisingData advData;
  BleScanFilter filter;
  filter.serviceUUID(txUuid);

  // start scanning for devices advertising TxUUID
  Vector<BleScanResult> scanResults =  BLE.scanWithFilter(filter); 
  
  if( scanResults.size() > 0 ){
    Serial.printf("%i devices found\n", scanResults.size());
    for(int ii = 0; ii < scanResults.size(); ii++){
      if(String(deviceCount) == scanResults[ii].advertisingData().deviceName()){
        peer = BLE.connect(scanResults[ii].address());
        scanCount = scanResults.size() - 1;
        if(peer.connected()){
          digitalWrite(D7, HIGH); 
          peer.getCharacteristicByUUID(peerTxCharacteristic, txUuid); // Get peer device and TX characteristics.
        }
        BLE.disconnect();
        deviceCount++;
        if((deviceCount > 12 ) || (deviceCount == 0)){
          deviceCount = 1;
          BLE.disconnect();

        }
        return;
        
      }
        
        
    }
    deviceCount++;
    if((deviceCount > 12 ) || (deviceCount == 0)){
      deviceCount = 1;
      BLE.disconnect();

    }
  }
    
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

void onDataReceived(const uint8_t *data, size_t len, const BlePeerDevice &peer, void *context){
  

  //Serial.printf("Received data from :%02X :%02X :%02X :%02X :%02X :%02X\n", peer.address()[0], peer.address()[1], peer.address()[2], peer.address()[3], peer.address()[4], peer.address()[5]);
  if(peer.address()[0] == 0xAD){
    parseIncomingData(data, device_1, 1,feedArray);
    
  }
  if(peer.address()[0] == 0xA5){
    parseIncomingData(data, device_2, 2, feedArray);
  
  }
  if(peer.address()[0] == 0x4D){
    parseIncomingData(data, device_3, 3, feedArray);
  }
  if(peer.address()[0] == 0xF1){
    parseIncomingData(data, device_4, 4, feedArray);
  }
  if(peer.address()[0] == 0xC1){
    parseIncomingData(data, device_5, 5, feedArray);
  }
  if(peer.address()[0] == 0x59){
    parseIncomingData(data, device_6, 6, feedArray);
  }
  if(peer.address()[0] == 0xB9){
    parseIncomingData(data, device_7, 7, feedArray);
  }
  if(peer.address()[0] == 0x55){
    parseIncomingData(data, device_8, 8, feedArray);
  }
  if(peer.address()[0] == 0xF5){
    parseIncomingData(data, device_9, 9, feedArray);
  }
  if(peer.address()[0] == 0xD9){
    parseIncomingData(data, device_10, 10, feedArray);
  }
  if((peer.address()[0] == 0x4D) && (peer.address()[1] == 0x54)){
    parseIncomingData(data, device_11, 11, feedArray);
  }
  if((peer.address()[0] == 0x55) && (peer.address()[1] == 0x54)){
    parseIncomingData(data, device_12, 12, feedArray);
  }
}


void parseIncomingData(const uint8_t *data, Devices deviceNum, int deviceID, Adafruit_MQTT_Publish feedName[12][12]){

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

  // Parse 415 wavelength
  newDelimiter = incomingMessage.indexOf(':');
  //Serial.printf("%i\n", newDelimiter);
  String nm415 = incomingMessage.substring(0, newDelimiter);
  incomingMessage.remove(0,newDelimiter+1);
  deviceNum.lightColorReadings[0] = atof(nm415.c_str());

  // Parse 445 wavelength
  newDelimiter = incomingMessage.indexOf(':');
  //Serial.printf("%i\n", newDelimiter);
  String nm445 = incomingMessage.substring(0, newDelimiter);
  incomingMessage.remove(0,newDelimiter+1);
  deviceNum.lightColorReadings[1] = atof(nm445.c_str());

  // Parse 480 wavelength
  newDelimiter = incomingMessage.indexOf(':');
  //Serial.printf("%i\n", newDelimiter);
  String nm480 = incomingMessage.substring(0, newDelimiter);
  incomingMessage.remove(0,newDelimiter+1);
  deviceNum.lightColorReadings[2] = atof(nm480.c_str());

  // Parse 515 wavelength
  newDelimiter = incomingMessage.indexOf(':');
  //Serial.printf("%i\n", newDelimiter);
  String nm515 = incomingMessage.substring(0, newDelimiter);
  incomingMessage.remove(0,newDelimiter+1);
  deviceNum.lightColorReadings[3] = atof(nm515.c_str());

  // Parse 555 wavelength
  newDelimiter = incomingMessage.indexOf(':');
  //Serial.printf("%i\n", newDelimiter);
  String nm555 = incomingMessage.substring(0, newDelimiter);
  incomingMessage.remove(0,newDelimiter+1);
  deviceNum.lightColorReadings[4] = atof(nm555.c_str());

  // Parse 590 wavelength
  newDelimiter = incomingMessage.indexOf(':');
  //Serial.printf("%i\n", newDelimiter);
  String nm590 = incomingMessage.substring(0, newDelimiter);
  incomingMessage.remove(0,newDelimiter+1);
  deviceNum.lightColorReadings[5] = atof(nm590.c_str());

  // Parse 630 wavelength
  newDelimiter = incomingMessage.indexOf(':');
  //Serial.printf("%i\n", newDelimiter);
  String nm630 = incomingMessage.substring(0, newDelimiter);
  incomingMessage.remove(0,newDelimiter+1);
  deviceNum.lightColorReadings[6] = atof(nm630.c_str());

  // Parse 680 wavelength
  newDelimiter = incomingMessage.indexOf(':');
  //Serial.printf("%i\n", newDelimiter);
  String nm680 = incomingMessage.substring(0, newDelimiter);
  //incomingMessage.remove(0,newDelimiter);
  deviceNum.lightColorReadings[7] = atof(nm680.c_str());

  Serial.printf("Readings from device ID: %i\nTemperature: %0.1f\nHumidity %i\nBarometric pressure: %0.1f\n415 Wavelength: %0.4f\n445 Wavelength: %0.4f\n480 Wavelength: %0.4f\n515 Wavelength: %0.4f\n555 Wavelength: %0.4f\n590 Wavelength: %0.4f\n630 Wavelength: %0.4f\n680 Wavelength: %0.4f\n", deviceID, deviceNum.tempF ,deviceNum.humidity, deviceNum.pressure, deviceNum.lightColorReadings[0], deviceNum.lightColorReadings[1], deviceNum.lightColorReadings[2], deviceNum.lightColorReadings[3], deviceNum.lightColorReadings[4], deviceNum.lightColorReadings[5], deviceNum.lightColorReadings[6], deviceNum.lightColorReadings[7]);
  if(mqtt.Update()){
    feedName[deviceID-1][0].publish(deviceNum.tempF);
    feedName[deviceID-1][1].publish(deviceNum.humidity);
    feedName[deviceID-1][2].publish(deviceNum.pressure);
    feedName[deviceID-1][3].publish(deviceNum.lightColorReadings[0]);
    feedName[deviceID-1][4].publish(deviceNum.lightColorReadings[1]);
    feedName[deviceID-1][5].publish(deviceNum.lightColorReadings[2]);
    feedName[deviceID-1][6].publish(deviceNum.lightColorReadings[3]);
    feedName[deviceID-1][7].publish(deviceNum.lightColorReadings[4]);
    feedName[deviceID-1][8].publish(deviceNum.lightColorReadings[5]);
    feedName[deviceID-1][9].publish(deviceNum.lightColorReadings[6]);
    feedName[deviceID-1][10].publish(deviceNum.lightColorReadings[7]);
    feedName[deviceID-1][11].publish(deviceID);
  }
}

// void connectToNetwork(){
//   unsigned int lastConnect = 0;

//   WiFi.on();
//   WiFi.connect();
//   while(WiFi.connecting() && ((millis() - lastConnect) < 60000)) {
//     Serial.printf(".");
//     delay(100);
//   }
//   if(WiFi.ready()){
//     Serial.printf("Connected to WiFi!\n");
//   }
//   else{
//     Serial.printf("Couldn't connected to WiFi\n");
//     Cellular.on();
//     Cellular.connect();
//     lastConnect = millis();
//   }
//   while(Cellular.connecting() && ((millis() - lastConnect) < 60000)){
//     Serial.printf("x");
//     delay(100);
//   }
//   if(Cellular.ready()){
//     Serial.printf("Connected to Cellular!\n");
//   }
//   else{
//     Serial.printf("Couldn't connected to Cellular\n");
//   }
  

//   // while(WiFi.connecting() && ((millis() - lastConnect) < 60000)){
//   //   Serial.printf(".");
//   // }
//   // if(!WiFi.ready()){
//   //   WiFi.disconnect();
//   //   WiFi.off();
//   //   Cellular.on();
//   //   Cellular.connect();
//   //   Serial.printf("Couldn't connect to WiFi\n");
//   // }
//   // else{
//   //   Serial.printf("Connected to WiFi!\n");
//   //   Particle.connect();
//   // }
//   // while(!WiFi.ready() && ((millis() - lastConnect) < 60000) && Cellular.connecting()){
//   //   Serial.printf("x");
//   // }
//   // if(!Cellular.ready()){
//   //   Cellular.disconnect();
//   //   Cellular.off();
//   //   Serial.printf("Couldn't connect to Cellular\n");
//   // }
//   // else{
//   //   Serial.printf("Connected to Cellular!\n");
//   // }
//   // if(!WiFi.ready() && !Cellular.ready()){
//   //   Serial.printf("Couldn't connect to WiFi or Cellular network\n");
//   //   //connectToNetwork();
//   // }
// }


// }