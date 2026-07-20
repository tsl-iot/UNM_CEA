/* 
 * Project Measuring and adjusting Ph of a liquid solution
 * Author: Edward Ishman
 * Date: 07/17/2026
 * 
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "DFRobot_PH.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"

SYSTEM_MODE(SEMI_AUTOMATIC);


const int PH_PIN = A1;
float voltage,phValue;
float temperature;
unsigned int lastPH;

Adafruit_SSD1306 display(-1);
DFRobot_PH ph;

void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected, 5000);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  
  display.display();
  delay(2000);
  display.clearDisplay();   
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.display();

  pinMode(PH_PIN, INPUT);
  ph.begin();
}

void loop() {

  if((millis()-lastPH) > 1000){       
    display.clearDisplay(); 
    display.setCursor(0,0);    

    voltage = analogRead(PH_PIN)/4095.0*3300;  // read the voltage
    phValue = ph.readPH(voltage,temperature);  // convert voltage to pH with temperature compensation

    Serial.printf("pH: %0.1f\n", phValue);
    display.printf("pH: %0.1f\n", phValue);
    display.display();
    
    lastPH = millis();
  }
}
