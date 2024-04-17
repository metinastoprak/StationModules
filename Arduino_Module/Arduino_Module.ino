/*
 * ArduinoModule for RoboRally
 *
 *  Created on: March 27, 2024
 *      Author: SunnyTeknoloji
 */



#include <Arduino.h>
#include "transceiver_api.h"

#define VERSION_MODULE "1.0"


unsigned long prevtime_T1;        // for IR msg handling
unsigned long prevtime_T2;        // for colorstate handling
unsigned long prevtime_T3;        // for IR message handling
/**
* @brief Function Name Arduino Setup Functions
*/
void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    while (!Serial) {
      ;  // wait for serial port to connect. Needed for native USB port only
    }  
     // wait for serial port to connect. Needed for native USB port only

    // Just to know which program is running on my Arduino
    Serial.println(F("\r\n RoboRally Module ver: " VERSION_MODULE));
    pinMode(LED_PIN, OUTPUT);
    pinMode(START_PIN, OUTPUT);
    Transceiver_Init();

    prevtime_T1 = millis();
    prevtime_T2 = millis();
    prevtime_T3 = millis();
    
}

/**
* @brief Function Name Arduino Loop Thread
*/
void loop() {
    static unsigned long currentTime;

    // put your main code here, to run repeatedly:
    currentTime = millis();
    if (currentTime - prevtime_T1 > 100)
    {
       Transceiver_StateHandler();
       prevtime_T1 = currentTime; 
    }
    else if (currentTime - prevtime_T2 > 500)
    {
      /* check color state */
      Color_StateHandler();
      prevtime_T2 = currentTime; 
    }
    else if (currentTime - prevtime_T3 > 50)
    {
      Transceiver_MsgHandler();
      prevtime_T3 = currentTime;
    }
}
