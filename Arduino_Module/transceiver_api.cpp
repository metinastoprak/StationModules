/*
 * transceiver_api.cpp
 *
 *  Created on: Jan 17, 2024
 *      Author: SunnyTeknoloji
 */

#define	TRANSCEIVER_API_CPP

#include "transceiver_api.h"

/* Using NEC protocol for communicaitons */
#define DECODE_NEC          // Includes Apple and Onkyo. To enable all protocols 

/*
 * This include defines the actual pin number for pins like IR_RECEIVE_PIN, IR_SEND_PIN for many different boards and architectures
 */
#include "PinDefinitionsAndMore.h"
#include <IRremote.hpp> // include the library
#include <EEPROM.h>

#include <Wire.h>
#include <Adafruit_TCS34725.h>


/*******************************************************************************
* LOCAL DEFINES
********************************************************************************/


/*******************************************************************************
* LOCAL TYPEDEFS 
********************************************************************************/

/*
typedef struct xxxx
{

}structXXXX;
*/



/*******************************************************************************
* LOCAL VARIABLES
********************************************************************************/
_roboData RoboParams = {.name = ROBOT_NAME,.id = ROBOT_ID,.state = STATE_IDLE,.colorState = COLOR_NULL};

static bool bStateTransition = 0;
static bool isReadyMsgReceived = 0;
static bool isStartMsgReceived = 0;
static bool isFinishACKReceived = 0;




const char * States_Table[STATE_MAX] = {"STATE_IDLE","STATE_READY","STATE_START","STATE_FINISH"};

const char * StateMsg_Table[STATE_MAX] = {"STATE_IDLE--> Waiting Ready msg", \
                                          "STATE_READY--> Waiting Start msg", \
                                          "STATE_START--> Race started ..", \
                                          "STATE_FINISH--> Race Finished !"};

const char * PrintMsg_CommandWait[4] = {"|","/","-","*"};
const char * Colors_Table[COLOR_MAX] = {"COLOR Undefined","COLOR Red","COLOR Greeen"};

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

/*******************************************************************************
* LOCAL FUNCTION PROTOTYPES
********************************************************************************/
static void displayWaitingMessage(void);
void send_ir_data(uint16_t sAddress,uint8_t sCommand,uint8_t sRepeats);


/*******************************************************************************
* FUNCTIONS
********************************************************************************/
/**
* @brief Function Name Transceiver_Init()
*/
void Transceiver_Init(void) {

    _roboData tmpRoboData;

    RoboParams.id = ROBOT_ID;
    Serial.println("Transceiver_Init");

    // Get Initial Params
    EEPROM.get(eeADDR_DATA, tmpRoboData);

    // check whether params valid or not
    if (tmpRoboData.name == ROBOT_NAME && \
        tmpRoboData.id == ROBOT_ID && \
        tmpRoboData.state < STATE_MAX && \
        tmpRoboData.colorState < COLOR_MAX)
    {   // signature OKY
        RoboParams = tmpRoboData;
        Serial.print("RoboParams verify OK!.");
    }
    else
    {
        EEPROM.put(eeADDR_DATA, RoboParams);
        Serial.print("RoboParams initialized & re-stored. ");
    }

    //_roboData RoboParams = {.id = ROBOT_ID,.name = "Roborally",.state = STATE_IDLE,.colorState = COLOR_NULL};
    Serial.print(F(" id:" STR(ROBOT_ID) " id-addr:0x"));Serial.print(ROBOT_ADDR,HEX);
    Serial.print(" state:");Serial.println(States_Table[RoboParams.state]);

    // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
    bStateTransition = false;

    // Start with IR_SEND_PIN -which is defined in PinDefinitionsAndMore.h- as send pin and enable feedback LED at default feedback LED pin
    IrSender.begin(); 
    Serial.println(F("Send IR signals at pin " STR(IR_SEND_PIN)));

    digitalWrite(LED_PIN, LOW);
    if (tcs.begin()) {
        Serial.println("TCS34725 found..");
    } else {
        Serial.println("TCS34725 NOT FOUND!!");
    }
   
}

/**
* @brief Function Name Transceiver_MsgHandler()
*/
void Transceiver_MsgHandler(void) {

    if (IrReceiver.decode()) {
        Serial.println();
        /* Print a summary of received data  */
        if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
            Serial.println(F("Received noise or an unknown protocol"));
            // We have an unknown protocol here, print extended info
            IrReceiver.printIRResultRawFormatted(&Serial, true);
            IrReceiver.resume(); // Do it here, to preserve raw data for printing with printIRResultRawFormatted()

        } 
        else {  // matched protocol
            IrReceiver.printIRResultShort(&Serial);
            //IrReceiver.printIRSendUsage(&Serial);
            IrReceiver.resume(); // Early enable receiving of the next IR frame

        }
        Serial.println();
        

        /*
         * Finally, check the received data and perform actions according to the received command
         */
        if (IrReceiver.decodedIRData.address == ROBOT_ADDR || IrReceiver.decodedIRData.address == CMD_ADR_ALL ) {

            Serial.print("Robot ID matched--> 0x");Serial.println(IrReceiver.decodedIRData.address,HEX);

            bStateTransition = false;    // print current State 
            isReadyMsgReceived = false;
            isStartMsgReceived = false;
            isFinishACKReceived = false;

            switch (IrReceiver.decodedIRData.command) {
            case 0x5B: // READY msg from Station
                isReadyMsgReceived = true;
                break;

            case 0x0E: // START msg from Station
                isStartMsgReceived = true;
                break;

            default:
                IrReceiver.resume(); // Early enable receiving of the next IR frame
                break;
            }
        }
        else
        {
            IrReceiver.resume(); // Early enable receiving of the next IR frame
        }
        

    }
}
/**
* @brief Function Name Transceiver_StateHandler()
*/
void Transceiver_StateHandler(void) {

/* Print Message while State Transition occurs */
    if(bStateTransition == false) {
        Serial.println();
        Serial.print(StateMsg_Table[RoboParams.state]);Serial.print(",  ");
        Serial.println(Colors_Table[RoboParams.colorState]);
        bStateTransition = true;
    }
 	switch(RoboParams.state)
	{
		case STATE_IDLE:
		{
            displayWaitingMessage();
            if (isReadyMsgReceived == true) {
                 RoboParams.state = STATE_READY;
                 isReadyMsgReceived = false;
                 bStateTransition = false;
                 Serial.println("[Receive] READY msg, send ACK");

                send_ir_data(ROBOT_ADDR,0x56,5);
            }   
			break;
		}
		case STATE_READY:
		{	
            if (isReadyMsgReceived == true) {
                isReadyMsgReceived = false;
                Serial.println("[Receive] READY msg again, send ACK");

                send_ir_data(ROBOT_ADDR,0x56,5);
            }
            else if (isStartMsgReceived == true) {
                 RoboParams.state = STATE_START;
                 isStartMsgReceived = false;
                 bStateTransition = false;
                 Serial.println("[Receive] START msg, send ACK");

                send_ir_data(ROBOT_ADDR,0x56,5);
            }
            
            displayWaitingMessage(); 
			break;
		}
		case STATE_START:
		{	
            if (isStartMsgReceived == true) {
                isStartMsgReceived = false;
                Serial.println("[Receive] START msg again, send ACK");

                send_ir_data(ROBOT_ADDR,0x56,5);
            }
			break;
		}
		case STATE_FINISH:
		{	

			break;
		}
	}

   

}
/*
 * Send NEC2 IR protocol
 */
void send_ir_data(uint16_t sAddress,uint8_t sCommand,uint8_t sRepeats) {

    IrReceiver.stop();delay(500);

    Serial.print(F("[Transmit] addr: 0x"));
    Serial.print(sAddress, HEX);
    Serial.print(F(" data: 0x"));
    Serial.print(sCommand, HEX);Serial.println(" ]");
    Serial.flush(); // To avoid disturbing the software PWM generation by serial output interrupts

    // Results for the first loop to: Protocol=NEC2 Address=0x102 Command=0x34 Raw-Data=0xCB340102 (32 bits)
    IrSender.sendNEC2(sAddress, sCommand, sRepeats);
    delay(500);

    IrReceiver.start();
}
/**
* @brief Function Name Color_StateHandler()
*/
void Color_StateHandler(void) {
    uint16_t clear, red, green, blue;
    colorState _colorState = COLOR_NULL;

    tcs.getRawData(&red, &green, &blue, &clear);

    // RGB değerlerini normalize et
    float total = red + green + blue;
    float r = red / total;
    float g = green / total;
    float b = blue / total;
/*
    Serial.print("RED: "); Serial.print(r);
    Serial.print("GREEN: "); Serial.print(g);
    Serial.print("BLUE: "); Serial.println(b);
*/
    // Renk algılama
    if (r > 0.5 && g < 0.5 && b < 0.5) {
        _colorState = COLOR_RED;
    } else if (g > 0.5 && r < 0.5 && b < 0.5) {
        _colorState = COLOR_GREEN;
    }

    if (_colorState !=  RoboParams.colorState)
    {
        RoboParams.colorState = _colorState;
        bStateTransition = false;
    }
    /* if color = RED , set PORT=LOW & send finish command */

}


/**
* @brief Function Name displayWaitingMessage()
*/
static void displayWaitingMessage(void) {
    static char msgCnt = 0;

    if (++msgCnt >= 5) {
        msgCnt = 0;
        //Serial.print("\b");
        //Serial.print(PrintMsg_CommandWait[msgCnt]);
        Serial.print(".");
    }
}