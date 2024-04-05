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

const char * States_Table[STATE_MAX] = {"STATE_IDLE","STATE_READY","STATE_START","STATE_FINISH"};

const char * StateMsg_Table[STATE_MAX] = {"STATE_IDLE--> Waiting Ready msg", \
                                          "STATE_READY--> Waiting Start msg", \
                                          "STATE_START--> Race started ..", \
                                          "STATE_FINISH--> Race Finished !"};

const char * PrintMsg_CommandWait[4] = {"|","/","-","*"};



/*******************************************************************************
* LOCAL FUNCTION PROTOTYPES
********************************************************************************/
static void displayWaitingMessage(void);

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
    Serial.print(F(" id:" STR(ROBOT_ID) ));
    Serial.print(" state:");Serial.print(RoboParams.state);Serial.print("-");Serial.println(States_Table[RoboParams.state]);

    // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
    bStateTransition = false;

    //
}

/**
* @brief Function Name Transceiver_StateHandler()
*/
void Transceiver_StateHandler(void) {

    /* Print Message while State Transition occurs */
    if(bStateTransition == false) {
        Serial.println(StateMsg_Table[RoboParams.state]);
        bStateTransition = true;
    }
	switch(RoboParams.state)
	{
		case STATE_IDLE:
		{
            displayWaitingMessage();    
			break;
		}
		case STATE_READY:
		{	

			break;
		}
		case STATE_START:
		{	

			break;
		}
		case STATE_FINISH:
		{	

			break;
		}
	}



}
/**
* @brief Function Name Color_StateHandler()
*/
void Color_StateHandler(void) {



}


/**
* @brief Function Name displayWaitingMessage()
*/
static void displayWaitingMessage(void) {
    static char msgCnt = 0;

    if (++msgCnt >= 4)
        msgCnt = 0;
    
    Serial.print("\b");
    Serial.print(PrintMsg_CommandWait[msgCnt]);
}