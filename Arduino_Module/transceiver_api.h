/*
 * transceiver_api.h
 *
 *  Created on: Jan 17, 2024
 *      Author: SunnyTeknoloji
 */

#ifndef TRANSCEIVER_API_H
#define TRANSCEIVER_API_H



#ifdef  TRANSCEIVER_API_CPP
#define INTERFACE
#else 
#define INTERFACE	extern
#endif


#include <Arduino.h>

/*******************************************************************************
* EXPORTED TYPES
********************************************************************************/
typedef enum _RoboState{
    STATE_IDLE,
    STATE_READY,
    STATE_START,
    STATE_FINISH,
    STATE_MAX
}RoboState;

typedef enum colorState{
    COLOR_NULL,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_MAX,
}_colorState;


typedef struct roboData{
    String name;            // robo module name
    byte id;                // robo ID
    byte state;
    _colorState colorState;  // reading color state
}_roboData;



/*******************************************************************************
* EXPORTED DEFS
********************************************************************************/
#define     DECIMAL_TOHEX(value) (((value & 0xFF) << 8) | ((~value) & 0xFF))

    /* commands ID for communication between STMStation module */
#define     CMD_READY   0x0B
#define     CMD_START   0x0C
#define     CMD_FINISH  0x0D

#define     CMD_ACK     0x10
#define     CMD_NACK    0xF0



#define     CMD_ADR_ALL 0x6229  //0xFF00      // received command will be processed by all robovehicle ID


#define     ROBOT_NAME  "RoboRally"
#define     ROBOT_ID    16
#define     ROBOT_ADDR  DECIMAL_TOHEX(ROBOT_ID)

#define     LED_PIN     7
#define     START_PIN   4


#define     eeADDR_DATA 0x10        // data stored memory at 16

/*******************************************************************************
* EXPORTED VARS
********************************************************************************/
extern _roboData RoboParams;

/*******************************************************************************
* EXPORTED FUNCTIONS
********************************************************************************/
INTERFACE void Transceiver_Init(void);
INTERFACE void Transceiver_StateHandler(void);
INTERFACE void Color_StateHandler(void);
INTERFACE void Transceiver_MsgHandler(void);




#undef	INTERFACE
#endif  /* TRANSCEIVER_API_H */