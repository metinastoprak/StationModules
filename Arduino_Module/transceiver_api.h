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
    /* commands ID for communication between STMStation module */
#define     CMD_READY   0xEC
#define     CMD_START   0xED
#define     CMD_FINISH  0xEE

#define     CMD_ADR_ALL 0xFF        // received command will be processed by all robovehicle ID


#define     ROBOT_NAME  "RoboRally"
#define     ROBOT_ID    16

#define     COLOR_RED   0x10
#define     COLOR_RED   0x11


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





#undef	INTERFACE
#endif  /* TRANSCEIVER_API_H */