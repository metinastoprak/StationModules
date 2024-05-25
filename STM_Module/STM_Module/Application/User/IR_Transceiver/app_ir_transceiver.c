/*
 * transceiver_app.c
 *
 *  Created on: March 17, 2024
 *      Author: SunnyTeknoloji
 */

#define	APP_IR_TRANSCEIVER_C

#include "stdio.h"
#include "main.h"
#include "app_ir_transceiver.h"
#include "app_netxduo.h"


/*******************************************************************************
* LOCAL DEFINES
********************************************************************************/
#define NEC_TIME_HEAD   13500
#define NEC_TIME_BIT0   1125
#define NEC_TIME_BIT1   2250
#define NEC_TIME_GAP    44000


#define NEC_TIME_BIT1_BOUNDARY_HI   (NEC_TIME_BIT1+(NEC_TIME_BIT1>>2))
#define NEC_TIME_BIT1_BOUNDARY_LO   (NEC_TIME_BIT1-(NEC_TIME_BIT1>>2))
#define NEC_TIME_BIT0_BOUNDARY_HI   (NEC_TIME_BIT0+(NEC_TIME_BIT0>>2))
#define NEC_TIME_BIT0_BOUNDARY_LO   (NEC_TIME_BIT0-(NEC_TIME_BIT0>>2))

#define NEC_PERIOD_US   26          // usec  38Khz modulation
#define NEC_PULSE_HEAD  346
#define NEC_PULSE_BIT   22

#define NEC_MARK_HEAD   (NEC_PULSE_HEAD*NEC_PERIOD_US)
#define NEC_SPACE_HEAD  (NEC_TIME_HEAD - NEC_MARK_HEAD)

#define NEC_MARK_BIT    (NEC_PULSE_BIT*NEC_PERIOD_US)
#define NEC_SPACE_BIT0  (NEC_TIME_BIT0 - NEC_MARK_BIT)
#define NEC_SPACE_BIT1  (NEC_TIME_BIT1 - NEC_MARK_BIT)



#define NEC_BIT_LENGHT      (16+16)
#define NEC_RAW_DATALENGHT  (NEC_BIT_LENGHT<<1)


/*******************************************************************************
* LOCAL TYPEDEFS 
********************************************************************************/
typedef struct {
    uint32_t rawTimerData[NEC_BIT_LENGHT+2];
    uint8_t decoded[4];

    NEC_RxState state;
    TIM_HandleTypeDef *timerHandle;

    uint32_t timerChannel;
    HAL_TIM_ActiveChannel timerChannelActive;

    uint8_t timeout;

    uint16_t timingHEADboundary;
    uint16_t timingBITboundary;

    uint16_t address;
    uint8_t command;

    void (*Rx_DecodedCallback)(uint8_t);
    void (*Rx_ErrorCallback)(uint8_t);
    void (*Rx_RepeatCallback)(uint8_t);
}t_NEC_RX;

typedef struct _t_NEC_TX {
    

    NEC_TxState state;
    TIM_HandleTypeDef *timerHandle;

    uint32_t timerChannel;
    HAL_TIM_ActiveChannel timerChannelActive;
    uint8_t timeout;

    uint8_t repeatCNT;

    uint8_t address;
    uint8_t command;
    uint32_t rawBitData;
    uint32_t encodedData;
    uint8_t readyMSG;
    uint8_t startMSG;


    uint8_t bitCount;
    uint16_t markspace[(NEC_BIT_LENGHT+2)<<1];

    void (*Tx_XmitCallback)();
}t_NEC_TX;

typedef struct _msgQueue {
    uint8_t id;
    uint8_t cmd;
    msgState state;
}t_msgQueue;

const char * NEC_rx_State[NEC_RX_STATE_MAX] = {" IDLE", \
                                                "INIT", \
                                                "HEAD_OK", \
                                                "HEAD_FAIL", \
                                                "ERROR", \
                                                "DONE"};



/*******************************************************************************
* LOCAL VARIABLES
********************************************************************************/
uint8_t thread_transceiver[THREAD_STACK_SIZE];
TX_THREAD transceiver_ptr;

uint8_t thread_portal[THREAD_STACK_SIZE];
TX_THREAD portal_ptr;

//CHAR queue_stack[QUEUE_SIZE*MESSAGE_SIZE];
//TX_QUEUE  Transceiver_queue_ptr;
//TX_QUEUE  Portal_queue_ptr;


// instansce NEC_rx and NEC_tx object
t_NEC_RX NEC_rx[2];
volatile t_NEC_TX NEC_tx;

t_msgQueue msgTransceiver;
t_msgQueue msgPortal;

t_msgQueue lastmsgReceived;


char logmsg[50];

/*******************************************************************************
* LOCAL FUNCTION PROTOTYPES
********************************************************************************/
VOID Transceiver_thread_entry(ULONG initial_param);
VOID Portal_thread_entry(ULONG initial_param);

void Transceiver_CheckQueueMessage(void);



// NEC -RX Protocol Functions 
void NEC_RX_Init(t_NEC_RX* handle);
void NEC_RX_DeInit(t_NEC_RX* handle);
void NEC_RX_TIM_IC_CaptureCallback(t_NEC_RX* handle);
void NEC_RX_StartCapture(t_NEC_RX* handle);
void NEC_RX_StopCapture(t_NEC_RX* handle) ;

// callback functions
void NEC_RX_DecodeHandler(IRsensor num);
void NEC_RX_ErrorHandler(IRsensor num);
void NEC_RX_RepeatHandler(IRsensor num);

// NEC -TX Protocol Functions 
void NEC_TX_Init(volatile t_NEC_TX* handle);
uint8_t NEC_TX_ReverseLSB(uint8_t data);

// callback functions
void NEC_TX_XmitHandler(volatile t_NEC_TX* handle);
void NEC_TX_SendMarkSpace(void);



static VOID App_Delay(ULONG Delay);
/*******************************************************************************
* FUNCTIONS
********************************************************************************/
/**
* @brief Function Name Transceiver_Init()
*/
void Transceiver_Init(void) {

    // main transceiver thread
    tx_thread_create(&transceiver_ptr, "Transceiver Thread", Transceiver_thread_entry, 0, thread_transceiver, THREAD_STACK_SIZE,
                         TRANSCEIVER_PRIORITY, TRANSCEIVER_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);


    //tx_thread_create(&portal_ptr, "Portal Thread", Portal_thread_entry, 0, thread_portal, THREAD_STACK_SIZE,
    //                     PORTAL_PRIORITY, PORTAL_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);

   
    tx_queue_create(&Transceiver_queue_ptr, "TransceiverQueue", MESSAGE_SIZE ,Transceiver_queue_stack,MESSAGE_SIZE*QUEUE_SIZE);
    tx_queue_create(&Portal_queue_ptr, "PortalQueue", MESSAGE_SIZE ,Portal_queue_stack,MESSAGE_SIZE*QUEUE_SIZE);

}

  /**
  * @brief  Transceiver thread entry function
  * @param  None
  * @retval None
  */
VOID Transceiver_thread_entry(ULONG initial_param) {
    

    NEC_RX_Init(&NEC_rx[SENSOR_1]);
    NEC_TX_Init(&NEC_tx);

    msgTransceiver.state = MSG_STATE_IDLE;
    msgPortal.state = MSG_STATE_IDLE;
    
    while (1)
    {
        tx_thread_sleep(20);        //100ms sleep

        switch(msgTransceiver.state)
        {
            case MSG_STATE_IDLE:
            {
                // check queue for incoming message?
                Transceiver_CheckQueueMessage();
                App_UDP_Thread_SendMESSAGE();

                if (msgTransceiver.state == MSG_STATE_IDLE ){
                    if (++NEC_tx.timeout >= 10){
                        NEC_tx.timeout = 0;
                        printf("\r[NEC TX] timeout, resend last packet \n");
                        if (NEC_tx.readyMSG){
                            printf("\r[NEC TX] timeout, resend READY comamnd\n");
                            msgTransceiver.state = MSG_STATE_READY;
                            NEC_tx.state = NEC_TX_STATE_IDLE;
                            msgTransceiver = lastmsgReceived;                   
                        }
                        else if (NEC_tx.startMSG){
                            printf("\r[NEC TX] timeout, resend START comamnd\n");
                            msgTransceiver.state = MSG_STATE_READY;
                            NEC_tx.state = NEC_TX_STATE_IDLE;
                            msgTransceiver = lastmsgReceived;                   
                        }
                    }
                }
                break;
            }
            case MSG_STATE_READY:
            {	
                if (NEC_tx.state == NEC_TX_STATE_IDLE)
                {
                    
                    NEC_RX_StopCapture(&NEC_rx[SENSOR_1]);        // disable receive
                    NEC_RX_StopCapture(&NEC_rx[SENSOR_2]);
                    if (msgTransceiver.cmd == CMD_READY) {
                        NEC_tx.address = CMD_ADR_ALL;    //NEC_rx.address>>8;
                        NEC_tx.command = CMD_READY;      //NEC_rx.command;
                        NEC_tx.timeout = 0;
                        NEC_tx.readyMSG = 1;
                        NEC_tx.startMSG = 0;
                        NEC_TX_XmitHandler(&NEC_tx);
                    }
                    else if(msgTransceiver.cmd == CMD_START){
                        NEC_tx.address = msgTransceiver.id;     //NEC_rx.address>>8;
                        NEC_tx.command = CMD_START;              //NEC_rx.command;
                        NEC_tx.timeout = 0;
                        NEC_tx.readyMSG = 0;
                        NEC_tx.startMSG = 1;
                        NEC_TX_XmitHandler(&NEC_tx);
                    }
                    else if (msgTransceiver.cmd  == CMD_FINISH){
                        NEC_tx.address = msgTransceiver.id;         //get received address
                        NEC_tx.command = CMD_ACK|CMD_FINISH;        //NEC_rx.command;
                        NEC_tx.timeout = 0;

                        printf("\r[NEC TX] send FINISH-ACK for id:%02d\n",NEC_tx.address);
                        NEC_TX_XmitHandler(&NEC_tx);
                    }

                }
                else if (NEC_tx.state == NEC_TX_STATE_TRANSMIT_DONE) {
                    printf("\r[NEC TX] transmit done ID:0x%X , Cmd:0x%X\n ",NEC_tx.address,NEC_tx.command);
                    NEC_tx.state = NEC_TX_STATE_IDLE;
                    msgTransceiver.state = MSG_STATE_PROCESS;
                }
                break;
            }
            case MSG_STATE_PROCESS:
            {	
                if (msgTransceiver.cmd  == CMD_FINISH) {
                    if (NEC_rx[SENSOR_1].state != NEC_RX_STATE_IDLE){
                        NEC_RX_StopCapture(&NEC_rx[SENSOR_1]);               // enable receive
                        printf("\r[NEC RX1] stopped after FINISH command\n");
                    }
                    if (NEC_rx[SENSOR_2].state != NEC_RX_STATE_IDLE){
                        NEC_RX_StopCapture(&NEC_rx[SENSOR_2]);               // enable receive
                        printf("\r[NEC RX2] stopped after FINISH command\n");
                    }

                }
                else {
                        NEC_RX_StartCapture(&NEC_rx[SENSOR_1]);               // enable receive....
                        NEC_RX_StartCapture(&NEC_rx[SENSOR_2]);               // enable receive....

                }
                msgTransceiver.state = MSG_STATE_IDLE;      // re-check queue message
                break;
            }
        }


    for(uint8_t ch=0;ch<SENSOR_MAX;ch++){
        switch(NEC_rx[ch].state)
        {
            case NEC_RX_HEAD_OK:
            {	// wait for 1sec to capture signal
                if (NEC_rx[ch].state != NEC_RX_STATE_INIT){
                    if (++NEC_rx[ch].timeout >= (TICK_1_SEC/10))
                    {
                        NEC_rx[ch].Rx_ErrorCallback(ch);                         
                    } 
                }

                break;
            }
            case NEC_RX_STATE_DONE:
            {	// if finish command received?
                
                if (++NEC_rx[ch].timeout >= (TICK_1_SEC/10))
                {
                    if (NEC_rx[ch].command == CMD_FINISH){
                        msgTransceiver.id = (NEC_rx[ch].address & 0xFF);
                        msgTransceiver.state = MSG_STATE_READY;         // send ACK command for FINISH
                        msgTransceiver.cmd = CMD_FINISH;
                    }
                    printf("\r[NEC RX%d] timeout, bus IDLE ...\n ",ch+1);
                    NEC_RX_StopCapture(&NEC_rx[ch]);
                }
                break;
            }
            default: 
            {
                if ((msgTransceiver.state == MSG_STATE_IDLE) && (NEC_tx.state == NEC_TX_STATE_IDLE || NEC_tx.state == NEC_TX_STATE_TRANSMIT_DONE)) {
                    //NEC_tx.state = NEC_TX_STATE_IDLE;

                    //printf("\r[NEC RX%d] Allstates at IDLE state...\n ",ch+1);
                    if (NEC_rx[ch].state != NEC_RX_STATE_INIT){
                        NEC_RX_StartCapture(&NEC_rx[ch]);
                    }    
                }
                break;
            }
        }
    }
#if 0
        if ((msgTransceiver.state == MSG_STATE_IDLE) && (NEC_tx.state == NEC_TX_STATE_IDLE || NEC_tx.state == NEC_TX_STATE_TRANSMIT_DONE)) {
            printf("\r[NEC RX] Allstates at IDLE state...\n ");
            NEC_tx.state = NEC_TX_STATE_IDLE;
            NEC_RX_StartCapture(&NEC_rx);
        }
        else if (NEC_rx.state == NEC_RX_HEAD_OK) {
            // wait for 1sec to capture signal    
            if (++NEC_rx.timeout >= (TICK_1_SEC/10))
            {
                NEC_rx.Rx_ErrorCallback();                         
            } 
        }
        else if (NEC_rx.state == NEC_RX_STATE_DONE) {
            // if finish command received?    
            if (NEC_rx.command == CMD_FINISH){
                NEC_RX_StopCapture(&NEC_rx);
                    
                NEC_tx.address = (NEC_rx.address & 0xFF);    //get received address
                NEC_tx.command = CMD_ACK|CMD_FINISH;         //NEC_rx.command;
                NEC_tx.timeout = 0;
                printf("\r[NEC TX] send FINISH-ACK for id:%02d\n",NEC_tx.address);
                NEC_TX_XmitHandler(&NEC_tx);
            }

            HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
            if (++NEC_rx.timeout >= (TICK_1_SEC/10))
            {
                printf("\r[NEC RX] timeout, bus IDLE ...\n ");
                if ((msgTransceiver.state == MSG_STATE_IDLE) && (NEC_tx.state == NEC_TX_STATE_IDLE))
                {
                    NEC_RX_StartCapture(&NEC_rx);
                }
                else 
                {
                    NEC_RX_StopCapture(&NEC_rx);
                }
            } 
        }
#endif            

        

        
    } //while(1)
}
  /**
  * @brief  Transceiver_CheckQueueMessage
  * @param  None
  * @retval None
  */
void Transceiver_CheckQueueMessage(void){
    UINT status;
    CHAR message[20+2] ="";

    CHAR *id_pos;       
    CHAR *cmd_pos;      
    CHAR *stat_pos;     
    unsigned char id, cmd, stat;
    static CHAR livecount= 0;


    if (++livecount >= 10){
        livecount = 0;
        printf("\r[Transceiver] RX1 State:%s, RX2 State:%s\n",NEC_rx_State[NEC_rx[SENSOR_1].state], NEC_rx_State[NEC_rx[SENSOR_2].state]);

        snprintf(logmsg, sizeof(logmsg), "RX1 State:%s, RX2 State:%s",NEC_rx_State[NEC_rx[SENSOR_1].state], NEC_rx_State[NEC_rx[SENSOR_2].state]);
        SENDLOG();
    }
    status = tx_queue_receive(&Transceiver_queue_ptr, &message, TX_NO_WAIT);
    if (status == TX_SUCCESS) {
        // Message received , parse it

        // get "id:"  "cmd:" "stat:" sub msg
        id_pos = strstr(message, "id:");
        cmd_pos = strstr(message, "cmd:");
        stat_pos = strstr(message, "stat:");

        // get values & convert to CHAR
        if (id_pos != NULL && cmd_pos != NULL && stat_pos != NULL) {
            id = (unsigned char)atoi(id_pos + 3); 
            cmd = (unsigned char)atoi(cmd_pos + 4);
            stat = (unsigned char)atoi(stat_pos + 5); 

            msgTransceiver.state = MSG_STATE_READY;
            msgTransceiver.id = id;
            msgTransceiver.cmd = cmd;
            lastmsgReceived = msgTransceiver; 
            printf("\r[Transceiver RecieveMsg] id: %u, cmd: %u, stat: %u\n", id, cmd, stat);

            snprintf(logmsg, sizeof(logmsg), "id: %u, cmd: %u, stat: %u", id, cmd, stat);
            SENDLOG();

        } 
        else {
            printf("\r[Transceiver RecieveMsg] ERROR id: cmd: stat:   received msg INVALID!\n");
        }
    }   
}
  /**
  * @brief  Portal thread entry function
  * @param  None
  * @retval None
  */
VOID Portal_thread_entry(ULONG initial_param) {
    UINT status;
    CHAR message[20+2];

    
    static UINT timerMSG = 0;

	while(1)	{

        // TX-RX thread dinleme
        status = tx_queue_receive(&Portal_queue_ptr, &message, TX_NO_WAIT);
        if (status == TX_SUCCESS) {
            // Message received , parse it
            printf("[Portal Queue] message received: %s\n", message);
        }   

        // check is there any msg avaliable to be sent?
        timerMSG++;
        if (timerMSG ==50) {

            //after 5sec send ready msg to TRANSCEIVER thread
            snprintf(message, sizeof(message), "id:%02d cmd:%02d stat:%02d", CMD_NULL, CMD_READY,CMD_NULL);       // addr:NULL --> ALL address  cmd:READY stat:NULL
            //snprintf(message, sizeof(message), "id:%02d cmd:%02d stat:%02d", 16, CMD_START,CMD_NULL);       // addr:ID  cmd:START stat:NULL

            status = tx_queue_send(&Transceiver_queue_ptr, message, TX_NO_WAIT);
            if (status == TX_SUCCESS) {
                printf("\r[Portal-->Transceiver] message send: %s\n", message);
            } 
        }
        else if (timerMSG == 100) {
            timerMSG = 0;
            //after 5sec send ready msg to TRANSCEIVER thread
            snprintf(message, sizeof(message), "id:%02d cmd:%02d stat:%02d", 16, CMD_START,CMD_NULL);       // addr:ID  cmd:START stat:NULL

            status = tx_queue_send(&Transceiver_queue_ptr, message, TX_NO_WAIT);
            if (status == TX_SUCCESS) {
                printf("\r[Portal-->Transceiver] message send: %s\n", message);
            } 
        }



        tx_thread_sleep(10);
	}
}
/**
 * @brief  NEC protocol decoded
 * @param  None
 * @retval None
 */
void NEC_RX_DecodeHandler(IRsensor num) {
    UINT status;
    CHAR message[20+2];
    CHAR ack = 1;

    //NEC_Rx_Read(&nec);
    printf("\r[NEC RX%d] frame captured Address:0x%X , Command:0x%X\n ",num+1,NEC_rx[num].address,NEC_rx[num].command);
    ack = NEC_rx[num].command>>4;
    NEC_rx[num].command &= 0x0F; 
    if (NEC_rx[num].command == CMD_READY || NEC_rx[num].command == CMD_START )
    {
        if (ack != 0x0F && ack != 0x01)
        {
            printf("\r[NEC RX%d] invalid ACK :%02d\n",num+1,ack);return;
        }
        if (ack == 0x01 && NEC_rx[num].command == CMD_READY){
            NEC_tx.readyMSG = 0;
        }
        if (ack == 0x01 && NEC_rx[num].command == CMD_START){
            NEC_tx.startMSG = 0;
        }

    }
    else if (NEC_rx[num].command == CMD_FINISH){
        printf("[NEC RX%d] FINISH command received\n ",num+1);
    }
    else
    {
        printf("[NEC RX%d] INVALID command received\n",num+1); return;
    }
    snprintf(message, sizeof(message), "id:%02d cmd:%02d stat:%02d", (NEC_rx[num].address & 0xFF), NEC_rx[num].command & 0x0f,ack);
    //sprintf(message, "id:%02d cmd:%02d stat:%02d", NEC_rx.address>>8, NEC_rx.command>>4,NEC_rx.command & 0x01);

    status = tx_queue_send(&Portal_queue_ptr, message, TX_NO_WAIT);
    if (status == TX_SUCCESS) {
        printf("\r[Transceiver-->Portal] message send: %s\n", message);

    }   

}
/**
 * @brief  NEC protocol error handler
 * @param  None
 * @retval None
 */
void NEC_RX_ErrorHandler(IRsensor ch) {

    printf("\r[NEC RX%d] Timeout or Error handled! \n ",ch+1);
    tx_thread_sleep(10);
    NEC_RX_StartCapture(&NEC_rx[ch]);
}
/**
 * @brief  NEC protocol repeat state handler
 * @param  None
 * @retval None
 */
void NEC_RX_RepeatHandler(IRsensor ch) {

    //NEC_Rx_Read(&nec);
    printf("\r[NEC RX%d] repeat \n ",ch+1);
    tx_thread_sleep(10);
    NEC_RX_StartCapture(&NEC_rx[ch]);
}





//***** NEC RX InputCapture Driver Functions *****// 
/**
 * @brief  NEC-RX initialize
 * @param  None
 * @retval None
 */
void NEC_RX_Init(t_NEC_RX* handle) {

    for (uint8_t sensor=0;sensor<SENSOR_MAX;sensor++){
        handle = &NEC_rx[sensor];
        handle->timerHandle = &htim1;

        if (sensor == 0){
            handle->timerChannel = TIM_CHANNEL_1;
            handle->timerChannelActive = HAL_TIM_ACTIVE_CHANNEL_1;
        }
        else{
            handle->timerChannel = TIM_CHANNEL_2;
            handle->timerChannelActive = HAL_TIM_ACTIVE_CHANNEL_2;
        }
        

        handle->timingBITboundary = (NEC_TIME_BIT0+NEC_TIME_BIT1)>>1;    //   1125 >---boundary---< 2250
        handle->timingHEADboundary = NEC_TIME_HEAD>>2;

        handle->Rx_DecodedCallback = NEC_RX_DecodeHandler;
        handle->Rx_ErrorCallback = NEC_RX_ErrorHandler;
        handle->Rx_RepeatCallback = NEC_RX_RepeatHandler;

    }
    printf("[NEC RX] state_init with TIM1_CH1 and TIM1_CH2\n ");

    //NEC_RX_StartCapture(handle);
}
/**
 * @brief  NEC-RX init state and start to capture signals
 * @param  None
 * @retval None
 */
void NEC_RX_StartCapture(t_NEC_RX* handle) {

    handle->state = NEC_RX_STATE_INIT;
    if (handle->timerChannel == TIM_CHANNEL_1)
        printf("\r[NEC RX-1] capture IR signal ready...\n ");
    else if (handle->timerChannel == TIM_CHANNEL_2)
        printf("\r[NEC RX-2] capture IR signal ready...\n ");

    handle->timeout = 0;
    handle->decoded[0]=0;handle->decoded[1]=0;
    handle->decoded[2]=0;handle->decoded[3]=0;
    HAL_TIM_IC_Stop_DMA(handle->timerHandle, handle->timerChannel);
    HAL_TIM_IC_Start_DMA(handle->timerHandle, handle->timerChannel,
            (uint32_t*) handle->rawTimerData, 4);

    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,0);
}
/**
 * @brief  NEC-RX de-init state and stop capture signals
 * @param  None
 * @retval None
 */
void NEC_RX_StopCapture(t_NEC_RX* handle) {

    handle->state = NEC_RX_STATE_IDLE;
    if (handle->timerChannel == TIM_CHANNEL_1)
        printf("\r[NEC RX-1] STOP capture IR signal\n ");
    else if (handle->timerChannel == TIM_CHANNEL_2)
        printf("\r[NEC RX-2] STOP capture IR signal\n ");

    handle->timeout = 0;
    handle->decoded[0]=0;handle->decoded[1]=0;
    handle->decoded[2]=0;handle->decoded[3]=0;
    HAL_TIM_IC_Stop_DMA(handle->timerHandle, handle->timerChannel);
}
/**
 * @brief  NEC-RX HAL_TIM_IC_CaptureCallback
 * @param  None
 * @retval None
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim1) {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
            NEC_RX_TIM_IC_CaptureCallback(&NEC_rx[SENSOR_1]);
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
            NEC_RX_TIM_IC_CaptureCallback(&NEC_rx[SENSOR_2]);
    }
}

/**
 * @brief  Input Capture  Callback function
 * @param  None
 * @retval None
 */
void NEC_RX_TIM_IC_CaptureCallback(t_NEC_RX *handle)
{

    uint16_t diffTime;
    uint8_t ch = handle->timerChannelActive-1; 
    handle->timeout = 0;
    if (handle->state == NEC_RX_STATE_INIT) {

        HAL_TIM_IC_Stop_DMA(handle->timerHandle, handle->timerChannel);
        diffTime = handle->rawTimerData[1] - handle->rawTimerData[0];
 
        if ( ((NEC_TIME_HEAD + handle->timingHEADboundary) > diffTime) && (diffTime > (NEC_TIME_HEAD - handle->timingHEADboundary)) ) {
            // Head sync captured
            handle->state = NEC_RX_HEAD_OK;
            HAL_TIM_IC_Start_DMA(handle->timerHandle, handle->timerChannel,
                    (uint32_t*) handle->rawTimerData+2, NEC_RAW_DATALENGHT);

        } 
        else {  // no sync - recapture
            handle->state = NEC_RX_STATE_INIT;
            HAL_TIM_IC_Start_DMA(handle->timerHandle, handle->timerChannel,
                    (uint32_t*) handle->rawTimerData, 4);
        }

    } else if (handle->state == NEC_RX_HEAD_OK) {

        HAL_TIM_IC_Stop_DMA(handle->timerHandle, handle->timerChannel);

        for (int pos = 0; pos < NEC_BIT_LENGHT; pos++) {
            diffTime = handle->rawTimerData[pos+2] - handle->rawTimerData[pos+1];
            if ( (NEC_TIME_BIT1_BOUNDARY_HI > diffTime) && (NEC_TIME_BIT1_BOUNDARY_LO < diffTime) ) {
                handle->decoded[pos / 8] |= 1 << (pos % 8);
            } 
            else if ( (NEC_TIME_BIT0_BOUNDARY_HI > diffTime) && (NEC_TIME_BIT0_BOUNDARY_LO < diffTime) ) {
                handle->decoded[pos / 8] &= ~(1 << (pos % 8));

            }
            else {   // bit timing error
                handle->Rx_ErrorCallback(ch);return;
            }
            handle->rawTimerData[pos] = diffTime; 
        }


        if ( (handle->decoded[2] ^ handle->decoded[3]) == 0xFF)
        {   // data matched
            handle->address = (handle->decoded[0]<<8)|handle->decoded[1];
            handle->command = handle->decoded[2];
            handle->state = NEC_RX_STATE_DONE;
            HAL_TIM_IC_Start_DMA(handle->timerHandle, handle->timerChannel,
            (uint32_t*) handle->rawTimerData, 2);

            handle->Rx_DecodedCallback(ch);
        }
        else {
            handle->Rx_ErrorCallback(ch);    
        }
    } else if (handle->state == NEC_RX_STATE_DONE) {
        HAL_TIM_IC_Start_DMA(handle->timerHandle, handle->timerChannel,
                (uint32_t*) handle->rawTimerData, 2);

    }

}

//********************************************************************************//
//********************************************************************************//

//***** NEC RX InputCapture Driver Functions *****// 
/**
 * @brief  NEC-RX initialize
 * @param  None
 * @retval None
 */
void NEC_TX_Init(volatile t_NEC_TX* handle) {
    handle->timerHandle = &htim2;

    handle->timerChannel = TIM_CHANNEL_1;
    handle->timerChannelActive = HAL_TIM_ACTIVE_CHANNEL_1;

    handle->Tx_XmitCallback = NEC_TX_SendMarkSpace;
    handle->state = NEC_TX_STATE_IDLE;
    handle->readyMSG = 0;
    handle->startMSG = 0;

    printf("[NEC TX] state_init with TIM2_CH1\n ");
}
/**
 * @brief  NEC-TX NEC_TX_XmitHandler
 * @param  address(8bit) command(8bit)
 * @retval None
 */
void NEC_TX_XmitHandler(volatile t_NEC_TX* handle) {

    uint8_t cnt;
    handle->rawBitData = NEC_TX_ReverseLSB(handle->address)<<24;
    handle->rawBitData |= NEC_TX_ReverseLSB(~handle->address)<<16;
    handle->rawBitData |= NEC_TX_ReverseLSB(handle->command)<<8;
    handle->rawBitData |= NEC_TX_ReverseLSB(~handle->command);

    handle->encodedData = handle->rawBitData;       // backup 32bit data

    handle->markspace[0] = NEC_MARK_HEAD;
    handle->markspace[1] = NEC_SPACE_HEAD;

    for(cnt=0;cnt<(NEC_BIT_LENGHT<<1);cnt++)
    {
        handle->markspace[cnt+2] = NEC_MARK_BIT;
        cnt++;
        if(handle->rawBitData & 0x80000000)
            handle->markspace[cnt+2] = NEC_SPACE_BIT1;    // logic1 Mark
        else
            handle->markspace[cnt+2] = NEC_SPACE_BIT0;

        handle->rawBitData <<=1;
    }
    // last mark
    handle->markspace[cnt+2] = NEC_MARK_BIT;
    handle->markspace[cnt+3] = NEC_TIME_GAP;
    for (cnt=0;cnt<5;cnt++)
    {
        handle->state = NEC_TX_STATE_TRANSMIT;
        handle->bitCount = 0;

//        handle->Tx_XmitCallback();
    if (__HAL_TIM_GET_IT_SOURCE(&htim17,TIM_IT_UPDATE) == RESET ){
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,9);
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

        __HAL_TIM_SET_COUNTER(&htim17,0);
        __HAL_TIM_SET_AUTORELOAD(&htim17,44000);
        HAL_TIM_Base_Start_IT(&htim17);
    }

        while (handle->state != NEC_TX_STATE_FRAME_OK);
    }

    handle->state = NEC_TX_STATE_TRANSMIT_DONE;
}
/**
 * @brief  NEC-TX NEC_TX_SendMarkSpace
 * @param  None
 * @retval None
 */
void NEC_TX_SendMarkSpace(void) {

    while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == GPIO_PIN_SET);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);


    // TIM_OCMODE_FORCED_INACTIVE
//    if (__HAL_TIM_GET_IT_SOURCE(&htim17,TIM_IT_UPDATE) == RESET ){
//        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,9);
//        HAL_TIM_Base_Start_IT(&htim17);
//    }

    __HAL_TIM_SET_AUTORELOAD(&htim17,NEC_tx.markspace[NEC_tx.bitCount]);
    __HAL_TIM_SET_COUNTER(&htim17,0);

    if (NEC_tx.bitCount & 0x01) // mark or space ?
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    else
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);


    if(++NEC_tx.bitCount >= (NEC_BIT_LENGHT+2)<<1)
    {
        HAL_TIM_Base_Stop_IT(&htim17);
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
        NEC_tx.state = NEC_TX_STATE_FRAME_OK;
    }

#if 0    
    // clear flag
    __HAL_TIM_CLEAR_IT(&htim2,TIM_IT_UPDATE);

    // Configure DMA. Use TIM UPDATE flag as DMA trigger, source=PWMduty, burst=1transfer, set HEAD pulse first
	HAL_TIM_DMABurst_MultiWriteStart(&htim2, TIM_DMABASE_CCR1, TIM_DMA_UPDATE, (uint32_t*)&PWMduty, TIM_DMABURSTLENGTH_1TRANSFER, pulse);

    // Set the Compare register to 0.
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);

    // Start PWM, no interrupt mode,  feed the DMA automatically after first cycle 
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    // When DMA finishes, call HAL_TIM_PeriodElapsedCallback
#endif    
}
/**
 * @brief  NEC_TX_TIM_PulseFinished_CaptureCallback
 * @param  TX handle pointer
 * @retval None
 */
void NEC_TX_TIM_PulseFinished_CaptureCallback(volatile t_NEC_TX * handle) {


    while (__HAL_TIM_GET_FLAG(&htim17,TIM_FLAG_UPDATE) == RESET);
    __HAL_TIM_CLEAR_FLAG(&htim17,TIM_FLAG_UPDATE);
    //HAL_TIM_Base_Stop(&htim17);
    

    if(++handle->bitCount >= 34)
    {
        __HAL_TIM_SET_AUTORELOAD(&htim17,NEC_TIME_GAP);
        // last GAP time
        while (__HAL_TIM_GET_FLAG(&htim17,TIM_FLAG_UPDATE) == RESET);
        HAL_TIM_Base_Stop(&htim17);
        handle->state = NEC_TX_STATE_FRAME_OK;
    }
    else {
        __HAL_TIM_SET_AUTORELOAD(&htim17,NEC_TIME_BIT0);        // logic0 Mark
        if(handle->rawBitData & 0x10000000)
            __HAL_TIM_SET_AUTORELOAD(&htim17,NEC_TIME_BIT1);    // logic1 Mark
        handle->rawBitData <<=1;
        NEC_TX_SendMarkSpace();
    }
}

// Timer output compare interrupt (When PWM goes low)
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){				// PWM done

	__HAL_TIM_CLEAR_IT(&htim2,TIM_IT_CC1);										// Clear flag
	__HAL_TIM_DISABLE_IT(&htim2,TIM_IT_CC1);									// Disable interrupt
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);									// Stop PWM (Or it will keep running with the last value)
	HAL_TIM_DMABurst_WriteStop(&htim2,TIM_DMA_UPDATE);							// Stop DMA

    if (htim == NEC_tx.timerHandle)
    {
        NEC_TX_TIM_PulseFinished_CaptureCallback(&NEC_tx);
    }
}
//HAL_TIM_PeriodElapsedCallback

/**
 * @brief  NEC-TX NEC_TX_ReverseLSB
 * @param  data(8bit)
 * @retval reversed byte
 */
uint8_t NEC_TX_ReverseLSB(uint8_t data) {

    uint8_t LSBfirst =0;
    for(uint8_t indx=0;indx<8;indx++)
    {
        LSBfirst |= (data & 0x01)<<(7-indx);
        data >>= 1;
    }
    return LSBfirst;
}

/**
  * @brief  Application Delay function.
  *   Delay : number of ticks(thread) to wait (1tick=10ms)
  * @retval None
  */
static void App_Delay(ULONG Delay)
{
  ULONG initial_time = tx_time_get();
  while ((tx_time_get() - initial_time) < Delay);
}




#if 0
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

            switch (IrReceiver.decodedIRData.command & 0x0F) {
            case CMD_READY:
                isReadyMsgReceived = true;
                if (RoboParams.state == STATE_IDLE || RoboParams.state == STATE_READY)
                {
                    digitalWrite(START_PIN, LOW);
                    if (RoboParams.colorState == COLOR_GREEN)
                    {
                        Serial.println("[Receive] READY msg, send ACK");
                        send_ir_data(ROBOT_ADDR,(CMD_ACK|CMD_READY),5);
                        RoboParams.state = STATE_READY;    
                    }
                    else
                    {
                        Serial.println("[Receive] READY msg, send NACK");
                        send_ir_data(ROBOT_ADDR,(CMD_NACK|CMD_READY),5);
                        RoboParams.state = STATE_IDLE;
                    } 
                } 
                break;

            case CMD_START:
                isStartMsgReceived = true;
                if (RoboParams.state == STATE_READY || RoboParams.state == STATE_START)
                {
                    Serial.println("[Receive] START msg, send ACK");
                    send_ir_data(ROBOT_ADDR,(CMD_ACK|CMD_START),5);
                    RoboParams.state = STATE_START;
                    // Active PIN High    
                    digitalWrite(START_PIN, HIGH);
                }
                else
                {
                    Serial.println("[Receive] START msg, send NACK");
                    send_ir_data(ROBOT_ADDR,(CMD_NACK|CMD_START),5);
                }
                break;

            case (CMD_ACK|CMD_FINISH):
                isFinishACKReceived = true;
                if (RoboParams.state == STATE_START)
                {
                    // race stopped
                    RoboParams.state = STATE_FINISH;
                    Serial.println("[Receive] FINISH ACK msg");
                }
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
			break;
		}
		case STATE_READY:
		{	
            displayWaitingMessage(); 
			break;
		}
		case STATE_START:
		{	
            if(RoboParams.colorState == COLOR_RED)
            {   //send repeatedly until receive ACK command
                digitalWrite(START_PIN, LOW);
                send_ir_data(ROBOT_ADDR,CMD_FINISH),5);
            }
            
            displayWaitingMessage();
			break;
		}
		case STATE_FINISH:
		{	
            Serial.println("******** Race COMPLETED ******");
            Serial.println("Press Power Off-ON to restart");
            while(1) displayWaitingMessage();
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
    delay(100);

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
    static char dotCnt = 0;

    if (++msgCnt >= 10) {
        msgCnt = 0;
        //Serial.print("\b");
        //Serial.print(PrintMsg_CommandWait[msgCnt]);
        Serial.print(".");
        if (++dotCnt == 10) {
            dotCnt = 0;
            Serial.println();
        }
    }
}
#endif
