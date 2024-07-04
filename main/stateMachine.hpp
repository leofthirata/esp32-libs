#ifndef __STATEMACHINE_HPP__
#define __STATEMACHINE_HPP__
// #ifdef __cplusplus
// extern "C" {
// #endif

#include "main.h"
#include "esp_event.h"

void stateTask (void* pvParameters);

void enterEmergency(void);
void exitEmergency(void);
void turnOnBLE(void);
void turnOffBLE(void);
void enterStockMode(void);
void turnOnOutput(void);
void turnOffOutput(void);
void resetRequest(void);
void changeBLEPower(void);
void changeBLETime(void);
void changeLoRaTimes(uint32_t *timeArray);
void sendStatus(void);
void enableLed(void);
void disableLed(void);

typedef enum {
    P2P_SN_Time,
    P2P_SE_Time,
    P2P_MN_Time,
    P2P_ME_Time,
    LRW_SN_Time,
    LRW_SE_Time,
    LRW_MN_Time,
    LRW_ME_Time,
} TimeType_t;


void changeTime(TimeType_t type, uint32_t time);


typedef enum {
	SM_WAIT_FOR_EVENT = 0,
	SM_UPDATE_TIMERS,
    SM_ENTER_EMERGENCY,
	SM_EXIT_EMERGENCY,
    SM_SEND_P2P,
    SM_SEND_LRW,
    SM_SEND_P2P_DONE,
    SM_SEND_LRW_DONE,
    SM_RCV_P2P,
    SM_RCV_LRW,
} Isca_SM_t;


/** APP event declarations */
typedef enum {
    APP_EVENT_REQ_P2P_SEND,
    APP_EVENT_REQ_LRW_SEND,
    APP_EVENT_QUEUE_P2P_SEND,
    APP_EVENT_QUEUE_LRW_SEND,
    APP_EVENT_LRW_RX,
    APP_EVENT_P2P_RX,
    APP_EVENT_P2P_TX_DONE,
    APP_EVENT_LRW_TX_DONE,
} app_event_t;

ESP_EVENT_DEFINE_BASE(APP_EVENT);

#define QUEUE_LRW_RX_SIZE   3
#define QUEUE_P2P_RX_SIZE   10
#define TIMEOUT_STATE_MACHINE   10000

// #ifdef __cplusplus
// }
// #endif

#endif //__STATEMACHINE_HPP__
