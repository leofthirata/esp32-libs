#ifndef __STATEMACHINE_HPP__
#define __STATEMACHINE_HPP__
#include "main.h"

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

void changeP2P_SN_Time(uint32_t time);
void changeP2P_SE_Time(uint32_t time);
void changeLRW_SN_Time(uint32_t time);
void changeLRW_SE_Time(uint32_t time);

typedef enum {
	SM_WAIT_FOR_EVENT = 0,
	SM_UPDATE_TIMERS,
    SM_ENTER_EMERGENCY,
	SM_EXIT_EMERGENCY,
    SM_SEND_P2P,
    SM_SEND_LRW,

} Isca_SM_t;

#endif //__STATEMACHINE_HPP__