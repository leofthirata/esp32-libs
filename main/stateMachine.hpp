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

void changeP2P_SN_Time(uint32_t time);
void changeP2P_SE_Time(uint32_t time);
void changeLRW_SN_Time(uint32_t time);
void changeLRW_SE_Time(uint32_t time);

#define LORA_MAX_PAYLOAD 128

typedef enum {
	SM_WAIT_FOR_EVENT = 0,
	SM_UPDATE_TIMERS,
    SM_ENTER_EMERGENCY,
	SM_EXIT_EMERGENCY,
    SM_SEND_P2P,
    SM_SEND_LRW,
    SM_RCV_P2P,
    SM_RCV_LRW,
} Isca_SM_t;


/** IP event declarations */
typedef enum {
    APP_EVENT_REQ_P2P_SEND,
    APP_EVENT_REQ_LRW_SEND,
    APP_EVENT_QUEUE_P2P_SEND,
    APP_EVENT_QUEUE_LRW_SEND,
    APP_EVENT_LRW_RX,
    APP_EVENT_P2P_RX,
} app_event_t;


typedef struct 
{
    uint32_t freq;
    uint8_t txPower;    
    uint8_t BW;
    uint8_t SF;
    uint8_t CR;
	uint8_t	 buffer[LORA_MAX_PAYLOAD];
	uint16_t size;
    uint32_t freqRX;
    uint32_t delayRX;
    uint32_t timeoutRX;
} loraP2PTXParam_t;

typedef struct 
{
    int16_t rssi;
    int16_t snr;
	uint8_t	 buffer[LORA_MAX_PAYLOAD];
	uint16_t size;
} loraP2PRXParam_t;

typedef struct 
{
    bool confirmed;
    uint8_t port;
    uint32_t upLinkCounter;
    uint8_t channel;
    int8_t Datarate;
	int8_t TxPower;
	uint8_t	 buffer[LORA_MAX_PAYLOAD];
	uint16_t size;
} loraLRWTXParam_t;

typedef struct 
{
    uint32_t freq;
    int16_t rssi;
    int16_t snr;
    uint8_t port;
	uint8_t	 buffer[LORA_MAX_PAYLOAD];
	uint16_t size;
} loraLRWRXParam_t;


ESP_EVENT_DEFINE_BASE(APP_EVENT);



// #ifdef __cplusplus
// }
// #endif

#endif //__STATEMACHINE_HPP__
