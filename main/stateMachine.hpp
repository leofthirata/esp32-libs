#ifndef __STATEMACHINE_HPP__
#define __STATEMACHINE_HPP__

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
    SM_SENSORS_STATUS,
    SM_P2P_TX_REQ,
    SM_P2P_TX_RES,
    SM_P2P_RX,
    SM_LRW_TX_REQ,
    SM_LRW_TX_RES,
    SM_LRW_RX,
    SM_SEND_LRW_STATUS,
    SM_GSM_TX_REQ,
    SM_GSM_TX_RES,
    SM_GSM_RX,
} Isca_SM_t;


/** APP event declarations */
typedef enum {
    APP_EVENT_P2P_TX_REQ,
    APP_EVENT_P2P_TX_RES,
    APP_EVENT_P2P_RX,
    APP_EVENT_LRW_TX_REQ,
    APP_EVENT_LRW_TX_RES,
    APP_EVENT_LRW_RX,
    APP_EVENT_GSM_TX_REQ,
    APP_EVENT_GSM_TX_RES,
    APP_EVENT_GSM_RX,
    APP_EVENT_REQ_LRW_SEND_STATUS,
    APP_EVENT_SEND_POS,
    APP_EVENT_SENSORS_STATUS,
    APP_EVENT_MOVEMENT,
    APP_EVENT_BAT_STATUS,
} app_event_t;

ESP_EVENT_DEFINE_BASE(APP_EVENT);

#define QUEUE_LRW_RX_SIZE   3
#define QUEUE_P2P_RX_SIZE   10
#define QUEUE_GSM_RX_SIZE   3
#define TIMEOUT_STATE_MACHINE   10000
#define BATT_CRITICAL_VOLTAGE   3500

#pragma pack(1)

 /* --------------- Command P2P --------------- */
typedef struct
{
    struct
    {
        uint8_t sequenceNumber : 5;
        uint8_t protocolVersion : 3;
    } header;
    uint8_t loraIdGw[3]; // lora id de quem vai enviar o comando
    uint8_t packetType;  // no caso de comandos sempre 0x41
    uint8_t crc8;
    uint8_t loraIdReceiveCommand[3];
    uint8_t param_desc1;
    uint8_t param_desc2;
    uint8_t param_desc3;
    uint8_t param_desc4;
    uint8_t loraEmergencyCommand;

} CommandP2P_t;

typedef union
{
    CommandP2P_t param;
    uint8_t array[sizeof(CommandP2P_t)];
} CommandP2PUnion_t;

/* --------------- Position P2P --------------- */
typedef struct
{
    struct
    {
        uint8_t sequenceNumber : 6;
        uint8_t protocolVersion : 2;
    } header;

    uint8_t loraId[3];
    uint8_t packetType; // no caso de posicoes sempre 0x50
    uint8_t crc8;

    int32_t latitude;
    int32_t longitude;
    struct
    {
        uint32_t headingGps : 9; // sugestao: usar quadrantes (8 quadrantes e 0x0 como sem gps)
        uint32_t accelerometerStatus : 1;
        uint32_t jammingDetectionStatus : 1;
        uint32_t gpsStatus : 1;
        uint32_t notused : 1;
        uint32_t criticalBatteryStatus : 1;
        uint32_t ignitionStatus : 1;
        uint32_t speedValue : 7; // verificar posibilidade remover
        uint32_t output1Status : 1;
        uint32_t powerSupplyStatus : 1;
        uint32_t emergencyStatus : 1;
        uint32_t batteryVoltageInfos : 2;
        uint32_t reservedForFutureUse : 5;
    } flags;

    uint8_t batteryVoltage;
} PositionP2P_t;

typedef union
{
    PositionP2P_t param;
    uint8_t array[sizeof(PositionP2P_t)];
} PositionP2PUnion_t;

#pragma pack()

#endif //__STATEMACHINE_HPP__
