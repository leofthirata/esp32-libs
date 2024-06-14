#ifndef __LORA_HPP__
#define __LORA_HPP__

#include "main.h"

#define MAX_PAYLOAD_LORA 255
#define JOINREQ_NBTRIALS 3			   /**< Number of trials for the join request. */
#define LORA_CODINGRATE 1		// [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0   // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define LORAWAN_APP_DATA_BUFF_SIZE 64  /**< Size of the data to be transmitted. */

typedef enum 
{
	LORA_SM_WAIT_FOR_QUEUE = 0,
    LORA_SM_WAIT_FOR_EVENT,
	LORA_SM_P2P_TX,
	LORA_SM_P2P_TX_DONE,
	LORA_SM_P2P_TX_TIMEOUT,
	LORA_SM_P2P_RX_DONE,
	LORA_SM_P2P_RX_TIMEOUT,
	LORA_SM_P2P_RX_ERROR,
	LORA_SM_LRW_TX,
	LORA_SM_LRW_TX_TIMEOUT,
	LORA_SM_LRW_TX_STATUS,
	LORA_SM_LRW_RX_DONE,
	LORA_SM_LRW_RX_TIMEOUT,
	LORA_SM_LRW_RX_ERROR,
} LoRa_SM_t;

typedef enum
{
	QUEUE_NONE = 0,
	QUEUE_SEND_P2P,
	QUEUE_SEND_LRW,
	QUEUE_SEND_STATUS,
} LoRa_Queue_t;

typedef struct 
{
	uint8_t	 payload[MAX_PAYLOAD_LORA];
	uint16_t size;
	int16_t rssi;
	int16_t snr;
} RX_Packet_t;


void loraTask(void* param);
void queueLRW();
void queueP2P();

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
        uint8_t sequenceNumber : 5;
        uint8_t protocolVersion : 3;
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
#endif //__LORA_HPP__