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

/* --------------- Command LRW Dictionary --------------- */
typedef enum: uint8_t
{
  EMERGENCY       	= 0x01,
  BLUETOOTH			= 0x02,
  STOCK_MODE		= 0x03,
  SET_OUTPUT		= 0x04,
  DO_RESET			= 0x05,
  BLE_POWER			= 0x06,
  BLE_TIME			= 0x07,
  ALL_TIMES			= 0x08,
  GET_STATUS		= 0x09,
  LED				= 0x10,
  P2P_MOV_NOR		= 0x11,
  P2P_MOV_EMER		= 0x12,
  P2P_STP_NOR		= 0x13,
  P2P_STP_EMER		= 0x14,
  LRW_MOV_NOR		= 0x15,
  LRW_MOV_EMER		= 0x16,
  LRW_STP_NOR		= 0x17,
  LRW_STP_EMER		= 0x18,
} CommandLRWDict_t;

#define CMD_LRW_HEADER 				0xD7
#define CMD_PARAM_SIZE_EMER			01
#define CMD_PARAM_SIZE_BLE			01
#define CMD_PARAM_SIZE_STOCK		01
#define CMD_PARAM_SIZE_OUTPUT		01
#define CMD_PARAM_SIZE_RESET		01
#define CMD_PARAM_SIZE_BLE_POWER	01
#define CMD_PARAM_SIZE_BLE_TIME		01
#define CMD_PARAM_SIZE_ALL_TIMES	20
#define CMD_PARAM_SIZE_GET_STATUS	01
#define CMD_PARAM_SIZE_LED			01
#define TIME_CMD_PARAM_SIZE			03

typedef enum 
{
	LORA_SM_WAIT_FOR_SEND = 0,
    LORA_SM_WAIT_FOR_TIMEOUT,
	LORA_SM_P2P_TX,
	LORA_SM_P2P_TX_DONE,
	LORA_SM_P2P_TX_TIMEOUT,
	LORA_SM_P2P_RX_DONE,
	LORA_SM_P2P_RX_TIMEOUT,
	LORA_SM_P2P_RX_ERROR,
	LORA_SM_LRW_TX,
    LORA_SM_LRW_TX_DONE,
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
uint8_t dallas_crc8(const uint8_t *pdata, const uint32_t size);
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