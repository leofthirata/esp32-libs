#ifndef __LORA_HPP__
#define __LORA_HPP__

#include "main.h"

#define LORA_MAX_PAYLOAD 128
#define JOINREQ_NBTRIALS 3			   /**< Number of trials for the join request. */
#define LORA_CODINGRATE 1		// [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0   // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define LORAWAN_APP_DATA_BUFF_SIZE 64  /**< Size of the data to be transmitted. */
#define LORA_TX_QUEUE_SIZE 10


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
	LORA_SM_LRW_STATUS_TX,
	LORA_SM_LRW_RX_DONE,
	LORA_SM_LRW_RX_TIMEOUT,
	LORA_SM_LRW_RX_ERROR,
} LoRaSM_t;

typedef enum
{
    TYPE_P2P_TX,
    TYPE_P2P_RX,
    TYPE_LRW_TX,
    TYPE_LRW_RX
} LoRaType_t;

typedef struct {
	uint8_t	 buffer[LORA_MAX_PAYLOAD];
	uint16_t size;
} LoRaPayload_t;

typedef struct 
{
    uint32_t txFreq;
    uint8_t txPower;
    uint8_t txTimeout;    
    uint8_t BW;
    uint8_t SF;
    uint8_t CR;
    uint32_t rxFreq;
    uint32_t rxDelay;
    uint32_t rxTimeout;
} LoRaP2PReqParams_t;

typedef struct 
{
    bool confirmed;
    uint8_t port;
} LoRaLRWTxReqParams_t;

typedef struct 
{
    uint32_t upLinkCounter;
    uint8_t channel;
    uint16_t length;
} LoRaLRWTxResParams_t;

typedef struct 
{
    int16_t rssi;
    int16_t snr;
} LoRaP2PRxParams_t;

typedef struct 
{
    uint32_t freq;
    int16_t rssi;
    int16_t snr;
    uint8_t port;
} LoRaLRWRxParams_t;

typedef struct
{
    LoRaPayload_t payload;
    LoRaP2PReqParams_t   params;
} LoRaP2PReq_t;

typedef struct
{
    LoRaPayload_t payload;
    LoRaP2PRxParams_t   params;
} LoRaP2PRx_t;

typedef struct
{
    LoRaPayload_t payload;
    LoRaLRWTxReqParams_t   params;
} LoRaLRWTxReq_t;

typedef struct
{
    LoRaPayload_t payload;
    LoRaLRWTxReqParams_t   params;
    LoRaLRWTxResParams_t done;
} LoRaLRWTxRes_t;

typedef struct
{
    LoRaPayload_t payload;
    LoRaLRWRxParams_t   params;
} LoRaLRWRx_t;

typedef enum
{
	QUEUE_NONE = 0,
	QUEUE_SEND_P2P,
	QUEUE_SEND_LRW,
	QUEUE_SEND_STATUS,
} LoRaQueueType_t;

typedef struct
{
    LoRaQueueType_t type;
} LoRaQueueElement_t;

void loraTask(void* param);

#endif //__LORA_HPP__