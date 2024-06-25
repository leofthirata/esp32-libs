#ifndef __MAIN_H__
#define __MAIN_H__

#define SOC_WIFI_SUPPORTED  1
#define ESP32	1
#define LIB_DEBUG 0
#define CORE_DEBUG_LEVEL	ARDUHAL_LOG_LEVEL_NONE

#include "Arduino.h"

#define PROMPT_STR "fk-isca"

#define PIN_NUM_SWITCH 4
#define PIN_LED_STATUS_RED  GPIO_NUM_33
#define PIN_LED_STATUS_GREEN GPIO_NUM_25
#define PIN_NUM_SDA         27
#define PIN_NUM_SCL         26

#define PIN_BAT_ADC_CTRL    GPIO_NUM_15

#define PIN_NUM_LORA_RESET  19	 // LORA RESET
#define PIN_NUM_LORA_NSS    17	 // LORA SPI CS
#define PIN_NUM_LORA_SCLK   5	 // LORA SPI CLK
#define PIN_NUM_LORA_MISO   23	 // LORA SPI MISO
#define PIN_NUM_LORA_DIO_1  21 // LORA DIO_1
#define PIN_NUM_LORA_BUSY   22	 // LORA SPI BUSY
#define PIN_NUM_LORA_MOSI   18	 // LORA SPI MOSI
#define PIN_NUM_RADIO_TXEN   -1	 // LORA ANTENNA TX ENABLE
#define PIN_NUM_RADIO_RXEN   -1	 // LORA ANTENNA RX ENABLE

#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 60										  /**< Maximum number of events in the scheduler queue. */

#define LORA_ID (0xD59F80)           // 14.000.000

// Define LoRa parameters

#define OVER_THE_AIR_ACTIVATION 0

#define LORA_P2P_FREQUENCY 903E6     // [Hz]
#define LORA_P2P_CMD_FREQUENCY 904E6 // [Hz]

#define TX_OUTPUT_POWER 22		// dBm
#define LORA_BANDWIDTH 2		// [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 11 // [SF7..SF12]

#define RX_TIMEOUT_VALUE 3000
#define TX_TIMEOUT_VALUE 3000

#define LRW_POS_PORT		5
#define LRW_STATUS_PORT		5
#define LRW_CMD_PORT		57


#pragma pack(1)

typedef struct
{
    uint8_t statusBattery : 2;
    uint8_t lastResetReason : 5;
    uint8_t reserved : 1;
    uint8_t emergency : 1;
    uint8_t lowBattery : 1;
    uint8_t jammer : 1;
    uint8_t movement : 1;
    uint8_t bleStatus : 1;
    uint8_t stockMode : 1;
    uint8_t output : 1;
    uint8_t input : 1;
} FlagsIsca_t;

typedef union
{
	FlagsIsca_t asBit;
    uint8_t asArray[sizeof(FlagsIsca_t)];
} FlagsIscaUnion_t;

typedef union
{
	uint64_t doubleWord : 40;
	uint8_t array[5];
} DoubleWordArrayUnion_t;


typedef struct
{
	FlagsIscaUnion_t flags;
    int16_t batteryMiliVolts;
    int8_t temperatureCelsius;
    uint32_t reset;
    uint8_t batteryStatus;
    uint32_t loraId;
    char* fwVer;
    uint8_t fwVerProtocol;
    uint8_t lrwProtocol;
    uint8_t hwVer;
    uint64_t lastP2PTick;
    uint64_t lastLRWTick;
	uint32_t p2pMovNorm : 20;
	uint32_t p2pMovEmer  : 20;
	uint32_t p2pStpNorm : 20;
	uint32_t p2pStpEmer  : 20;
	uint32_t lrwMovNorm : 20;
	uint32_t lrwMovEmer  : 20;
	uint32_t lrwStpNorm : 20;
	uint32_t lrwStpEmer  : 20;
    uint32_t gsmMovNorm : 20;
	uint32_t gsmMovEmer  : 20;
	uint32_t gsmStpNorm : 20;
	uint32_t gsmStpEmer  : 20;
	uint8_t blePower;
	uint16_t bleAdvTime;
    uint8_t bleMac[6];
    uint8_t P2PCount;
    int32_t acc[3];
    float temp;
    uint8_t nodeDeviceEUI[8];
    uint8_t nodeAppEUI[8];
    uint8_t nodeAppKey[8];
    uint32_t nodeDevAddr;
    uint8_t nodeNwsKey[16];
    uint8_t nodeAppsKey[16];
} Isca_t;

#pragma pack()

void setup();
void loop();



#endif //__MAIN_H__