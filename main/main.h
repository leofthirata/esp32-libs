#ifndef __MAIN_H__
#define __MAIN_H__

#define SOC_WIFI_SUPPORTED  1
#define ESP32	1
#define LIB_DEBUG 0
#define CORE_DEBUG_LEVEL	ARDUHAL_LOG_LEVEL_NONE
#define ESP_MODEM_C_API_STR_MAX 512
#include "Arduino.h"
#include "sdkconfig.h"

#define PROMPT_STR "fk-isca"

#define PIN_NUM_SWITCH 4
#define PIN_LED_STATUS_RED  GPIO_NUM_33
#define PIN_LED_STATUS_GREEN GPIO_NUM_25
#define PIN_NUM_SDA         27
#define PIN_NUM_SCL         26
#define PIN_NUM_LORA_RESET  19	 // LORA RESET
#define PIN_NUM_LORA_NSS    17	 // LORA SPI CS
#define PIN_NUM_LORA_SCLK   5	 // LORA SPI CLK
#define PIN_NUM_LORA_MISO   23	 // LORA SPI MISO
#define PIN_NUM_LORA_DIO_1  21 // LORA DIO_1
#define PIN_NUM_LORA_BUSY   22	 // LORA SPI BUSY
#define PIN_NUM_LORA_MOSI   18	 // LORA SPI MOSI
#define PIN_NUM_RADIO_TXEN   -1	 // LORA ANTENNA TX ENABLE
#define PIN_NUM_RADIO_RXEN   -1	 // LORA ANTENNA RX ENABLE

#define PIN_BAT_ADC_CTRL    GPIO_NUM_15

#define PIN_NUM_GSM_NETLIGHT 37

#define SIO 12 // AT21CS01 connected to
               // External pull-up NOT REQUIRED!

#define PIN_SDA_GPIO GPIO_NUM_27
#define PIN_SCL_GPIO GPIO_NUM_26
#define PIN_WP_GPIO GPIO_NUM_12

#define I2C_FREQUENCY   400000

#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 60										  /**< Maximum number of events in the scheduler queue. */

#define LORA_ID (0xD59F80)           // 14.000.000

// Define LoRa parameters

#define OVER_THE_AIR_ACTIVATION 0

#define P2P_POS_FREQ 903E6     // [Hz]
#define P2P_CMD_FREQ 904E6      // [Hz]

#define P2P_TX_POWER 22		
#define P2P_BANDWIDTH 2		
#define P2P_SPREADING_FACTOR 11 

#define P2P_RX_TIMEOUT 3000
#define P2P_TX_TIMEOUT 500

#define LRW_POS_PORT		5
#define LRW_STATUS_PORT		5
#define LRW_CMD_PORT		57

#define GSM_APN "simplepm.algar.br"
#define GSM_SERVER "0.tcp.sa.ngrok.io"
#define GSM_PORT 11611

// #define GSM_SERVER "mogno.ceabs.net"
// #define GSM_PORT 9015

#pragma pack(1)

typedef struct
{
    uint8_t statusBattery : 2;
    uint8_t lastResetReason : 5;
    uint8_t powerSupply : 1;
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
    uint64_t lastP2PTick;
    uint64_t lastLRWTick;
    uint64_t lastGSMTick;
    uint64_t P2PAlarm;
    uint64_t LRWAlarm;
    uint64_t GSMAlarm;
    uint8_t P2PCount;
    uint16_t GSMCount;
    int32_t acc[3];
    float temperatureFloat;
} IscaStatus_t;

typedef struct
{
    uint8_t hwVer;
    uint32_t loraId;
    uint8_t bleMac[6];
    uint8_t imei[7];
    uint8_t deviceEUI[8];
    uint8_t appEUI[8];
    uint8_t appKey[8];
    uint32_t devAddr;
    uint8_t nwkSKey[16];
    uint8_t appSKey[16];
} IscaROM_t;

typedef struct
{
    char apn[64];
    char user[64];
    char pswd[64];
    uint16_t port;
    char server[64];
}IscaGSMConfig_t;

typedef struct
{
    uint8_t sf; // [SF7..SF12]
    uint8_t bw; // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
    uint8_t cr;
    uint32_t txFreq;
    uint32_t txTimeout; //rx window (ms)
    uint8_t txPower; // dBm
    uint32_t rxFreq;
    uint32_t rxTimeout; //rx window (ms)
    uint32_t rxDelay; //delay from tx done (ms)
} IscaP2PConfig_t;

typedef struct
{
    uint8_t protocol;
    uint8_t sf;
    bool adr;
    bool confirmed;
    uint8_t posPort;
    uint8_t cmdPort;
    uint8_t statusPort;
} IscaLRWConfig_t;

typedef struct
{
	uint8_t power;
} IscaBLEConfig_t;

typedef struct
{
    char versionString[15];
    uint8_t versionByte;
} IscaFWConfig_t;

typedef struct
{
    IscaFWConfig_t fw;
    IscaGSMConfig_t gsm;
    IscaP2PConfig_t p2p;
    IscaLRWConfig_t lrw;
    IscaBLEConfig_t ble;
} IscaConfig_t;

typedef struct
{
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
    uint16_t bleAdvTime;
} IscaTimers_t;

typedef struct
{
	//status
    IscaStatus_t status;
    
    //rom
    IscaROM_t rom;

    //configuration
    IscaConfig_t config;
    
    //TIMERS
	IscaTimers_t timers;
} Isca_t;

#pragma pack()

void setup();
void loop();



#endif //__MAIN_H__