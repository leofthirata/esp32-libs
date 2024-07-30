#ifndef __GSM_H__
#define __GSM_H__

#ifdef __cplusplus
extern "C" {
#endif
#pragma once
void gsmTask(void *pvParameters);

typedef struct
{
    int cell;   // 0 The serving cell, 1-6 The index of the neighboring cell
    int bcch;   // ARFCN(Absolute radio frequency channel number) of BCCH carrier, in decimal format
    int rxl;    // Receive level, in decimal format
    int rxq;    // Receive quality, in decimal format
    int mcc;    // Mobile country code, in decimal format
    int mnc;    // Mobile network code, in decimal format
    int bsic;   // Base station identity code, in decimal format
    int cellid; // Cell id, in hexadecimal format
    int rla;    // Receive level access minimum, in decimal format
    int txp;    // Transmit power maximum CCCH, in decimal format
    int lac;    // Location area code, in hexadecimal format
    int ta;     // Timing Advance, in decimal format
    int dbm;    // Receiving level in dBm
    int c1;     // C1 value
    int c2;     // C2 value
} mdm_lbs_cell_t;

typedef struct
{
    uint8_t input:1;
    uint8_t statusBattery:2;
    uint8_t online:1;
    uint8_t reserved:4;    
    uint8_t emergency:1;
    uint8_t lowBattery:1;
    uint8_t jammerGsm:1;
    uint8_t jammerLoRa:1;
    uint8_t movement:1;
    uint8_t bt:1;
    uint8_t stockMode:1;
    uint8_t output:1;
}GSMFlags_t;

typedef union
{
	GSMFlags_t asBit;
    uint8_t asArray[sizeof(GSMFlags_t)];
} GSMFlagsUnion_t;

typedef struct
{
    uint8_t header;
    uint8_t serialNumber[5];
    uint8_t imei[7];
    uint8_t fw[2];
    uint8_t hw;
    uint8_t protocol;
    uint8_t counter[2];
    uint8_t timestamp[4];
    uint8_t type;
    uint8_t loraID[3];
    uint8_t temp;
    uint8_t batteryVoltage[2];
    uint8_t crc;
    GSMFlagsUnion_t flags;
    uint8_t lastRst;
    uint8_t n_erbs;
} GSMTxPayload_t;

typedef struct
{
    char apn[64];
    char user[64];
    char pswd[64];
    uint16_t port;
    char server[64];
} GSMTxConfig_t;

typedef struct
{
    GSMTxPayload_t payload;
    GSMTxConfig_t config;
} GSMTxReq_t;

typedef struct
{
    uint8_t id[5];
    uint8_t mcc[2];
    uint8_t mnc[2];
    uint8_t bsic;
    uint8_t ta;
    uint8_t lac[5];
} GSMERBPacket_t;

#define NUM_MAX_ERB 7

typedef enum
{
    STATE_INIT,
    STATE_DEINIT,
    STATE_SYNC,
    STAT_CONFIG_PRMS,
    STATE_CHECK_SIGNAL,
    STATE_CHECK_OPERATOR,
    STATE_SCAN_NETWORKS,
    STATE_UPDATE_CLOCK,
    STATE_GET_IP,
    STATE_GET_LBS_POSITION,
    STATE_CHECK_IP,
    STATE_SEND_PACKET,
    STATE_RECEIVE_PACKET,
    STATE_SEND_PREV_PACKET,
    STATE_WAIT_EVENT,
    STATE_OPEN_SOCKET,
    STATE_CLOSE_SOCKET,
    STATE_SLEEP,
    STATE_POWERON,
    STATE_REPORT_ERROR,
} en_task_state;


typedef struct
{
    char base64[ESP_MODEM_C_API_STR_MAX];
    uint16_t size;
    en_task_state lastState;
    esp_err_t lastErr;
} GSMTxRes_t;


typedef struct
{
    char payload[ESP_MODEM_C_API_STR_MAX];
    uint16_t size;
} GSMRx_t;

#define NETLIGHT_REPORT_TIMEOUT 60000000 //us
#define NETLIGHT_OFF_TIMEOUT    3300000 //us

#define NETLIGHT_QUEUE_SIZE 5
#define GSM_RX_WINDOW  5000000 //us

typedef enum
{
    OFF,
    NOT_REGISTERED,
    REGISTERED,
    GPRS_CONNECTED,
} NetlightStatus_t;

typedef struct
{
    NetlightStatus_t status;
    NetlightStatus_t prev;
    unsigned long  positivePulseWidth;
    unsigned long negativePulseWidth;
    unsigned long lastNegativePulseStartTime;
} R800CNetlight_t;

const char* printState(en_task_state state);
char *generate_position_hex(mdm_lbs_cell_t *cells, GSMTxPayload_t *_data, size_t *size);
uint8_t crc8_itu(uint8_t *data, uint16_t length);
#ifdef __cplusplus
}
#endif


#endif //__GSM_H__