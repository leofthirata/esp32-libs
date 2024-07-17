#ifndef __GSM_HPP__
#define __GSM_HPP__

#ifdef __cplusplus
extern "C" {
#endif
#pragma once
void gsmTask(void *pvParameters);

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
    char apn[64];
    char user[64];
    char pswd[64];
    uint16_t port;
    char server[64];
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
    GSMTxReq_t params;
    GSMERBPacket_t erbs[NUM_MAX_ERB];
    en_task_state lastState;
    esp_err_t lastRet;
} GSMTxRes_t;


typedef struct
{
    //IP
    //TimeStamp
    //port
    // size
    //payload
} GSMRx_t;

const char* printState(en_task_state state);
const char *printError(esp_err_t ret);
#ifdef __cplusplus
}
#endif


#endif //__GSM_HPP__