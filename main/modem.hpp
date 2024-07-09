#ifndef __MODEM_HPP__
#define __MODEM_HPP__

#ifdef __cplusplus
extern "C" {
#endif
#pragma once
void modem_task_function(void *pvParameters);

typedef struct
{
    uint8_t input:1;
    uint8_t statusBattery:2;
    uint8_t reserved:5;    
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
} GSMElementTx_t;

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
#ifdef __cplusplus
}
#endif


#endif //__MODEM_HPP__