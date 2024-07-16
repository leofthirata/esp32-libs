#ifndef __MAIN_H__
#define __MAIN_H__

#pragma pack(1)

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

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

//configuration
    char* fwVer;
    uint8_t fwVerProtocol;
    uint8_t lrwProtocol;
    char apn[64];
    char gsmUser[64];
    char gsmPswd[64];
    uint16_t gsmPort;
    char gsmServer[64];
    uint32_t p2pTXFreq;
    uint32_t p2pRXFreq;
    uint8_t p2pBW;
    uint8_t p2pSF;
    uint8_t p2pCR;
    uint8_t lrwSF;
    bool lrwADR;
} Isca_t;

typedef struct
{
    uint8_t gsm[150];
} GSM_t;

#pragma pack()

#endif //__MAIN_H__