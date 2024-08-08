#ifndef __MEMORY_HPP__
#define __MEMORY_HPP__

#include "stdint.h"
#include "main.h"

typedef struct 
{
    uint8_t memVer;
    uint8_t hwVer;
    uint8_t prefixSN;
    uint8_t loraID[3];
    uint8_t devAddr[4];
    uint8_t devEUI[8];
    uint8_t appEUI[8];
    uint8_t nwSKey[16];
    uint8_t appSKey[16];
    uint8_t bleMac[6];
    uint8_t imei[7];
} OTPMemory_t;

#define AT24C02D_MEMORY_SIZE 256
#define OTP_MEM_VER 0x01

void taskMemory(void* pvParamters);
void memoryInit(Isca_t *_m_isca);

uint8_t otpInit(void *parameter);
uint8_t otpRead(OTPMemory_t *memoryRead);

#endif //__MEMORY_HPP__