#ifndef __OTP_HPP__
#define __OTP_HPP__

#include "stdint.h"

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
} OTPMemory_t;

void otpInit(void *parameter);
void otpRead(OTPMemory_t *memoryRead);

#define OTP_MEM_VER 0x01
#endif //__OTP_HPP__
