#include "at21cs01.h"
#include "otp.hpp"
#include "main.h"
#include "lora.hpp"
static AT21CS01_Struct at21cs01_config;
static const uint8_t serialNumberSize = 8;
static uint8_t serialNumber[serialNumberSize];
uint8_t memoryRead[72];
#define SIO 12  // AT21CS01 connected to 
                // External pull-up NOT REQUIRED!



void otpInit(void *parameter)
{
    at21cs01_config.pin = SIO;
    at21cs01_config.debug = 1;
    at21cs01_config.prefCommunicationSpeed = AT21CS01_SPEED_SLOW;
    
    at21cs01_config.setMode = pinMode;
    at21cs01_config.setLevel = digitalWrite;
    at21cs01_config.getLevel = digitalRead;
    at21cs01_config.timer_us = delayMicroseconds;

    at21cs01_Init(&at21cs01_config);
    at21cs01_Connect();
}

void otpRead(OTPMemoryUnion_t *memoryRead)
{
    memset(memoryRead->asArray, 0, sizeof(memoryRead->asArray));
    for(int k = 0; k < 9; k++)
    {
        at21cs01_ReadFromAddress(k*8, (memoryRead->asArray+k*8), 8);
        uint8_t crc_cal = dallas_crc8((memoryRead->asArray+k*8), 7);
        if(crc_cal != *(memoryRead->asArray+ k * 8 + 7))
        {
            printf("corrupted data %d, trying again\r\n", k);
            k--;
            delay(30);
        }
    }
    OTPMemoryUnion_t parseData;

    for(int i = 0; i < 8; i++)
    {
        memcpy(&parseData.asArray[i*7], (memoryRead->asArray + i*8), 7);
    }
    memcpy(&parseData.asArray[56], memoryRead->asArray + 64, 2);

    printf("    [PARSE] memVer: %d | hwVer: %d | prefixSN: %d \r\n", parseData.asParam.memVer,
            parseData.asParam.hwVer, parseData.asParam.prefixSN);
    uint32_t loraIDDec =  (parseData.asParam.loraID[0] << 16) + (parseData.asParam.loraID[1] << 8) + 
        (parseData.asParam.loraID[2]);
    printf("        loraID: 0x%02X 0x%02X 0x%02X = %ld\r\n", parseData.asParam.loraID[0], 
        parseData.asParam.loraID[1], parseData.asParam.loraID[2], loraIDDec);
    printf("        devAddress: 0x%02X 0x%02X 0x%02X 0x%02X\r\n", parseData.asParam.devAddr[0], 
        parseData.asParam.devAddr[1], parseData.asParam.devAddr[2],  parseData.asParam.devAddr[3]);
    printf("        devEUI: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\r\n", parseData.asParam.devEUI[0], 
        parseData.asParam.devEUI[1], parseData.asParam.devEUI[2],  parseData.asParam.devEUI[3], parseData.asParam.devEUI[4],
        parseData.asParam.devEUI[5], parseData.asParam.devEUI[6],  parseData.asParam.devEUI[7]);
    printf("        appEUI: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\r\n", parseData.asParam.appEUI[0], 
        parseData.asParam.appEUI[1], parseData.asParam.appEUI[2],  parseData.asParam.appEUI[3], parseData.asParam.appEUI[4],
        parseData.asParam.appEUI[5], parseData.asParam.appEUI[6],  parseData.asParam.appEUI[7]);
    printf("        nwSKey: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\r\n", parseData.asParam.nwSKey[0], 
        parseData.asParam.nwSKey[1], parseData.asParam.nwSKey[2],  parseData.asParam.nwSKey[3], parseData.asParam.nwSKey[4],
        parseData.asParam.nwSKey[5], parseData.asParam.nwSKey[6],  parseData.asParam.nwSKey[7], parseData.asParam.nwSKey[8],
        parseData.asParam.nwSKey[9], parseData.asParam.nwSKey[10],  parseData.asParam.nwSKey[11], parseData.asParam.nwSKey[12],
        parseData.asParam.nwSKey[13], parseData.asParam.nwSKey[14],  parseData.asParam.nwSKey[15]);
    printf("        appSKey: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\r\n", parseData.asParam.appSKey[0], 
        parseData.asParam.appSKey[1], parseData.asParam.appSKey[2],  parseData.asParam.appSKey[3], parseData.asParam.appSKey[4],
        parseData.asParam.appSKey[5], parseData.asParam.appSKey[6],  parseData.asParam.appSKey[7], parseData.asParam.appSKey[8],
        parseData.asParam.appSKey[9], parseData.asParam.appSKey[10],  parseData.asParam.appSKey[11], parseData.asParam.appSKey[12],
        parseData.asParam.appSKey[13], parseData.asParam.appSKey[14],  parseData.asParam.appSKey[15]);
}