#include "at21cs01.h"
#include "otp.hpp"
#include "main.h"
#include "stateMachine.hpp"

static AT21CS01_Struct at21cs01_config;

#define SIO 12 // AT21CS01 connected to
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

void otpRead(OTPMemory_t *memoryRead)
{
    uint8_t readFromMemory[72]; // 9 pages of 8 bytes each
    memset(readFromMemory, 0, sizeof(readFromMemory));

    OTPMemory_t parseData;
    uint8_t *parseData_p = (uint8_t *)&parseData;

    for (int k = 0; k < 9; k++)
    {
        at21cs01_ReadFromAddress(k * 8, (&readFromMemory[k * 8]), 8);
        uint8_t crc_cal = dallas_crc8(&(readFromMemory[k * 8]), 7);
        if (crc_cal != (readFromMemory[k * 8 + 7]))
        {
            printf("corrupted data %d, trying again\r\n", k);
            k--;
            delay(30);
        }
    }

    for (int i = 0; i < 8; i++)
    {
        memcpy((parseData_p + (i * 7)), (&readFromMemory[i * 8]), 7);
    }
    memcpy((parseData_p + 56), &readFromMemory[64], 2);

    memcpy(memoryRead, parseData_p, sizeof(OTPMemory_t));

    printf("    [PARSE] memVer: %d | hwVer: %d | prefixSN: %d \r\n", parseData.memVer,
           parseData.hwVer, parseData.prefixSN);
    uint32_t loraIDDec = (parseData.loraID[0] << 16) + (parseData.loraID[1] << 8) +
                         (parseData.loraID[2]);
    printf("        loraID: 0x%02X 0x%02X 0x%02X = %ld\r\n", parseData.loraID[0],
           parseData.loraID[1], parseData.loraID[2], loraIDDec);
    printf("        devAddress: 0x%02X 0x%02X 0x%02X 0x%02X\r\n", parseData.devAddr[0],
           parseData.devAddr[1], parseData.devAddr[2], parseData.devAddr[3]);
    printf("        devEUI: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\r\n", parseData.devEUI[0],
           parseData.devEUI[1], parseData.devEUI[2], parseData.devEUI[3], parseData.devEUI[4],
           parseData.devEUI[5], parseData.devEUI[6], parseData.devEUI[7]);
    printf("        appEUI: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\r\n", parseData.appEUI[0],
           parseData.appEUI[1], parseData.appEUI[2], parseData.appEUI[3], parseData.appEUI[4],
           parseData.appEUI[5], parseData.appEUI[6], parseData.appEUI[7]);
    printf("        nwSKey: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\r\n", parseData.nwSKey[0],
           parseData.nwSKey[1], parseData.nwSKey[2], parseData.nwSKey[3], parseData.nwSKey[4],
           parseData.nwSKey[5], parseData.nwSKey[6], parseData.nwSKey[7], parseData.nwSKey[8],
           parseData.nwSKey[9], parseData.nwSKey[10], parseData.nwSKey[11], parseData.nwSKey[12],
           parseData.nwSKey[13], parseData.nwSKey[14], parseData.nwSKey[15]);
    printf("        appSKey: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\r\n", parseData.appSKey[0],
           parseData.appSKey[1], parseData.appSKey[2], parseData.appSKey[3], parseData.appSKey[4],
           parseData.appSKey[5], parseData.appSKey[6], parseData.appSKey[7], parseData.appSKey[8],
           parseData.appSKey[9], parseData.appSKey[10], parseData.appSKey[11], parseData.appSKey[12],
           parseData.appSKey[13], parseData.appSKey[14], parseData.appSKey[15]);
}