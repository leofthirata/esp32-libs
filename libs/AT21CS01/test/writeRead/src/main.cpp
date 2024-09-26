#include <Arduino.h>
#include "at21cs01.h"

#define SIO 12  // AT21CS01 connected to 
                // External pull-up NOT REQUIRED!
#define LED_PIN 25


static AT21CS01_Struct at21cs01_config;
static const uint8_t serialNumberSize = 8;
static uint8_t serialNumber[serialNumberSize];
uint8_t memoryRead[72];

void setup()
{
    Serial.begin(115200);
    vTaskDelay(1000);
    Serial.println("AT21CS01 ESP32 arduino driver");

    pinMode(LED_PIN, OUTPUT);

    at21cs01_config.pin = SIO;
    at21cs01_config.debug = 1;
    at21cs01_config.prefCommunicationSpeed = AT21CS01_SPEED_SLOW;
    
    at21cs01_config.setMode = pinMode;
    at21cs01_config.setLevel = digitalWrite;
    at21cs01_config.getLevel = digitalRead;
    at21cs01_config.timer_us = delayMicroseconds;

    at21cs01_Init(&at21cs01_config);
    at21cs01_Connect();

    uint8_t memoryArray[72];
    memset(memoryArray, 0, sizeof(memoryArray));

    for (int i = 0; i < sizeof(memoryArray) ; i++)
    {
       memoryArray[i] = i;
    }

    //write 9 pages of 8 bytes
    for (int k = 0; k < 9; k++)
    {
        at21cs01_WriteToAddress(k*8, &memoryArray[k*8], 8);
    }

    printf("Memory arrange:\r\n");
    for (int i = 0; i < sizeof(memoryArray); i++)
    {
        if((i > 1) && (i % 8 == 0))
            printf("\r\n");
        printf("%02X ", memoryArray[i]);
    }

    printf("\r\n\r\nWait 5s to try reading\r\n");
    vTaskDelay(5000);
}


void loop()
{
    printf("Serial Number:\r\n");
    memset(serialNumber, 0, sizeof(serialNumber));
    at21cs01_ReadSerialNumber(serialNumber, serialNumberSize);
    printf("\r\n");

    printf("72 bytes read from Memory:\r\n");
    memset(memoryRead, 0, sizeof(memoryRead));
    for(int k = 0; k < 9; k++)
    {
        at21cs01_ReadFromAddress(k*8, &memoryRead[k*8], 8);
    }
  

    REG_WRITE(0x3ff44008, (1 << LED_PIN)); // LED_PIN  = HIGH
    delay(2000);
    
    REG_WRITE(0x3ff4400C, (1 << LED_PIN)); // LED_PIN = LOW
    delay(2000);

    printf("\r\n\r\n");

}