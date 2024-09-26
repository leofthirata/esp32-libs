#ifndef __AT21CS01_H__
#define __AT21CS01_H__

//Adapted from https://github.com/KOTzulla/stm32_at21cs01

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>

#define AT21CS01_SPEED_SLOW 0xD0
#define AT21CS01_SPEED_FAST 0xE0

#define AT21CS01_CMD_EEPROMREADWRITE 0xA0
#define AT21CS01_CMD_SERIALNUMBERREAD 0xB0

#define AT21CS01_PIN_LOW   0x0
#define AT21CS01_PIN_HIGH  0x1

#define AT21CS01_MODE_OUTPUT_OD_PU      0x17
#define AT21CS01_MODE_OUTPUT		0x03

typedef struct {
	uint8_t pin;
	uint8_t debug;
	uint8_t prefCommunicationSpeed;
    void (*setLevel)(uint8_t pin, uint8_t value);
    void (*setMode)(uint8_t pin, uint8_t mode);
    int (*getLevel)(uint8_t pin);
	void (*timer_us)(uint32_t timeout);
} AT21CS01_Struct;

void at21cs01_Init(AT21CS01_Struct *settings);
uint8_t at21cs01_Connect(void);
void at21cs01_FillWholeMemory(uint8_t byte);
void at21cs01_ReadFromAddress(uint8_t address, uint8_t *readData, uint8_t size);
uint8_t at21cs01_WriteToAddress(uint8_t address, uint8_t Data[],uint8_t size);
void at21cs01_ReadSerialNumber(uint8_t *readData, uint8_t size);

#ifdef __cplusplus
}
#endif

#endif //__AT21CS01_H__
