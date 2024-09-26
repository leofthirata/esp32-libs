//Adapted from https://github.com/KOTzulla/stm32_at21cs01
#include "at21cs01.h"

static AT21CS01_Struct *at21cs01_Settings;
static uint8_t at21cs01_slaveAddress = 0;
static uint8_t CommSpeed = 0;

void at21cs01_Init(AT21CS01_Struct *settings)
{
    if ( (settings != NULL) && 
         (settings->setMode != NULL) && (settings->setLevel != NULL) &&
         (settings->getLevel != NULL) && (settings->timer_us != NULL) )
    {
        at21cs01_Settings = settings;
        at21cs01_Settings->setMode(at21cs01_Settings->pin, AT21CS01_MODE_OUTPUT_OD_PU);
        at21cs01_Settings->setLevel(at21cs01_Settings->pin, AT21CS01_PIN_HIGH);
    }
    else
    {
        if(at21cs01_Settings->debug)
            printf("[at21cs01] Error settings not defined!\r\n");
    }
}

void at21cs01_delay_us(uint32_t us)
{
    at21cs01_Settings->timer_us(us);
}
void at21cs01_delay_ms(uint16_t ms)
{
    at21cs01_Settings->timer_us(ms*1000);
}

void at21cs01_SIO_SetHigh()
{
    at21cs01_Settings->setLevel(at21cs01_Settings->pin, AT21CS01_PIN_HIGH);
}
void at21cs01_SIO_SetLow()
{
    at21cs01_Settings->setLevel(at21cs01_Settings->pin, AT21CS01_PIN_LOW);
}

uint8_t at21cs01_SIO_GetValue()
{
    uint8_t res = 0;

    if ((at21cs01_Settings->getLevel(at21cs01_Settings->pin)) != 0)
        res = 1;

    return res;
}

uint8_t at21cs01_ackNack()
{
    uint8_t temp = 0;

    if (CommSpeed == 0)
    {
        at21cs01_SIO_SetLow();
        at21cs01_delay_us(1);
        at21cs01_SIO_SetHigh();
        at21cs01_delay_us(1);
        if (at21cs01_SIO_GetValue() == 0)
        {
            temp = 0x00;
        }
        else
        {
            temp = 0xFF;
        }
        at21cs01_delay_us(9);
        at21cs01_SIO_SetHigh();
    }

    else
    {
        at21cs01_SIO_SetLow();
        at21cs01_delay_us(4);
        at21cs01_SIO_SetHigh();
        at21cs01_delay_us(2);
        if (at21cs01_SIO_GetValue() == 0)
        {
            temp = 0x00;
        }
        else
        {
            temp = 0xFF;
        }
        at21cs01_delay_us(34);
        at21cs01_SIO_SetHigh();
    }

    return temp;
}

uint8_t at21cs01_discoveryResponse()
{

    uint8_t temp = 0;

    at21cs01_SIO_SetHigh();
    at21cs01_delay_us(200);
    at21cs01_SIO_SetLow();
    at21cs01_delay_us(500);
    at21cs01_SIO_SetHigh();
    at21cs01_delay_us(20);

    at21cs01_SIO_SetLow();
    at21cs01_delay_us(1);
    at21cs01_SIO_SetHigh();
    at21cs01_delay_us(2);
    if (at21cs01_SIO_GetValue() == 0)
    {
        temp = 0x00;
    }
    else
    {
        temp = 0xFF;
    }
    at21cs01_delay_us(21);
    at21cs01_SIO_SetHigh();

    CommSpeed = 0;
    return temp;
}
void at21cs01_startHS()
{
    at21cs01_SIO_SetHigh();
    if (CommSpeed == 0)
    {
        at21cs01_delay_us(200);
    }

    else
    {
        at21cs01_delay_us(650);
    }
}

void at21cs01_tx1()
{
    if (CommSpeed == 0)
    {
        at21cs01_SIO_SetLow();
        at21cs01_delay_us(1);
        at21cs01_SIO_SetHigh();
        at21cs01_delay_us(14);
    }

    else
    {
        at21cs01_SIO_SetLow();
        at21cs01_delay_us(4);
        at21cs01_SIO_SetHigh();
        at21cs01_delay_us(41);
    }
}

void at21cs01_tx0()
{
    if (CommSpeed == 0)
    {
        at21cs01_SIO_SetLow();
        at21cs01_delay_us(10);
        at21cs01_SIO_SetHigh();
        at21cs01_delay_us(5);
    }

    else
    {
        at21cs01_SIO_SetLow();
        at21cs01_delay_us(24);
        at21cs01_SIO_SetHigh();
        at21cs01_delay_us(21);
    }
}

uint8_t at21cs01_txByte(uint8_t dataByte)
{
    uint8_t temp = 0;

    for (uint8_t ii = 0; ii < 8; ii++)
    {
        if (0x80 & dataByte)
        {
            at21cs01_tx1();
        }
        else
        {
            at21cs01_tx0();
        }
        dataByte <<= 1;
    }
    temp = at21cs01_ackNack();
    return temp;
}

uint8_t at21cs01_readByte()
{
    uint8_t temp = 0;
    uint8_t dataByte = 0x00;

    if (CommSpeed == 0)
    {
        for (int8_t ii = 0; ii < 8; ii++)
        {
            at21cs01_SIO_SetLow();
            at21cs01_delay_us(1);
            at21cs01_SIO_SetHigh();
            at21cs01_delay_us(1);
            asm("NOP");
            asm("NOP");
            temp = at21cs01_SIO_GetValue();
            temp &= 0x01;
            dataByte = (uint8_t)((dataByte << 1) | temp);
            at21cs01_delay_us(9);
        }
    }

    else
    {
        for (int8_t ii = 0; ii < 8; ii++)
        {
            at21cs01_SIO_SetLow();
            at21cs01_delay_us(4);
            at21cs01_SIO_SetHigh();
            at21cs01_delay_us(2);
            temp = at21cs01_SIO_GetValue();
            temp &= 0x01;
            dataByte = (uint8_t)((dataByte << 1) | temp);
            at21cs01_delay_us(34);
        }
    }
    return dataByte;
}

uint8_t at21cs01_scanDeviceAddress()
{
    uint8_t temp = 0;
    uint8_t address = 0;

    for (uint8_t ii = 0; ii < 8; ii++)
    {
        if (at21cs01_discoveryResponse() == 0)
        {
            at21cs01_startHS();
            temp = ii;
            temp <<= 1;
            temp |= 0xA0;
            if (at21cs01_txByte(temp) == 0x00)
            {
                address = ii;
                address <<= 1;
                at21cs01_startHS();
                break;
            }
        }
    }
    if (at21cs01_Settings->debug == 1)
    {
        printf("    Slave Address = ");
        if (address == 0x00)
        {
            printf("000b\r\n");
        }
        if (address == 0x02)
        {
            printf("001b\r\n");
        }
        if (address == 0x04)
        {
            printf("010b\r\n");
        }
        if (address == 0x06)
        {
            printf("011b\r\n");
        }
        if (address == 0x08)
        {
            printf("100b\r\n");
        }
        if (address == 0x0A)
        {
            printf("101b\r\n");
        }
        if (address == 0x0C)
        {
            printf("110b\r\n");
        }
        if (address == 0x0E)
        {
            printf("111b\r\n");
        }
    }
    at21cs01_slaveAddress = address;
    return address;
}

uint8_t at21cs01_eepromwrite(uint8_t cmd, uint8_t address, uint8_t Data[], uint8_t size)
{

    if (size > 8)
        return 1;
    cmd |= at21cs01_slaveAddress;

    at21cs01_startHS();
    at21cs01_txByte(cmd);
    at21cs01_txByte(address);
    for (uint8_t ii = 0; ii < size; ii++)
    {
        at21cs01_txByte(Data[ii]);
    }
    at21cs01_Settings->setMode(at21cs01_Settings->pin, AT21CS01_MODE_OUTPUT);
    at21cs01_startHS();
    at21cs01_delay_ms(5);

    at21cs01_Settings->setMode(at21cs01_Settings->pin, AT21CS01_MODE_OUTPUT_OD_PU);
    at21cs01_SIO_SetHigh();
    return 0;
}

uint8_t at21cs01_WriteToAddress(uint8_t address, uint8_t Data[], uint8_t size)
{
    return at21cs01_eepromwrite(AT21CS01_CMD_EEPROMREADWRITE, address, Data, size);
}

void at21cs01_ReadFromAddress(uint8_t address, uint8_t *readData, uint8_t size)
{

    uint8_t cmd = AT21CS01_CMD_EEPROMREADWRITE;
    cmd |= at21cs01_slaveAddress;

    at21cs01_startHS();
    at21cs01_txByte(cmd);
    at21cs01_txByte(address);
    at21cs01_startHS();
    at21cs01_txByte((uint8_t)(cmd | 0x01));
    for (uint8_t ii = 0; ii < size; ii++)
    {
        readData[ii] = at21cs01_readByte();
        if (ii < (size - 1))
        {
            at21cs01_tx0();
        }
    }
    at21cs01_tx1();
    at21cs01_startHS();
    if (at21cs01_Settings->debug == 1)
    {
        printf("    Data Read =");
        for (uint8_t ii = 0; ii < size; ii++)
        {
            printf(" 0x");
            printf("%02X", readData[ii]);
        }
        printf("\r\n");
    }
}

void at21cs01_currentRead(uint8_t cmd, uint8_t *readData, uint8_t size)
{

    cmd |= at21cs01_slaveAddress;

    at21cs01_startHS();
    at21cs01_txByte((uint8_t)(cmd | 0x01));
    for (uint8_t ii = 0; ii < size; ii++)
    {
        readData[ii] = at21cs01_readByte();
        if (ii < (size - 1))
        {
            at21cs01_tx0();
        }
    }
    at21cs01_tx1();
    at21cs01_startHS();
    if (at21cs01_Settings->debug == 1)
    {
        printf("    Current Address =");
        for (uint8_t ii = 0; ii < size; ii++)
        {
            printf(" 0x");
            printf("%02X", readData[ii]);
        }
        printf("\r\n");
    }
}

uint8_t at21cs01_setCommuncationSpeed(uint8_t speed)
{
    uint8_t temp = 0;
    speed |= at21cs01_slaveAddress;

    at21cs01_startHS();
    temp = at21cs01_txByte((uint8_t)(speed));
    at21cs01_startHS();
    if (((speed & 0xF0) == AT21CS01_SPEED_SLOW) & (temp == 0x00))
    {
        CommSpeed = 1;
    }
    else
    {
        CommSpeed = 0;
    }
    return temp;
}

uint8_t at21cs01_checkCommuncationSpeed(uint8_t speed)
{

    uint8_t temp = 0;
    speed |= at21cs01_slaveAddress;

    at21cs01_startHS();
    temp = at21cs01_txByte((uint8_t)(speed | 0x01));
    at21cs01_startHS();

    if (temp == 0x00)
    {
        if ((speed & 0xF0) == AT21CS01_SPEED_SLOW)
        {
            printf("    Standard Speed ACK\r\n");
        }
        if ((speed & 0xF0) == AT21CS01_SPEED_FAST)
        {
            printf("    High-Speed ACK\r\n");
        }
    }
    else
    {
        if ((speed & 0xF0) == AT21CS01_SPEED_SLOW)
        {
            printf("    Standard Speed NACK\r\n");
        }
        if ((speed & 0xF0) == AT21CS01_SPEED_FAST)
        {
            printf("    High-Speed NACK\r\n");
        }
    }
    return temp;
}

void at21cs01_FillWholeMemory(uint8_t byte)
{
    uint8_t data[128];
    memset(data, byte, 128);
    uint8_t addr = 0x00;
    for (uint8_t z = 0; z < 16; z++)
    {
        at21cs01_eepromwrite(AT21CS01_CMD_EEPROMREADWRITE, addr, data, 8);
        addr += 8;
    }
}

void at21cs01_ReadSerialNumber(uint8_t *readData, uint8_t size)
{

    uint8_t cmd = AT21CS01_CMD_SERIALNUMBERREAD;
    cmd |= at21cs01_slaveAddress;

    at21cs01_startHS();
    at21cs01_txByte(cmd);
    at21cs01_txByte(0x00);
    at21cs01_startHS();
    at21cs01_txByte((uint8_t)(cmd | 0x01));
    for (uint8_t ii = 0; ii < size; ii++)
    {
        readData[ii] = at21cs01_readByte();
        if (ii < (size - 1))
        {
            at21cs01_tx0();
        }
    }
    at21cs01_tx1();
    at21cs01_startHS();
    if (at21cs01_Settings->debug == 1)
    {
        printf("    Data Read =");
        for (uint8_t ii = 0; ii < size; ii++)
        {
            printf(" 0x");
            printf("%02X", readData[ii]);
        }
        printf("\r\n");
    }
}

uint8_t at21cs01_Connect()
{
    at21cs01_slaveAddress = at21cs01_scanDeviceAddress();
    at21cs01_setCommuncationSpeed(at21cs01_Settings->prefCommunicationSpeed);
    // if (at21cs01_Settings->debug == 1)
    return at21cs01_checkCommuncationSpeed(at21cs01_Settings->prefCommunicationSpeed);
}
