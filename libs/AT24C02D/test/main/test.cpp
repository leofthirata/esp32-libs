#include "sdkconfig.h"

#include "esp_log.h"

#include "AT24C02D/at24c02d.hpp"
#include "freertos/task.h"

uint8_t mac[6];
static const char* TAG = "test.cpp";
uint8_t dallas_crc8(const uint8_t *pdata, const uint32_t size);

#define AT24C02D_MEMORY_SIZE 256
#define CONFIG_SDA_GPIO GPIO_NUM_27
#define CONFIG_SCL_GPIO GPIO_NUM_26
#define CONFIG_WP_GPIO GPIO_NUM_12

typedef struct 
{
    uint8_t memVer[2];
    uint8_t hwVer[2];
    uint8_t bleMac[4];
    uint8_t loraID[3];
    uint8_t devAddr[4];
    uint8_t devEUI[8];
    uint8_t appEUI[8];
    uint8_t nwSKey[16];
    uint8_t appSKey[16];
} OTPMemory_t;

typedef union 
{
    OTPMemory_t asParam;
    uint8_t asArray[sizeof(OTPMemory_t)];
}OTPMemoryUnion_t;

typedef struct 
{
    uint8_t memVer[2];
    uint8_t hwVer[2];
    uint8_t bleMac[4];
} Aux_t;

typedef union 
{
    Aux_t asParam;
    uint8_t asArray[sizeof(Aux_t)];
} AuxUnion_t;

uint8_t memoryRead[64];

void otp_read_no_crc(OTPMemoryUnion_t *memoryRead)
{
    EEPROM::EEPROM *dev = new EEPROM::EEPROM();
	
    i2c_port_t port = I2C_NUM_0;
    int sda = CONFIG_SDA_GPIO;
    int scl = CONFIG_SCL_GPIO;
    int wp = CONFIG_WP_GPIO;

	ESP_ERROR_CHECK(dev->init(port, sda, scl, wp, true));

    uint8_t readFromMemory[72];
    memset(readFromMemory, 0, sizeof(readFromMemory));
    OTPMemoryUnion_t parseData;

    //memset(memoryRead->asArray, 0, sizeof(memoryRead->asArray));
    for(int k = 0; k < 9; k++)
    {
        dev->at24c02d_read_byte(k*8, (&readFromMemory[k*8]), 8);
        uint8_t crc_cal = dallas_crc8(&(readFromMemory[k*8]), 7);
        if(crc_cal != (readFromMemory[k * 8 + 7]))
        {
            printf("corrupted data %d, trying again\r\n", k);
            k--;
            delay(30);
        }
    }

    for(int i = 0; i < 8; i++)
    {
        memcpy(&parseData.asArray[i*7], (&readFromMemory[i*8]), 7);
    }
    memcpy(&parseData.asArray[56], &readFromMemory[64], 2);

    memcpy(memoryRead, parseData.asArray, sizeof(parseData.asArray));

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

uint8_t dallas_crc8(const uint8_t *pdata, const uint32_t size)
{

	uint8_t crcr = 0;
	for (uint32_t i = 0; i < size; ++i)
	{
		uint8_t inbyte = pdata[i];
		for (uint8_t j = 0; j < 8; ++j)
		{
			uint8_t mix = (crcr ^ inbyte) & 0x01;
			crcr >>= 1;
			if (mix)
				crcr ^= 0x8C;
			inbyte >>= 1;
		}
	}
	return crcr;
}

extern "C" void app_main(void)
{
    EEPROM::EEPROM *dev = new EEPROM::EEPROM();
	
    i2c_port_t port = I2C_NUM_0;
    int sda = CONFIG_SDA_GPIO;
    int scl = CONFIG_SCL_GPIO;
    int wp = CONFIG_WP_GPIO;

	ESP_ERROR_CHECK(dev->init(port, sda, scl, wp, true));

	esp_err_t err = ESP_OK;
	uint8_t buffer[AT24C02D_MEMORY_SIZE];
    uint8_t data = 0xFF;
    uint8_t myBuff[16];
    uint8_t swp;

    dev->at24c02d_read_uid(myBuff);
    ESP_LOG_BUFFER_HEX("AT24C02D UID", myBuff, sizeof(myBuff));

    dev->at24c02d_read_swp(&swp);
    ESP_LOGW(TAG, "swp %d", swp);

    dev->at24c02d_reset_swp();

	// write entire memory
	for (uint16_t data_addr = 0; data_addr < AT24C02D_MEMORY_SIZE; data_addr++) 
    {
		dev->at24c02d_write_byte(data_addr, &data, 1);
	}

	// read entire memory
	for (uint16_t data_addr = 0; data_addr < AT24C02D_MEMORY_SIZE; data_addr++) 
    {
		dev->at24c02d_read_byte(data_addr, &buffer[data_addr], 1);
	}

    ESP_LOG_BUFFER_HEX("AT24C02D", buffer, sizeof(buffer));

    // write some address
    
    data = 0xAA;
    err = dev->at24c02d_write_byte(0x00, &data, 1);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "at24c02d_write_byte fail %s", esp_err_to_name(err));
    }

    data = 0xBB;
    err = dev->at24c02d_write_byte(0x01, &data, 1);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "at24c02d_write_byte fail %s", esp_err_to_name(err));
    }

    data = 0xCC;
    err = dev->at24c02d_write_byte(0x02, &data, 1);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "at24c02d_write_byte fail %s", esp_err_to_name(err));
    }

    uint8_t temp3[3];
    err = dev->at24c02d_read_byte(0x00, temp3, 3);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "at24c02d_read_byte fail %s", esp_err_to_name(err));
    }
    ESP_LOG_BUFFER_HEX("temp3", temp3, sizeof(temp3));

    OTPMemoryUnion_t otp;
    
    uint8_t bleMac[4] = { 0x0C, 0xAA, 0xBB, 0xCC };
    uint8_t loraId[4] = { 0x00, 0xC6, 0x5D, 0xBB };
    uint8_t devAddr[4] = { 0x00, 0x80, 0x4c, 0x28 };
    uint8_t devEui[8] = { 0xde, 0xa1, 0x4e, 0x2a, 0x60, 0x84, 0x4c, 0x94 };
    uint8_t appEui[8] = { 0x97, 0x7d, 0x60, 0xee, 0xaf, 0xc9, 0xee, 0x72 };
    uint8_t nwsKey[16] = { 0xbd, 0x75, 0x58, 0x87, 0x5d, 0xa2, 0xc8, 0x5c, 0xfb, 0x41, 0x14, 0xf3, 0xfa, 0x2a, 0xa1, 0x9d };
    uint8_t appsKey[16] = { 0xcd, 0x15, 0x64, 0xd6, 0x74, 0x27, 0x82, 0x84, 0x89, 0xd8, 0xb1, 0xe7, 0x7a, 0x6b, 0x59, 0x53 };
    
    uint8_t memVer[2] = { 0x00, 0x12 };
    uint8_t hwVer[2] = { 0x00, 0xab };

    memcpy(otp.asParam.memVer, memVer, 2);
    memcpy(otp.asParam.hwVer, hwVer, 2);
    memcpy(otp.asParam.bleMac, bleMac, 4);
    memcpy(otp.asParam.loraID, loraId, 3);
    memcpy(otp.asParam.devAddr, devAddr, 4);
    memcpy(otp.asParam.devEUI, devEui, 8);
    memcpy(otp.asParam.appEUI, appEui, 8);
    memcpy(otp.asParam.nwSKey, nwsKey, 16);
    memcpy(otp.asParam.appSKey, appsKey, 16);
    ESP_LOG_BUFFER_HEX("otp", otp.asArray, sizeof(OTPMemory_t));

    uint8_t otpCrc[72];
    memset(otpCrc, 0, 72);

    for(int i = 0; i < 9; i++)
    {
        memcpy(&otpCrc[i*8], &otp.asArray[i*7], 7);
        otpCrc[(i+1)*7+i] = dallas_crc8(&(otp.asArray[i*7]), 7);
        printf("otpCrc = %d", dallas_crc8(&(otp.asArray[i*7]), 7));
    }
    ESP_LOG_BUFFER_HEX("otpCrc", otpCrc, 72);

    err = dev->at24c02d_write_byte(0x00, otpCrc, 72);
    // err = dev->at24c02d_write_byte(0x02, hwVer, 2);
    // err = dev->at24c02d_write_byte(0x04, bleMac, 4);
    // err = dev->at24c02d_write_byte(0x08, &otp.asArray[3], sizeof(OTPMemory_t)-3);
    // err = dev->at24c02d_write_byte(0x08, loraId, 4);
    // err = dev->at24c02d_write_byte(0x0C, devAddr, 4);
    // err = dev->at24c02d_write_byte(0x10, devEui, 8);
    // err = dev->at24c02d_write_byte(0x18, appEui, 8);
    // err = dev->at24c02d_write_byte(0x20, nwsKey, 16);
    // err = dev->at24c02d_write_byte(0x30, appsKey, 16);

    uint8_t temp256[256];
    err = dev->at24c02d_read_byte(0x00, temp256, 256);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "at24c02d_read_byte fail %s", esp_err_to_name(err));
    }
    ESP_LOG_BUFFER_HEX("temp256", temp256, sizeof(temp256));

    dev->deinit();

    OTPMemoryUnion_t otp2;
    otp_read_no_crc(&otp2);
}