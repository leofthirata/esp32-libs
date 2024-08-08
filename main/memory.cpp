#include "memory.hpp"
#include "esp_mac.h"
#include "at21cs01.h"
#include "AT24C02D/at24c02d.hpp"
#include "esp_log.h"
#include "utils.h"

EEPROM::EEPROM *dev;
static bool isI2C = false;
static AT21CS01_Struct at21cs01_config;
uint8_t mac[6];
static OTPMemory_t readMemory;
static const char *TAG = "memory.cpp";

static RTC_DATA_ATTR Isca_t m_config;

//return 0 if got ack from otp
//return 1 if no i2c or onewire is present 
//return -1 if config fail
uint8_t otpInit(void *parameter)
{
       dev = new EEPROM::EEPROM();
       uint8_t ret = 0;
       ret = dev->init(I2C_NUM_0, (int)PIN_SDA_GPIO, (int)PIN_SCL_GPIO, PIN_WP_GPIO, true);
       if(ret != ESP_OK)
       {
              ESP_LOGE("OTP", "init I2C error %s", esp_err_to_name(ret));
              return -1;
       }
       uint8_t myBuff[16];

       if(dev->at24c02d_read_uid(myBuff) == ESP_OK)
       {
              isI2C = true;
              ESP_LOG_BUFFER_HEX("AT24C02D UID", myBuff, sizeof(myBuff));
       }
       else
       {
              isI2C = false;
              at21cs01_config.pin = SIO;
              at21cs01_config.debug = 1;
              at21cs01_config.prefCommunicationSpeed = AT21CS01_SPEED_SLOW;

              at21cs01_config.setMode = pinMode;
              at21cs01_config.setLevel = digitalWrite;
              at21cs01_config.getLevel = digitalRead;
              at21cs01_config.timer_us = delayMicroseconds;
               
              uint8_t numTries = 0;
              do
              {
                     at21cs01_Init(&at21cs01_config);
                     ret = at21cs01_Connect();
                     if(ret != 0) //NACK
                     {
                            vTaskDelay(pdMS_TO_TICKS(1000));
                     }

              } while (numTries++ < 4 && ret != 0);
       }
       dev->deinit();
       // delete[] dev;
       return ret;
}

uint8_t otpRead(OTPMemory_t *memoryRead)
{
       uint8_t readFromMemory[72]; // 9 pages of 8 bytes each
       memset(readFromMemory, 0, sizeof(readFromMemory));

       OTPMemory_t parseData;
       uint8_t *parseData_p = (uint8_t *)&parseData;
       uint8_t count = 0;
       uint8_t ret = 0;

       if(isI2C)
       {
              ret = dev->init(I2C_NUM_0, (int)PIN_SDA_GPIO, (int)PIN_SCL_GPIO, PIN_WP_GPIO, true);
              if(ret != ESP_OK)
              {
                     ESP_LOGE("OTP", "init I2C error %s", esp_err_to_name(ret));
                     return -1;
              }
       }
       for (int k = 0; k < 9; k++)
       {
              if(isI2C)
                     dev->at24c02d_read_byte(k * 8, (&readFromMemory[k * 8]), 8);
              else
                     at21cs01_ReadFromAddress(k * 8, (&readFromMemory[k * 8]), 8);
              

              uint8_t crc_cal = dallas_crc8(&(readFromMemory[k * 8]), 7);
              if (crc_cal != (readFromMemory[k * 8 + 7]))
              {
                     printf("corrupted data %d, trying again\r\n", k);
                     k--;
                     count++;
                     delay(3000);
              }
              if (count >= 5)
                     return -1;
       }

       if(isI2C)
       {
              dev->deinit();
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

       return 0;
}

void memoryInit(Isca_t *_m_isca)
{
//     IscaROM_t rom = {0};
    
//     if(storage_otp_get(&rom) != ESP_OK)
//     {
       uint8_t ret = otpInit(NULL);
       if(ret == 0)
       {
              ret = otpRead(&readMemory);
       }
       else
       {
              ESP_LOGE(TAG, "error OTP memory");
              OTPMemory_t hardCodedROM = {
                            .memVer = 0xDD,
                            .hwVer = 0xAA,
                            .prefixSN = 0x6D,
                            .loraID = {0xbe, 0xbc, 0x25},
                            .devAddr = {0x5d, 0x1f, 0x42, 0x61},
                            .devEUI = {0x34, 0xe5, 0x51, 0xdf, 0x6d, 0xab, 0xc5, 0x6b},
                            .appEUI = {0x2c, 0xa5, 0xae, 0x9d, 0xf4, 0xaf, 0x41, 0x4a},
                            .nwSKey = {0x45, 0x5d, 0xd8, 0xb2, 0x11, 0x65, 0x3e, 0x5c, 0xbd, 0xbf, 0xb1, 0x29, 0xa3, 0x29, 0xb2, 0x99},
                            .appSKey = {0x5e, 0xc0, 0x07, 0x25, 0x72, 0x47, 0xf1, 0xb0, 0x9a, 0xad, 0x8f, 0x61, 0xe8, 0xca, 0x4d, 0xd1}
              };
              memcpy(&readMemory, &hardCodedROM, sizeof(OTPMemory_t));
       }

       memset(_m_isca, 0, sizeof(Isca_t));
       _m_isca->rom.loraId = (readMemory.loraID[0]<<16) + (readMemory.loraID[1]<<8) + (readMemory.loraID[2]);
       memcpy(_m_isca->rom.deviceEUI, readMemory.devEUI, 8);
       memcpy(_m_isca->rom.appEUI, readMemory.appEUI, 8);
       _m_isca->rom.devAddr = (readMemory.devAddr[0]<<24) + (readMemory.devAddr[1]<<16) +
              (readMemory.devAddr[2]<<8) + readMemory.devAddr[3];
       memcpy(_m_isca->rom.nwkSKey, readMemory.nwSKey, 16);
       memcpy(_m_isca->rom.appSKey, readMemory.appSKey, 16);
       
       printf("loraID: %ld | devAddress: %08lX\r\n", _m_isca->rom.loraId,_m_isca->rom.devAddr);
       ESP_LOG_BUFFER_HEX("deviceEUI",_m_isca->rom.deviceEUI, sizeof(_m_isca->rom.deviceEUI));
       ESP_LOG_BUFFER_HEX("appEUI", _m_isca->rom.appEUI, sizeof(_m_isca->rom.appEUI));
       ESP_LOG_BUFFER_HEX("nwSKey", _m_isca->rom.nwkSKey, sizeof(_m_isca->rom.nwkSKey));
       ESP_LOG_BUFFER_HEX("appSKey", _m_isca->rom.appSKey, sizeof(_m_isca->rom.appSKey));

       // }
       // else
       // {

       // }
ESP_ERROR_CHECK(esp_efuse_mac_get_default(mac));
    ESP_ERROR_CHECK(esp_base_mac_addr_set(mac));
    ESP_LOGW(TAG, "BASE MAC: " MACSTR " ", MAC2STR(mac));

// esp_err_t esp_read_mac(uint8_t *mac, esp_mac_type_t type)

       esp_read_mac(mac, ESP_MAC_BT);
       ESP_LOGW(TAG, "BLE MAC: " MACSTR " ", MAC2STR(mac));
    memcpy(_m_isca->rom.bleMac, mac, sizeof(_m_isca->rom.bleMac));

    _m_isca->config.p2p.sf = P2P_SPREADING_FACTOR;
    _m_isca->config.p2p.bw = P2P_BANDWIDTH;
    _m_isca->config.p2p.cr = 1;
    _m_isca->config.p2p.txFreq = P2P_POS_FREQ;
    _m_isca->config.p2p.txPower = P2P_TX_POWER;
    _m_isca->config.p2p.rxFreq = P2P_CMD_FREQ;
    _m_isca->config.p2p.rxDelay = 0;
    _m_isca->config.p2p.rxTimeout = P2P_RX_TIMEOUT;
    _m_isca->config.p2p.txTimeout = P2P_TX_TIMEOUT;

    _m_isca->config.lrw.confirmed = false;
    _m_isca->config.lrw.posPort = LRW_POS_PORT;
    _m_isca->config.lrw.cmdPort = LRW_CMD_PORT;
    _m_isca->config.lrw.statusPort = LRW_STATUS_PORT;
    _m_isca->config.lrw.adr = false;



    memcpy(_m_isca->config.gsm.apn, GSM_APN, strlen(GSM_APN));
    memcpy(_m_isca->config.gsm.server, GSM_SERVER, strlen(GSM_SERVER));
    _m_isca->config.gsm.port = GSM_PORT;
}

void taskMemory(void *pvParameters)
{

    while(1)
    {

    }
}