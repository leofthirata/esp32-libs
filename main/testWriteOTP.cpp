#include "sdkconfig.h"
#include "main.h"
#include "esp_log.h"
#include "AT24C02D/at24c02d.hpp"
#include "freertos/task.h"
#include "argtable3/argtable3.h"

#include "esp_console.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "Storage/storage.hpp"
#include "cmd_system.h"

#include "esp_timer.h"
#include "esp_mac.h"
#include "memory.hpp"
#include "testGetImei.h"
#include "esp_netif.h"
#include "esp_event.h"
uint8_t mac[6];
static const char *TAG = "test.cpp";
uint8_t dallas_crc8(const uint8_t *pdata, const uint32_t size);


#define AT24C02D_MEMORY_SIZE 256
#define CONFIG_SDA_GPIO GPIO_NUM_27
#define CONFIG_SCL_GPIO GPIO_NUM_26
#define CONFIG_WP_GPIO GPIO_NUM_12


uint8_t memoryRead[71];
uint8_t imei[7];
EEPROM::EEPROM *dev;

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

// https://stackoverflow.com/a/35452093
uint8_t *convertHexString2HexArray(char *string, size_t stringLen, uint8_t *arrayLenOrIndex)
{
    *arrayLenOrIndex = 0;

    if (string == NULL)
        return NULL;

    if ((stringLen % 2) != 0) // must be even
        return NULL;

    size_t dlength = stringLen / 2;

    uint8_t *data = (uint8_t *)malloc(dlength);
    memset(data, 0, dlength);

    size_t index = 0;
    while (index < stringLen)
    {
        char c = string[index];
        int value = 0;
        if (c >= '0' && c <= '9')
            value = (c - '0');
        else if (c >= 'A' && c <= 'F')
            value = (10 + (c - 'A'));
        else if (c >= 'a' && c <= 'f')
            value = (10 + (c - 'a'));
        else
        {
            *arrayLenOrIndex = index;
            free(data);
            return NULL;
        }

        data[(index / 2)] += value << (((index + 1) % 2) * 4);

        index++;
    }
    *arrayLenOrIndex = dlength;
    return data;
}

esp_err_t storage_init()
{

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    return err;
}

static struct
{
    struct arg_str *value;
    struct arg_end *end;
} set_args;

static struct
{
    struct arg_end *end;
} get_args;

static char memory[128] = {0};

static int otp_set(int argc, char **argv)
{
    uint8_t swp;
    int nerrors = arg_parse(argc, argv, (void **)&set_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, set_args.end, argv[0]);
        return 1;
    }

    const char *values = set_args.value->sval[0];

    printf("%s\r\n", values);
    memcpy(memory, values, strlen(values));

    uint8_t hexArraySize = 0;
    uint8_t *hexArray = convertHexString2HexArray((char *)values, strlen((char *)values), &hexArraySize);
    if (hexArray != NULL)
    {
        if (hexArraySize == 58)
        {
            dev->at24c02d_read_swp(&swp);
            ESP_LOGW(TAG, "swp %d", swp);
            dev->at24c02d_reset_swp();

            uint8_t otpCrc[88]; // 71 + 10 CRC + 7 padding
            memset(otpCrc, 0, 88);

            uint8_t otpNoCrc[71]; // 58 + 6 BLE + 7 IMEI

            memcpy(otpNoCrc, hexArray, hexArraySize);
            uint8_t *otp_p = (uint8_t *)otpNoCrc;

            for (int k = 0; k < sizeof(mac); k++)
                *(otp_p + hexArraySize + k) = mac[k];

            for (int k = 0; k < sizeof(imei); k++)
                *(otp_p + hexArraySize + sizeof(mac) + k) = imei[k];

            for (int i = 0; i < 10; i++)
            {
                memcpy(&otpCrc[i * 8], (otp_p + (i * 7)), 7);
            }
            // copy last byte
            memcpy(&otpCrc[80], otp_p + 70, 1);

            for (int i = 0; i < 11; i++)
            {
                ESP_LOG_BUFFER_HEX("otp", &otpCrc[8*i], 8);
            }

            for (int i = 0; i < 10; i++)
            {
                otpCrc[7 + i * 8] = dallas_crc8(&otpCrc[i * 8], 7);
                printf("otpCrc[%d] %02X | ", 7 + i * 8, otpCrc[7 + i * 8]);
            }

            // last byte to crc
            otpCrc[87] = dallas_crc8(&otpCrc[80], 7);
            printf("otpCrc[87] = %02X\r\n", otpCrc[87]);

            // ESP_LOG_BUFFER_HEX("Write2Mem", otpCrc, 88);
            for (int i = 0; i < 11; i++)
            {
                ESP_LOG_BUFFER_HEX("Write2Mem", &otpCrc[8*i], 8);
            }

            esp_err_t err = dev->at24c02d_write_byte(0x00, otpCrc, 88);
        }
    }

    free(hexArray);

    return 0;
}

static int otp_get(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&get_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, get_args.end, argv[0]);
        return 1;
    }

    uint8_t readFromMemory[88];
    memset(readFromMemory, 0, sizeof(readFromMemory));
    OTPMemory_t parseData;
    uint8_t *parseData_p = (uint8_t *)&parseData;
    uint8_t count = 0;

    for (int k = 0; k < 11; k++)
    {
        dev->at24c02d_read_byte(k * 8, (&readFromMemory[k * 8]), 8);
        uint8_t crc_cal = dallas_crc8(&(readFromMemory[k * 8]), 7);
        if (crc_cal != (readFromMemory[k * 8 + 7]))
        {
            printf("corrupted data %d crc_calc:%02X crc_mem: %02X, trying again\r\n", k,
                   crc_cal, readFromMemory[k * 8 + 7]);
            k--;
            count++;
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
        if (count >= 5)
            return -1;
    }

    for (int i = 0; i < 10; i++)
    {
        memcpy((parseData_p + (i * 7)), (&readFromMemory[i * 8]), 7);
    }
    memcpy((parseData_p + 70), &readFromMemory[80], 1);

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
    printf("        mac: %02X:%02X:%02X:%02X:%02X:%02X\r\n", parseData.bleMac[0], parseData.bleMac[1],
            parseData.bleMac[2],parseData.bleMac[3], parseData.bleMac[4], parseData.bleMac[5]);
    
    uint64_t _imei = 0L;
        _imei += (((uint64_t) parseData.imei[0])<<48);
        _imei += (((uint64_t) parseData.imei[1])<<40);
        _imei += (((uint64_t) parseData.imei[2])<<32);
        _imei += (((uint64_t) parseData.imei[3])<<24);
        _imei += (((uint64_t) parseData.imei[4])<<16);
        _imei += (((uint64_t) parseData.imei[5])<<8);
        _imei += (((uint64_t) parseData.imei[6]));

    printf("        imei: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X = %lld\r\n", parseData.imei[0], parseData.imei[1],
            parseData.imei[2], parseData.imei[3], parseData.imei[4], parseData.imei[5], parseData.imei[6], _imei);
    return 0;
}

static int ble_get(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&get_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, get_args.end, argv[0]);
        return 1;
    }
    esp_read_mac(mac, ESP_MAC_BT);
    ESP_LOGW(TAG, "BLE MAC: " MACSTR " ", MAC2STR(mac));
    return 0;
}

static int imei_get(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&get_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, get_args.end, argv[0]);
        return 1;
    }

    esp_err_t ret = getImei(imei);
    if(ret == ESP_OK)
    {
        char print[30];
        for(int k = 0; k < sizeof(imei); k++)
        {
            sprintf(print+k*3, "%02X ", imei[k]);   
        }
        print[21] = 0x00;
        uint64_t _imei = 0L;
        _imei += (((uint64_t) imei[0])<<48);
        _imei += (((uint64_t) imei[1])<<40);
        _imei += (((uint64_t) imei[2])<<32);
        _imei += (((uint64_t) imei[3])<<24);
        _imei += (((uint64_t) imei[4])<<16);
        _imei += (((uint64_t) imei[5])<<8);
        _imei += (((uint64_t) imei[6]));
        printf("imei: %lld | %s\r\n", _imei, print);
    }
    return 0;
}

void register_custom()
{
    set_args.value = arg_str1("v", "value", "<value>", "value to be stored");
    set_args.end = arg_end(2);

    get_args.end = arg_end(2);

    const esp_console_cmd_t set_otp = {
        .command = "otp_set",
        .help = "Set OTP memory version 0x02\n"
                "Example: \n"
                "otp_set -v memVer[1]hwVer[1]prefixSN[1]...\n" 
                " loraID[3]devAdd[4]devEUI[8]appEUI[8]...\n"
                " nwSKey[16]appSKey[16] (58bytes)\n",
        .hint = NULL,
        .func = &otp_set,
        .argtable = &set_args};

    ESP_ERROR_CHECK(esp_console_cmd_register(&set_otp));
    const esp_console_cmd_t get_otp = {
        .command = "otp_get",
        .help = "Read form OTP \n",
        .hint = NULL,
        .func = &otp_get,
        .argtable = &get_args};
    ESP_ERROR_CHECK(esp_console_cmd_register(&get_otp));
    
    const esp_console_cmd_t get_ble = {
        .command = "ble_get",
        .help = "Get BLE MAC \n",
        .hint = NULL,
        .func = &ble_get,
        .argtable = &get_args};
    ESP_ERROR_CHECK(esp_console_cmd_register(&get_ble));
        const esp_console_cmd_t get_imei = {
        .command = "imei_get",
        .help = "Get IMEI \n",
        .hint = NULL,
        .func = &imei_get,
        .argtable = &get_args};
    ESP_ERROR_CHECK(esp_console_cmd_register(&get_imei));
}

void setup(void)
{
    storage_init();

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    register_system_common();
    register_custom();
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));

    i2c_port_t port = I2C_NUM_0;
    int sda = CONFIG_SDA_GPIO;
    int scl = CONFIG_SCL_GPIO;
    int wp = CONFIG_WP_GPIO;

    dev = new EEPROM::EEPROM();
    ESP_ERROR_CHECK(dev->init(port, sda, scl, wp, true));


    esp_err_t err = ESP_OK;
    uint8_t myBuff[16];
    uint8_t swp;
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_efuse_mac_get_default(mac));
    ESP_ERROR_CHECK(esp_base_mac_addr_set(mac));

    esp_err_t ret = getImei(imei);
    if(ret == ESP_OK)
    {
        char print[30];
        for(int k = 0; k < sizeof(imei); k++)
        {
            sprintf(print+k*3, "%02X ", imei[k]);   
        }
        print[21] = 0x00;
        uint64_t _imei = 0L;
        _imei += (((uint64_t) imei[0])<<48);
        _imei += (((uint64_t) imei[1])<<40);
        _imei += (((uint64_t) imei[2])<<32);
        _imei += (((uint64_t) imei[3])<<24);
        _imei += (((uint64_t) imei[4])<<16);
        _imei += (((uint64_t) imei[5])<<8);
        _imei += (((uint64_t) imei[6]));
        printf("imei: %lld | %s\r\n", _imei, print);
    }
    
    esp_read_mac(mac, ESP_MAC_BT);
    ESP_LOGW(TAG, "BLE MAC: " MACSTR " ", MAC2STR(mac));

    dev->at24c02d_read_uid(myBuff);
    ESP_LOG_BUFFER_HEX("AT24C02D UID", myBuff, sizeof(myBuff));


    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(3000));
}