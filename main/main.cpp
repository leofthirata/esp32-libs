#include "main.h"

#include "esp_mac.h"
#include "Button.h"

#include "sensors.hpp"
#include "freertos/task.h"
#include "led.hpp"
#include "lora.hpp"
#include "esp_console.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "Storage/storage.hpp"
#include "cmd_system.h"

#include "stateMachine.hpp"
#include "gsm.h"
#include "esp_netif.h"
#include "esp_event.h"
#include <esp_sleep.h>
#include <rom/rtc.h>

#include "otp.hpp"
#include "esp_log.h"
#include "memory.h"

#include "mbedtls/base64.h"
#include "sys/time.h"

Storage::Storage *storage = nullptr;
uint8_t mac[6];
static const char* TAG = "main.cpp";

using namespace BiColorStatus;
 
static Isca_t _isca;
static OTPMemory_t readMemory;

static GSMTxRes_t gsmTxRes;
static TaskHandle_t xTaskToNotify = NULL;
#define BIT_GSM_TX_RES 0x200

void print_reset_reason(RESET_REASON reason)
{
  switch ( reason)
  {
    case 1 : Serial.println ("POWERON_RESET");break;          /**<1,  Vbat power on reset*/
    case 3 : Serial.println ("SW_RESET");break;               /**<3,  Software reset digital core*/
    case 4 : Serial.println ("OWDT_RESET");break;             /**<4,  Legacy watch dog reset digital core*/
    case 5 : Serial.println ("DEEPSLEEP_RESET");break;        /**<5,  Deep Sleep reset digital core*/
    case 6 : Serial.println ("SDIO_RESET");break;             /**<6,  Reset by SLC module, reset digital core*/
    case 7 : Serial.println ("TG0WDT_SYS_RESET");break;       /**<7,  Timer Group0 Watch dog reset digital core*/
    case 8 : Serial.println ("TG1WDT_SYS_RESET");break;       /**<8,  Timer Group1 Watch dog reset digital core*/
    case 9 : Serial.println ("RTCWDT_SYS_RESET");break;       /**<9,  RTC Watch dog Reset digital core*/
    case 10 : Serial.println ("INTRUSION_RESET");break;       /**<10, Instrusion tested to reset CPU*/
    case 11 : Serial.println ("TGWDT_CPU_RESET");break;       /**<11, Time Group reset CPU*/
    case 12 : Serial.println ("SW_CPU_RESET");break;          /**<12, Software reset CPU*/
    case 13 : Serial.println ("RTCWDT_CPU_RESET");break;      /**<13, RTC Watch dog Reset CPU*/
    case 14 : Serial.println ("EXT_CPU_RESET");break;         /**<14, for APP CPU, reseted by PRO CPU*/
    case 15 : Serial.println ("RTCWDT_BROWN_OUT_RESET");break;/**<15, Reset when the vdd voltage is not stable*/
    case 16 : Serial.println ("RTCWDT_RTC_RESET");break;      /**<16, RTC Watch dog reset digital core and rtc module*/
    default : Serial.println ("NO_MEAN");
  }
}

void _sendPosition()
{
    static uint16_t counter = 0;
    static GSMTxReq_t gsmTx;

    gsmTx.payload.header = 0xD7;
    uint64_t serial = 0;
    serial = (uint64_t) 109*100000000 + _isca.rom.loraId;
    gsmTx.payload.serialNumber[0] = (serial >> 32) & 0xFF;
    gsmTx.payload.serialNumber[1] = (serial >> 24) & 0xFF;
    gsmTx.payload.serialNumber[2] = (serial >> 16) & 0xFF;
    gsmTx.payload.serialNumber[3] = (serial >> 8) & 0xFF;
    gsmTx.payload.serialNumber[4] = (serial) & 0xFF;
    gsmTx.payload.fw[0] = 0xDD;
    gsmTx.payload.fw[1] = 0xEE;
    gsmTx.payload.hw = _isca.rom.hwVer;
    gsmTx.payload.protocol = 0xCC;
    gsmTx.payload.counter[0] = (counter >> 8) & 0xFF;
    gsmTx.payload.counter[1] = counter & 0xFF;
    _isca.status.GSMCount = counter;
    if(counter == 0xFFFF)
        counter = 0;
    else
        counter++;
    struct timeval current;
    gettimeofday(&current, NULL);
    gsmTx.payload.timestamp[0] = (current.tv_sec >> 24) & 0xFF;
    gsmTx.payload.timestamp[1] = (current.tv_sec >> 16) & 0xFF;
    gsmTx.payload.timestamp[2] = (current.tv_sec >> 8) & 0xFF;
    gsmTx.payload.timestamp[3] = (current.tv_sec) & 0xFF;
    
    
    gsmTx.payload.type = 0x00;
    gsmTx.payload.loraID[0] = (_isca.rom.loraId >> 16) & 0xff;
    gsmTx.payload.loraID[1] = (_isca.rom.loraId >> 8) & 0xff;
    gsmTx.payload.loraID[2] = (_isca.rom.loraId) & 0xff;
    memcpy(gsmTx.payload.imei, _isca.rom.imei, sizeof(_isca.rom.imei));
    gsmTx.payload.temp = _isca.status.temperatureCelsius;
    gsmTx.payload.batteryVoltage[0] = (_isca.status.batteryMiliVolts >> 8) & 0xff;
    gsmTx.payload.batteryVoltage[1] = (_isca.status.batteryMiliVolts) & 0xff;
    gsmTx.payload.flags.asBit.bt = _isca.status.flags.asBit.bleStatus;
    gsmTx.payload.flags.asBit.emergency = _isca.status.flags.asBit.emergency;
    gsmTx.payload.flags.asBit.input = _isca.status.flags.asBit.input;
    gsmTx.payload.flags.asBit.jammerGsm = _isca.status.flags.asBit.jammer;
    gsmTx.payload.flags.asBit.jammerLoRa = _isca.status.flags.asBit.jammer;
    gsmTx.payload.flags.asBit.lowBattery = _isca.status.flags.asBit.lowBattery;
    gsmTx.payload.flags.asBit.movement = _isca.status.flags.asBit.movement;
    gsmTx.payload.flags.asBit.output = _isca.status.flags.asBit.output;
    gsmTx.payload.flags.asBit.statusBattery = _isca.status.flags.asBit.statusBattery;
    gsmTx.payload.flags.asBit.stockMode = _isca.status.flags.asBit.stockMode;
    gsmTx.payload.lastRst = 0x00;
    memcpy(&gsmTx.config.apn, _isca.config.gsm.apn, strlen(_isca.config.gsm.apn));
    gsmTx.config.port = _isca.config.gsm.port;
    memcpy(&gsmTx.config.server, _isca.config.gsm.server, strlen(_isca.config.gsm.server));
    esp_event_post(APP_EVENT, APP_EVENT_GSM_TX_REQ, &gsmTx, sizeof(GSMTxReq_t), 0);
}

void button_handler (button_queue_t param)
{

    switch(param.buttonState)
    {
        case BTN_PRESSED:
            ESP_LOGD("BTN", "pressed");
        break;

        case BTN_RELEASED:

            if(param.buttonPrev == BTN_PRESSED)
            {
                BiColorStatus::sendPosition();
                // _sendPosition();
                esp_event_post(APP_EVENT, APP_EVENT_SEND_POS, NULL, NULL, 0);
            }
            else
            {
                BiColorStatus::batError();
            }
        break;

        case BTN_HOLD:        
            ESP_LOGI("BTN", "hold");
        break;
    }
    
}

// static void app_event_handler(void *arg, esp_event_base_t event_base,
//                                int32_t event_id, void *event_data)
// {
//     if(event_base == APP_EVENT && event_id == APP_EVENT_GSM_TX_RES)
//     {
//         GSMTxRes_t *gsmTxRes_p = (GSMTxRes_t*) event_data;
//         memcpy(&gsmTxRes, gsmTxRes_p, sizeof(GSMTxRes_t));
//         xTaskNotify(xTaskToNotify, BIT_GSM_TX_RES, eSetBits);
//     }
// }

void setup()
{
    storage_init();
    
    // esp_console_repl_t *repl = NULL;
    // esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    // register_system_common();
    // esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    // ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));
    Serial.begin(115200);
    
    Serial.print("CPU0 reset reason: ");
    print_reset_reason(rtc_get_reset_reason(0));

    Serial.print("CPU1 reset reason: ");
    print_reset_reason(rtc_get_reset_reason(1));

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
   
    // repl_config.prompt = PROMPT_STR ">";
    // repl_config.max_cmdline_length = 1024;

    uint8_t numTries = 0;
    uint8_t ret = 0;
    do
    {
        ret = otpInit(NULL);
        if(ret == 0)
        {
            ret = otpRead(&readMemory);
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

    } while (numTries++ < 4 && ret != 0);
    
    if(ret != 0)
    {
        ESP_LOGE(TAG, "error memory OneWire");
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

    memset(&_isca, 0, sizeof(_isca));
    _isca.rom.loraId = (readMemory.loraID[0]<<16) + (readMemory.loraID[1]<<8) + (readMemory.loraID[2]);
    memcpy(_isca.rom.deviceEUI, readMemory.devEUI, 8);
    memcpy(_isca.rom.appEUI, readMemory.appEUI, 8);
    _isca.rom.devAddr = (readMemory.devAddr[0]<<24) + (readMemory.devAddr[1]<<16) +
        (readMemory.devAddr[2]<<8) + readMemory.devAddr[3];
    memcpy(_isca.rom.nwkSKey, readMemory.nwSKey, 16);
    memcpy(_isca.rom.appSKey, readMemory.appSKey, 16);
    
    printf("loraID: %ld | devAddress: %08lX\r\n", _isca.rom.loraId,_isca.rom.devAddr);
    ESP_LOG_BUFFER_HEX("deviceEUI", _isca.rom.deviceEUI, sizeof(_isca.rom.deviceEUI));
    ESP_LOG_BUFFER_HEX("appEUI", _isca.rom.appEUI, sizeof(_isca.rom.appEUI));
    ESP_LOG_BUFFER_HEX("nwSKey", _isca.rom.nwkSKey, sizeof(_isca.rom.nwkSKey));
    ESP_LOG_BUFFER_HEX("appSKey", _isca.rom.appSKey, sizeof(_isca.rom.appSKey));

    _isca.config.p2p.sf = P2P_SPREADING_FACTOR;
    _isca.config.p2p.bw = P2P_BANDWIDTH;
    _isca.config.p2p.cr = 1;
    _isca.config.p2p.txFreq = P2P_POS_FREQ;
    _isca.config.p2p.txPower = P2P_TX_POWER;
    _isca.config.p2p.rxFreq = P2P_CMD_FREQ;
    _isca.config.p2p.rxDelay = 0;
    _isca.config.p2p.rxTimeout = P2P_RX_TIMEOUT;
    _isca.config.p2p.txTimeout = P2P_TX_TIMEOUT;

    _isca.config.lrw.confirmed = false;
    _isca.config.lrw.posPort = LRW_POS_PORT;
    _isca.config.lrw.cmdPort = LRW_CMD_PORT;
    _isca.config.lrw.statusPort = LRW_STATUS_PORT;
    _isca.config.lrw.adr = false;

    ESP_ERROR_CHECK(esp_efuse_mac_get_default(mac));
    ESP_ERROR_CHECK(esp_base_mac_addr_set(mac));
    ESP_LOGW(TAG, "MAC: " MACSTR " ", MAC2STR(mac));

    memcpy(&_isca.rom.bleMac, mac, sizeof(_isca.rom.bleMac));

    memcpy(&_isca.config.gsm.apn, GSM_APN, strlen(GSM_APN));
    memcpy(&_isca.config.gsm.server, GSM_SERVER, strlen(GSM_SERVER));
    _isca.config.gsm.port = GSM_PORT;

    button_init_t button = {
        .buttonEventHandler = button_handler,
        .pin_bit_mask = (1ULL<<PIN_NUM_SWITCH),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };

    buttonInit(&button);
    
    BiColorStatus::init();
    BiColorStatus::turnOn();

    // xTaskToNotify = xTaskGetCurrentTaskHandle();

    // esp_event_handler_instance_register(APP_EVENT, ESP_EVENT_ANY_ID, &app_event_handler, nullptr, nullptr);     
    
    // Low priority numbers denote low priority tasks. The idle task has priority zero (tskIDLE_PRIORITY). 
    xTaskCreatePinnedToCore(sensorsTask, "sensorsTask", 4096, (void*) &_isca, 5, NULL, 0);
    xTaskCreatePinnedToCore(loraTask, "loraTask", 4096, (void*) &_isca, 5, NULL, 0);
    // xTaskCreatePinnedToCore(stateTask, "stateTask", 4096, (void*) &_isca, 6, NULL, 0);
    xTaskCreate(gsmTask, "gsmTask", 8192, (void*) &_isca, uxTaskPriorityGet(NULL), NULL);
    // ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

uint32_t ulNotifiedValue = 0;

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(3000));
    // xTaskNotifyWait( pdFALSE, pdTRUE, &ulNotifiedValue, portMAX_DELAY);
    // unsigned char hexArray[ESP_MODEM_C_API_STR_MAX];  
    // size_t hexArrayLen= 0, encodedLen = 0;
    // mbedtls_base64_decode(hexArray, sizeof(hexArray), &hexArrayLen, (const unsigned char*)gsmTxRes.base64, gsmTxRes.size);
    
    // GSMTxPayload_t *element_p = (GSMTxPayload_t*)hexArray;
    // uint64_t serialNumber = 0L;
    // serialNumber += (((uint64_t) element_p->serialNumber[0]) << 32);
    // serialNumber += (((uint64_t) element_p->serialNumber[1]) << 24);
    // serialNumber += (((uint64_t) element_p->serialNumber[2]) << 16);
    // serialNumber += (((uint64_t) element_p->serialNumber[3]) << 8);
    // serialNumber += (((uint64_t) element_p->serialNumber[4]));

    // uint64_t imei = 0L;
    // imei += (((uint64_t) element_p->imei[0]) << 48);
    // imei += (((uint64_t) element_p->imei[1]) << 40);
    // imei += (((uint64_t) element_p->imei[2]) << 32);
    // imei += (((uint64_t) element_p->imei[3]) << 24);
    // imei += (((uint64_t) element_p->imei[4]) << 16);
    // imei += (((uint64_t) element_p->imei[5]) << 8);
    // imei += (((uint64_t) element_p->imei[6]));
    
    // uint16_t fw = (element_p->fw[0] << 8);
    // fw += (element_p->fw[1]);

    // uint16_t count = (element_p->counter[0] << 8);
    // count += element_p->counter[1];

    // uint32_t timestamp = (element_p->timestamp[0] << 24);
    // timestamp += (element_p->timestamp[1] << 16);
    // timestamp += (element_p->timestamp[2] << 8);
    // timestamp += (element_p->timestamp[3]);

    // uint32_t loraId = (element_p->loraID[0] << 16);
    // loraId += (element_p->loraID[1] << 8);
    // loraId += (element_p->loraID[2]);
    // uint16_t battery = (element_p->batteryVoltage[0] << 8);
    // battery += element_p->batteryVoltage[1];
    // uint16_t flags = (element_p->flags.asArray[0] << 8);
    // flags += (element_p->flags.asArray[1]);

    // if(gsmTxRes.lastErr != ESP_OK)
    // {
    //     ESP_LOGE(TAG, "GSM error [%d] %s at state: %s", gsmTxRes.lastErr,
    //                 esp_err_to_name(gsmTxRes.lastErr), printState(gsmTxRes.lastState));

    //     element_p->flags.asBit.online = 0;
    //     element_p->loraID[2] = (gsmTxRes.lastErr)>>8;
    //     element_p->loraID[1] = gsmTxRes.lastErr & 0xff;
    //     element_p->loraID[0] = gsmTxRes.lastState;

    //     element_p->crc = 0;
    //     uint8_t crc_calc = crc8_itu((uint8_t *)hexArray, hexArrayLen);
    //     element_p->crc = crc_calc;
    //     memset(gsmTxRes.base64, 0, sizeof(gsmTxRes.base64));
    //     mbedtls_base64_encode((unsigned char *)gsmTxRes.base64, sizeof(gsmTxRes.base64), &encodedLen, (unsigned char *)hexArray, hexArrayLen);
    //     uint32_t id;
    //     gsmTxRes.base64[gsmTxRes.size] = 0x00;
    //     esp_err_t err = storage_gsm_save_position(gsmTxRes.base64, &id);
    //     ESP_LOGW(TAG, "Store GSM_POS status [%d] %s -> [%ld] %s", err, esp_err_to_name(err), id, gsmTxRes.base64);
    // }
    // else
    // {
    //     ESP_LOGI(TAG, "GSM SENT ");
    // }
    // printf("[PARSE] header: %d | sn: %llu | imei: %llu | fw: %04X | hw: %d\r\n",
    //             element_p->header, serialNumber, imei, fw, element_p->hw);
    
    // printf("\t proto: %02X | counter: %d | timestamp: %ld | type: %d | loraId: %ld\r\n",
    //             element_p->protocol, count, timestamp, element_p->type, loraId);
    // printf("\t temp: %d | battery: %d | crc: %d | flags: %04X | reset: %d | #erbs: %d\r\n",
    //             element_p->temp, battery, element_p->crc, flags, element_p->lastRst, element_p->n_erbs);
                
    // state = SM_WAIT_FOR_EVENT;
}