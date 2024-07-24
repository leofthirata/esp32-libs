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

Storage::Storage *storage = nullptr;
uint8_t mac[6];
static const char* TAG = "main.cpp";

using namespace BiColorStatus;
 
static Isca_t _isca;
static OTPMemory_t readMemory;

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


    otpInit(NULL);
    otpRead(&readMemory);

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

    // Low priority numbers denote low priority tasks. The idle task has priority zero (tskIDLE_PRIORITY). 
    xTaskCreatePinnedToCore(sensorsTask, "sensorsTask", 4096, (void*) &_isca, 5, NULL, 0);
    //xTaskCreatePinnedToCore(loraTask, "loraTask", 4096, (void*) &_isca, 5, NULL, 0);
    //xTaskCreatePinnedToCore(stateTask, "stateTask", 4096, (void*) &_isca, 6, NULL, 0);
    xTaskCreate(gsmTask, "gsmTask", 8192, (void*) &_isca, uxTaskPriorityGet(NULL), NULL);
    // ESP_ERROR_CHECK(esp_console_start_repl(repl));
}


void loop()
{
    vTaskDelay(3000);
}