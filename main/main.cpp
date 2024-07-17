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
#include "gsm.hpp"
#include "esp_netif.h"
#include "esp_event.h"
#include <esp_sleep.h>
#include <rom/rtc.h>

#include "otp.hpp"
#include "esp_log.h"

Storage::Storage *storage = nullptr;
uint8_t mac[6];
static const char* TAG = "main.cpp";

using namespace BiColorStatus;
 
Isca_t config;
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
    static uint32_t cnt = 0;
    uint8_t type;

    switch(param.buttonState)
    {
        case BTN_PRESSED:
            ESP_LOGD("BTN", "pressed");
        break;

        case BTN_RELEASED:

            if(param.buttonPrev == BTN_PRESSED)
            {
                BiColorStatus::sendPosition();
                //queueP2P();
               // esp_event_post(APP_EVENT, APP_EVENT_REQ_P2P_SEND, NULL, 0, 0);
            }
            else
            {
                BiColorStatus::batError();
                //queueLRW();
                //esp_event_post(APP_EVENT, APP_EVENT_REQ_LRW_SEND, NULL, 0, 0);
            }
        break;

        case BTN_HOLD:        
            ESP_LOGI("BTN", "hold");
        break;
    }
    
}

static void initialize_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

void setup()
{
    // esp_console_repl_t *repl = NULL;
    // esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    initialize_nvs();
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

    storage = new Storage::Storage();
    ESP_ERROR_CHECK(storage->init());

    otpInit(NULL);
    otpRead(&readMemory);

    memset(&config, 0, sizeof(config));
    config.rom.loraId = (readMemory.loraID[0]<<16) + (readMemory.loraID[1]<<8) + (readMemory.loraID[2]);
    memcpy(config.rom.deviceEUI, readMemory.devEUI, 8);
    memcpy(config.rom.appEUI, readMemory.appEUI, 8);
    config.rom.devAddr = (readMemory.devAddr[0]<<24) + (readMemory.devAddr[1]<<16) +
        (readMemory.devAddr[2]<<8) + readMemory.devAddr[3];
    memcpy(config.rom.nwkSKey, readMemory.nwSKey, 16);
    memcpy(config.rom.appSKey, readMemory.appSKey, 16);
    
    printf("loraID: %ld | devAddress: %08lX\r\n", config.rom.loraId,config.rom.devAddr);
    ESP_LOG_BUFFER_HEX("deviceEUI", config.rom.deviceEUI, sizeof(config.rom.deviceEUI));
    ESP_LOG_BUFFER_HEX("appEUI", config.rom.appEUI, sizeof(config.rom.appEUI));
    ESP_LOG_BUFFER_HEX("nwSKey", config.rom.nwkSKey, sizeof(config.rom.nwkSKey));
    ESP_LOG_BUFFER_HEX("appSKey", config.rom.appSKey, sizeof(config.rom.appSKey));

    config.p2pConfig.sf = P2P_SPREADING_FACTOR;
    config.p2pConfig.bw = P2P_BANDWIDTH;
    config.p2pConfig.cr = 1;
    config.p2pConfig.txFreq = P2P_POS_FREQ;
    config.p2pConfig.txPower = P2P_TX_POWER;
    config.p2pConfig.rxFreq = P2P_CMD_FREQ;
    config.p2pConfig.rxDelay = 0;
    config.p2pConfig.rxTimeout = P2P_RX_TIMEOUT;
    config.p2pConfig.txTimeout = P2P_TX_TIMEOUT;

    config.lrwConfig.confirmed = false;
    config.lrwConfig.posPort = LRW_POS_PORT;
    config.lrwConfig.cmdPort = LRW_CMD_PORT;
    config.lrwConfig.statusPort = LRW_STATUS_PORT;
    config.lrwConfig.adr = false;

    ESP_ERROR_CHECK(esp_efuse_mac_get_default(mac));
    ESP_ERROR_CHECK(esp_base_mac_addr_set(mac));
    ESP_LOGW(TAG, "MAC: " MACSTR " ", MAC2STR(mac));

    memcpy(&config.rom.bleMac, mac, sizeof(config.rom.bleMac));

    memcpy(&config.gsmConfig.apn, GSM_APN, strlen(GSM_APN));
    memcpy(&config.gsmConfig.server, GSM_SERVER, strlen(GSM_SERVER));
    config.gsmConfig.port = GSM_PORT;

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
    xTaskCreatePinnedToCore(sensorsTask, "sensorsTask", 4096, (void*) &config, 5, NULL, 0);
    //xTaskCreatePinnedToCore(loraTask, "loraTask", 4096, (void*) &config, 5, NULL, 0);
    xTaskCreatePinnedToCore(stateTask, "stateTask", 4096, (void*) &config, 6, NULL, 0);
    xTaskCreate(gsmTask, "gsmTask", 8192, (void*) &config, uxTaskPriorityGet(NULL), NULL);
    // ESP_ERROR_CHECK(esp_console_start_repl(repl));
}


void loop()
{
    vTaskDelay(5000);
}