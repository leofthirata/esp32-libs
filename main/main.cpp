#include "main.h"
#include "sdkconfig.h"
#include "esp_mac.h"
#include "Button.h"

#include "sensors.hpp"
#include "freertos/task.h"
#include "led.hpp"
#include "lora.hpp"
#include "esp_console.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "cmd_system.h"

#include "stateMachine.hpp"

uint8_t mac[6];
static const char* TAG = "main.cpp";

using namespace BiColorStatus;
 
Isca_t config;

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
                queueP2P();
            }
            else
            {
                BiColorStatus::batError();
                queueLRW();
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
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    
    /* Prompt to be printed before each line.
     * This can be customized, made dynamic, etc.
     */
    repl_config.prompt = PROMPT_STR ">";
    repl_config.max_cmdline_length = 1024;

    config.loraId = LORA_ID; 
    
    initialize_nvs();
    register_system_common();
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));

    Serial.begin(115200);

    ESP_ERROR_CHECK(esp_efuse_mac_get_default(mac));
    ESP_ERROR_CHECK(esp_base_mac_addr_set(mac));
    ESP_LOGW(TAG, "MAC: " MACSTR " ", MAC2STR(mac));

    button_init_t button = {
        .buttonEventHandler = button_handler,
        .pin_bit_mask = (1ULL<<PIN_NUM_SWITCH),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };

    buttonInit(&button);
    
    BiColorStatus::init();
    BiColorStatus::turnOn();
    
    xTaskCreatePinnedToCore(sensorsTask, "sensorsTask", 4096, (void*) &config, 5, NULL, 0);
    xTaskCreatePinnedToCore(loraTask, "loraTask", 4096, (void*) &config, 5, NULL, 0);
    xTaskCreatePinnedToCore(stateTask, "stateTask", 4096, (void*) &config, 5, NULL, 0);
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

void loop()
{

    vTaskDelay(5000);
}