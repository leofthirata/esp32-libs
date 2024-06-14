#include "main.h"
#include "sdkconfig.h"
#include "esp_mac.h"
#include "Button.h"
#include "LIS2DW12Sensor.h"
#include <Wire.h>
#include "adc.hpp"
#include "freertos/task.h"
#include "led.hpp"
#include "lora.hpp"
#include "esp_console.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "cmd_system.h"

uint8_t mac[6];
static const char* TAG = "main.cpp";

using namespace BiColorStatus;
 
Isca_t config;

TwoWire dev_i2c(0);  
LIS2DW12Sensor Acc(&dev_i2c, LIS2DW12_I2C_ADD_L);

void button_handler (button_queue_t param)
{
    static uint32_t cnt = 0;
    uint8_t type;

    switch(param.buttonState)
    {
        case BTN_PRESSED:
        ESP_LOGI("button_handler", "botão pressionado");
        // cnt++;
        // type = cnt%10;
        // switch(type)
        // {
        //     case 0:
        //     BiColorStatus::turnOn();
        //     ESP_LOGI(TAG, "turnOn");
        //     break;

        //     case 1:
        //     BiColorStatus::batCharged();
        //     ESP_LOGI(TAG, "batCharged");
        //     break;

        //     case 2:
        //     BiColorStatus::batCharging();
        //     ESP_LOGI(TAG, "batCharging");
        //     break;
            
        //     case 3:
        //     BiColorStatus::batError();
        //     ESP_LOGI(TAG, "batError");
        //     break;

        //     case 4:
        //     BiColorStatus::working();
        //     ESP_LOGI(TAG, "working");
        //     break;

        //     case 5:
        //     BiColorStatus::transmitting();
        //     ESP_LOGI(TAG, "transmitting");
        //     break;

        //     case 6:
        //     BiColorStatus::receiving();
        //     ESP_LOGI(TAG, "receiving");
        //     break;

        //     case 7:
        //     BiColorStatus::sendPosition();
        //     ESP_LOGI(TAG, "sendPosition");
        //     break;

        //     case 8:
        //     BiColorStatus::storingPosition();
        //     ESP_LOGI(TAG, "storingPosition");
        //     break;

        //     case 9:
        //     BiColorStatus::simCardError();
        //     ESP_LOGI(TAG, "simCardError");
        //     break;
        // }
        queueP2P();
        break;

        case BTN_RELEASED:
        // ESP_LOGI("button_handler", "RELEASED");
        break;

        case BTN_HOLD:
        queueLRW();
        ESP_LOGI("button_handler", "HOLD");
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

    initialize_nvs();
    register_system_common();
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));

    Serial.begin(115200);

    dev_i2c.begin(PIN_NUM_SDA, PIN_NUM_SCL, 100000);
    Acc.begin();
    Acc.Enable_X();

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
    
    xTaskCreatePinnedToCore(adcTask, "adcTask", 4096, (void*) &config, 5, NULL, 0);
    xTaskCreatePinnedToCore(loraTask, "loraTask", 4096, (void*) &config, 5, NULL, 0);
    
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

int32_t accelerometer[3];
float temp = 0.0;

void loop()
{
    Acc.Get_X_Axes(accelerometer);
    ESP_LOGI(TAG, "acc: %ld | %ld | %ld", 
        accelerometer[0], accelerometer[1], accelerometer[2]);

    Acc.Get_Temperature(&temp);
    ESP_LOGW(TAG, "temp: %02.2f", temp);

    vTaskDelay(5000);
}