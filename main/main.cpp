#include "main.h"
#include "esp_mac.h"
#include "Button.h"
#include "LedBicolor.hpp"
#include "led.hpp"
#include "LIS2DW12Sensor.h"
#include <Wire.h>

uint8_t mac[6];
static const char* TAG = "main.cpp";

using namespace BiColor;
LedBicolor *led; 

TwoWire dev_i2c(0);  
LIS2DW12Sensor Acc(&dev_i2c, LIS2DW12_I2C_ADD_L);

void button_handler (button_queue_t param)
{
    static uint32_t cnt = 0;
    uint8_t type;

    switch(param.buttonState)
    {
        case BTN_PRESSED:
        // ESP_LOGI("button_handler", "botão pressionado");
        cnt++;
        type = cnt%10;
        switch(type)
        {
            case 0:
            BiColor::init();
            ESP_LOGI(TAG, "init");
            break;

            case 1:
            BiColor::batCharged();
            ESP_LOGI(TAG, "batCharged");
            break;

            case 2:
            BiColor::batCharging();
            ESP_LOGI(TAG, "batCharging");
            break;
            
            case 3:
            BiColor::batError();
            ESP_LOGI(TAG, "batError");
            break;

            case 4:
            BiColor::working();
            ESP_LOGI(TAG, "working");
            break;

            case 5:
            BiColor::transmitting();
            ESP_LOGI(TAG, "transmitting");
            break;

            case 6:
            BiColor::receiving();
            ESP_LOGI(TAG, "receiving");
            break;

            case 7:
            BiColor::sendPosition();
            ESP_LOGI(TAG, "sendPosition");
            break;

            case 8:
            BiColor::storingPosition();
            ESP_LOGI(TAG, "storingPosition");
            break;

            case 9:
            BiColor::simCardError();
            ESP_LOGI(TAG, "simCardError");
            break;
        }
        break;

        case BTN_RELEASED:
        // ESP_LOGI("button_handler", "RELEASED");
        break;

        case BTN_HOLD:
        // ESP_LOGI("button_handler", "HOLD");
        break;
    }
    
}

void setup()
{
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
    
    led = new LedBicolor();
    led->begin(PIN_LED_STATUS_RED, PIN_LED_STATUS_GREEN);
    BiColor::init();
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