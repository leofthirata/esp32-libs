#include "main.h"
#include "esp_mac.h"

uint8_t mac[6];
static const char* TAG = "main.cpp";

void setup()
{
    Serial.begin(115200);
            
    ESP_ERROR_CHECK(esp_efuse_mac_get_default(mac));
    ESP_ERROR_CHECK(esp_base_mac_addr_set(mac));
    ESP_LOGW(TAG, "MAC: " MACSTR " ", MAC2STR(mac));
}

void loop()
{
    vTaskDelay(1000);
}