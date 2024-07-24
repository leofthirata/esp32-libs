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


Storage::Storage *storage = nullptr;
uint8_t mac[6];
static const char* TAG = "main.cpp";


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


void setup()
{
    storage_init();
    
    Serial.begin(115200);
    
    Serial.print("CPU0 reset reason: ");
    print_reset_reason(rtc_get_reset_reason(0));

    Serial.print("CPU1 reset reason: ");
    print_reset_reason(rtc_get_reset_reason(1));

    uint8_t hexArray[]={0xd7, 0x02, 0x8a, 0x6f, 0x89, 0x20, 0x03, 0x10, 0xf5, 0x27, 0x8a, 0xbb, 0xf9, 0xdd, 0xee, 0x00, 0xcc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3d, 0x00, 0xbe, 0xbc, 0x20, 0x20, 0x10, 0x62, 0xed, 0x06, 0x00, 0x00, 0x00};
    char encode64[ESP_MODEM_C_API_STR_MAX];
    size_t len_encoded = 0, size;
    size = sizeof(hexArray);
    mbedtls_base64_encode((unsigned char *)encode64, sizeof(encode64), &len_encoded, (unsigned char *)hexArray, size);
    uint32_t id;
    //encode64[len_encoded] = 0x00;
    esp_err_t err = storage_gsm_save_position(encode64, &id);
    ESP_LOGW(TAG, "Store GSM_POS status [%d] %s -> [%ld] %s", err, esp_err_to_name(err), id, encode64);
                
    uint32_t prevPacketQty = 0;
    esp_err_t ret = ESP_FAIL;
    do
    {
        storage_gsm_get_id(&prevPacketQty);
        ESP_LOGI(TAG, "[PREV_PACKETS] #%ld", prevPacketQty);
        if(prevPacketQty)
        {
            char base64[ESP_MODEM_C_API_STR_MAX];
            size_t size = 0;
            ret = storage_gsm_get_last_position(base64, &size);
            ESP_LOGI(TAG, "get last position ret [%d] %s", ret, esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(2000));

    } while((prevPacketQty) && ret == ESP_OK);
}


void loop()
{
    vTaskDelay(5000);
}