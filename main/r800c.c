#include "esp_log.h"
#include <esp_check.h>
#include "esp_netif.h"
#include "esp_netif_defaults.h"
#include "esp_netif_ppp.h"
#include "esp_modem_api.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#define GPIO_PIN_SEL(pin) (1ULL << pin)
#define TXD_PIN ((gpio_num_t)GPIO_NUM_14)
#define RXD_PIN ((gpio_num_t)GPIO_NUM_34)
#define GPIO_OUTPUT_PWRKEY ((gpio_num_t)GPIO_NUM_16)
#define GPIO_OUTPUT_POWER_ON ((gpio_num_t)GPIO_NUM_13)
#define GPIO_OUTPUT_RST ((gpio_num_t)GPIO_NUM_NC)

static const char *R800C_TAG = "r800c.c";

void power_on_modem()
{
    ESP_LOGI(R800C_TAG, "Power on the modem");
    gpio_set_level(GPIO_OUTPUT_POWER_ON, true);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(GPIO_OUTPUT_PWRKEY, 1);
    vTaskDelay(pdMS_TO_TICKS(2000));
    gpio_set_level(GPIO_OUTPUT_PWRKEY, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
}

void power_off_modem(esp_modem_dce_t *dce)
{
    char _response[512] = {};
    ESP_LOGI(R800C_TAG, "\t\t AT+CPOWD=1");
    esp_err_t ret = esp_modem_at_raw(dce, "AT+CPOWD=1\r", _response, "NORMAL POWER DOWN", "ERROR", 1500);
    ESP_LOGI(R800C_TAG, "Power off the modem | [%d] %s | %s", ret, esp_err_to_name(ret), _response);

    gpio_set_level(GPIO_OUTPUT_POWER_ON, false);
}

void config_pwrkey_gpio(void)
{
    gpio_config_t io_conf = {}; // zero-initialize the config structure.

    io_conf.intr_type = GPIO_INTR_DISABLE;                                                        // disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT;                                                              // set as output mode
    io_conf.pin_bit_mask = GPIO_PIN_SEL(GPIO_OUTPUT_POWER_ON) | GPIO_PIN_SEL(GPIO_OUTPUT_PWRKEY); // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;                                                 // disable pull-down mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;                                                     // disable pull-up mode

    gpio_config(&io_conf); // configure GPIO with the given settings

    gpio_set_level(GPIO_OUTPUT_POWER_ON, false);
    gpio_set_level(GPIO_OUTPUT_PWRKEY, false);
}

esp_err_t getImei(uint8_t *imei)
{
    esp_err_t err = ESP_OK;
    char response[512] = {};
    uint8_t count = 0;
    ESP_LOGI(R800C_TAG, "modem task starting");
    // uint8_t *imei = (uint8_t*) pvParams; 
    /* Configure and create the UART DTE */
    esp_modem_dte_config_t dte_config = ESP_MODEM_DTE_DEFAULT_CONFIG();
    /* setup UART specific configuration based on kconfig options */
    dte_config.uart_config.tx_io_num = TXD_PIN;
    dte_config.uart_config.rx_io_num = RXD_PIN;
    dte_config.uart_config.rts_io_num = -1;
    dte_config.uart_config.cts_io_num = -1;
    esp_modem_dce_config_t dce_config = ESP_MODEM_DCE_DEFAULT_CONFIG("simplepm.algar.br");
    esp_netif_config_t netif_ppp_config = ESP_NETIF_DEFAULT_PPP();
    esp_netif_t *esp_netif = esp_netif_new(&netif_ppp_config);
    assert(esp_netif);

    ESP_LOGI(R800C_TAG, "Initializing esp_modem for the R800C module...");
    esp_modem_dce_t *dce = esp_modem_new_dev(ESP_MODEM_DCE_SIM800, &dte_config, &dce_config, esp_netif);
    
    config_pwrkey_gpio();

    power_on_modem();

    count = 0;
    char imeiString[20];
    do
    {
        esp_err_t _ret = esp_modem_get_imei(dce, imeiString);
        if (_ret == ESP_OK)
        {
            ESP_LOGI(R800C_TAG, "Modem imei: %s", imeiString);
            uint64_t _imei = strtoll(imeiString, NULL, 10);
            *imei = (_imei >> (6 * 8)) & 0xFF;
            *(imei + 1) = (_imei >> (5 * 8)) & 0xFF;
            *(imei + 2) = (_imei >> (4 * 8)) & 0xFF;
            *(imei + 3) = (_imei >> (3 * 8)) & 0xFF;
            *(imei + 4) = (_imei >> (2 * 8)) & 0xFF;
            *(imei + 5) = (_imei >> (1 * 8)) & 0xFF;
            *(imei + 6) = (_imei) & 0xFF;
            break;
        }
        else
        {
            ESP_LOGE(R800C_TAG, "Error get imei: [%d] %s | Trying again! in 3s", _ret, esp_err_to_name(_ret));
            vTaskDelay(pdMS_TO_TICKS(3000));
            count++;
        }
    } while (count < 4);

    if (count >= 4)
    {
        ESP_LOGE(R800C_TAG, "Error getting IMEI - Going to sleep");
        err = ESP_FAIL;
    }
    else
    {
        ESP_LOGI(R800C_TAG, "\t\t ATE0");
        esp_modem_at_raw(dce, "ATE0\r", response, "OK", "", 1000);
        ESP_LOGI(R800C_TAG, "ATE0 response: %s", response);

        ESP_LOGI(R800C_TAG, "\t\t AT+CLTS=1");
        esp_modem_at_raw(dce, "AT+CLTS=1\r", response, "OK", "ERROR", 1000);
        ESP_LOGI(R800C_TAG, "AT+CLTS=1 response: %s", response);

        ESP_LOGI(R800C_TAG, "\t\t AT&W");
        esp_modem_at_raw(dce, "AT&W\r", response, "OK", "ERROR", 1000);
        ESP_LOGI(R800C_TAG, "AT&W response: %s", response);
        err = ESP_OK;
    }

    power_off_modem(dce);
    esp_netif_destroy(esp_netif);
    esp_modem_destroy(dce);
    // vTaskDelete(NULL);
    return err;
}