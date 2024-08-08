#include "main.h"
#include "gsm.h"
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

static esp_timer_handle_t netlightTimer;
static QueueHandle_t xQueueR800CNetlight;

static const char *TAG = "getImei.cpp";

const char *printR800CNetlightStatus(NetlightStatus_t status)
{
    switch (status)
    {
    case OFF:
        return "Powered off";
        break;

    case NOT_REGISTERED:
        return "Not registered to the network";
        break;

    case REGISTERED:
        return "Registered to the network";
        break;

    case GPRS_CONNECTED:
        return "GPRS communication is established";
        break;
    }
    return "ERROR";
}

void power_on_modem()
{
    R800CNetlight_t qElementNetlight;
    while (uxQueueSpacesAvailable(xQueueR800CNetlight) < NETLIGHT_QUEUE_SIZE)
    {
        xQueueReceive(xQueueR800CNetlight, &qElementNetlight, pdMS_TO_TICKS(10000));
        ESP_LOGW(TAG, "Taking netlight status from queue: %s | positive %lu ms | negative %lu ms",
                 printR800CNetlightStatus(qElementNetlight.status),
                 qElementNetlight.positivePulseWidth, qElementNetlight.negativePulseWidth);
    }

    ESP_LOGI(TAG, "Power on the modem");
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
    R800CNetlight_t qElementNetlight;
    ESP_LOGI(TAG, "\t\t AT+CPOWD=1");
    esp_err_t ret = esp_modem_at_raw(dce, "AT+CPOWD=1\r", _response, "NORMAL POWER DOWN", "ERROR", 1500);
    ESP_LOGI(TAG, "Power off the modem | [%d] %s | %s", ret, esp_err_to_name(ret), _response);
    int64_t start = esp_timer_get_time();

    while ((esp_timer_get_time() - start) < 10000000)
    {
        if (xQueueReceive(xQueueR800CNetlight, &qElementNetlight, pdMS_TO_TICKS(3000)))
        {
            ESP_LOGI(TAG, "Got netlight status: [%d] %s | positive %lu ms | negative %lu ms",
                     qElementNetlight.status, printR800CNetlightStatus(qElementNetlight.status),
                     qElementNetlight.positivePulseWidth, qElementNetlight.negativePulseWidth);
            if (qElementNetlight.status == OFF)
                break;
        }
    }

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

static void IRAM_ATTR netlightISR()
{
    static unsigned long positivePulseStartTime = 0, negativePulseStartTime = 0;
    static R800CNetlight_t m_R800CNetlight = {OFF, OFF, 0, 0, 0};

    static unsigned long lastReport = 0;
    unsigned long now = micros();

    if (gpio_get_level((gpio_num_t)PIN_NUM_GSM_NETLIGHT) == HIGH) // If the change was a RISING edge
    {
        positivePulseStartTime = now; // Store the start time (in microseconds)
        if ((now - m_R800CNetlight.lastNegativePulseStartTime) < NETLIGHT_OFF_TIMEOUT)
            m_R800CNetlight.negativePulseWidth = (now - negativePulseStartTime) / 1000; // pulse in ms
        else
        {
            m_R800CNetlight.negativePulseWidth = 0;
            m_R800CNetlight.status = OFF;
            m_R800CNetlight.prev = OFF;
        }
    }
    else // If the change was a FALLING edge
    {
        negativePulseStartTime = now;
        if ((now - m_R800CNetlight.lastNegativePulseStartTime) < NETLIGHT_OFF_TIMEOUT)
            m_R800CNetlight.positivePulseWidth = (now - positivePulseStartTime) / 1000; // pulse in ms
        else
        {
            m_R800CNetlight.positivePulseWidth = 0;
            m_R800CNetlight.status = OFF;
            m_R800CNetlight.prev = OFF;
        }

        m_R800CNetlight.lastNegativePulseStartTime = now;

        if (esp_timer_is_active(netlightTimer))
        {
            esp_timer_stop(netlightTimer);
        }
        esp_timer_start_once(netlightTimer, NETLIGHT_OFF_TIMEOUT);
    }

    // 64 - 10%
    if (m_R800CNetlight.positivePulseWidth > 58)
    {
        // 300ms +- 10%
        if (m_R800CNetlight.negativePulseWidth < 330 && m_R800CNetlight.negativePulseWidth > 270)
            m_R800CNetlight.status = GPRS_CONNECTED;

        // 800ms +- 10%
        else if (m_R800CNetlight.negativePulseWidth < 880 && m_R800CNetlight.negativePulseWidth > 720)
            m_R800CNetlight.status = NOT_REGISTERED;

        // 3000ms +- 10%
        else if (m_R800CNetlight.negativePulseWidth < 3300 && m_R800CNetlight.negativePulseWidth > 2700)
            m_R800CNetlight.status = REGISTERED;
    }
    else
    {
        m_R800CNetlight.status = OFF;
    }

    if (m_R800CNetlight.status != m_R800CNetlight.prev)
    {
        lastReport = now;
        xQueueSendFromISR(xQueueR800CNetlight, &m_R800CNetlight, NULL);
        m_R800CNetlight.prev = m_R800CNetlight.status;
    }
    else
    {
        if (now - lastReport > NETLIGHT_REPORT_TIMEOUT && m_R800CNetlight.status != OFF)
        {
            lastReport = now;
            xQueueSendFromISR(xQueueR800CNetlight, &m_R800CNetlight, NULL);
        }
    }
}

static void netlightTimerCallback(void *arg)
{
    R800CNetlight_t R800CNetlight = {OFF, OFF, 0, 0, 0};
    xQueueSend(xQueueR800CNetlight, &R800CNetlight, 0);
}

esp_err_t getImei(uint8_t *imei)
{
    esp_err_t err = ESP_OK;
    char response[512] = {};
    uint8_t count = 0;
    R800CNetlight_t qElementNetlight;
    ESP_LOGI(TAG, "modem task starting");
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

    ESP_LOGI(TAG, "Initializing esp_modem for the SIM800 module...");
    esp_modem_dce_t *dce = esp_modem_new_dev(ESP_MODEM_DCE_SIM800, &dte_config, &dce_config, esp_netif);

    const esp_timer_create_args_t netlightTimerArgs = {
        .callback = &netlightTimerCallback,
        .name = "netlightTimer"};

    ESP_ERROR_CHECK(esp_timer_create(&netlightTimerArgs, &netlightTimer));
    
    config_pwrkey_gpio();

    xQueueR800CNetlight = xQueueCreate(NETLIGHT_QUEUE_SIZE, sizeof(R800CNetlight_t));
    if (xQueueR800CNetlight == NULL)
    {
        printf("\t\t\t ERROR!\r\n");
        while (1)
            ;
    }

    pinMode(PIN_NUM_GSM_NETLIGHT, INPUT);                                              // Set the input pin
    attachInterrupt(digitalPinToInterrupt(PIN_NUM_GSM_NETLIGHT), netlightISR, CHANGE); // Run the calcPulsewidth function on signal CHANGE

    if (digitalRead(PIN_NUM_GSM_NETLIGHT) == 0)
    {
        esp_timer_start_once(netlightTimer, NETLIGHT_OFF_TIMEOUT);
    }

    do
    {
        power_on_modem();
        if (xQueueReceive(xQueueR800CNetlight, &qElementNetlight, pdMS_TO_TICKS(10000)))
        {
            ESP_LOGI(TAG, "Got netlight status: %s | positive %lu ms | negative %lu ms",
                        printR800CNetlightStatus(qElementNetlight.status),
                        qElementNetlight.positivePulseWidth, qElementNetlight.negativePulseWidth);

            if (qElementNetlight.status == NOT_REGISTERED) // modem turned on
                break;
        }
        else
        {
            ESP_LOGE(TAG, "Error PowerOn: Timeout netlightstatus | Trying again! in 3s");
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    } while (count++ < 4);

    if (count < 4)
    {
        count = 0;
        char imeiString[20];
        do
        {
            esp_err_t _ret = esp_modem_get_imei(dce, imeiString);
            if (_ret == ESP_OK)
            {
                ESP_LOGI(TAG, "Modem imei: %s", imeiString);
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
                ESP_LOGE(TAG, "Error get imei: [%d] %s | Trying again! in 3s", _ret, esp_err_to_name(_ret));
                vTaskDelay(pdMS_TO_TICKS(3000));
                count++;
            }
        } while (count < 4);

        if (count >= 4)
        {
            ESP_LOGE(TAG, "Error getting IMEI - Going to sleep");
            err = ESP_FAIL;
        }
        else
        {
            ESP_LOGI(TAG, "\t\t ATE0");
            esp_modem_at_raw(dce, "ATE0\r", response, "OK", "", 1000);
            ESP_LOGI(TAG, "ATE0 response: %s", response);

            ESP_LOGI(TAG, "\t\t AT+CLTS=1");
            esp_modem_at_raw(dce, "AT+CLTS=1\r", response, "OK", "ERROR", 1000);
            ESP_LOGI(TAG, "AT+CLTS=1 response: %s", response);

            ESP_LOGI(TAG, "\t\t AT&W");
            esp_modem_at_raw(dce, "AT&W\r", response, "OK", "ERROR", 1000);
            ESP_LOGI(TAG, "AT&W response: %s", response);
            err = ESP_OK;
        }
    }
    else
    {
        err = ESP_FAIL;
    }

    power_off_modem(dce);
    esp_netif_destroy(esp_netif);
    esp_modem_destroy(dce);
    esp_timer_stop(netlightTimer);
    esp_timer_delete(netlightTimer);
    vQueueDelete(xQueueR800CNetlight);
    // vTaskDelete(NULL);
    return err;
}