#include "main.h"
#include "esp_log.h"
#include <esp_check.h>
#include "esp_netif.h"
#include "esp_netif_ppp.h"
#include "driver/gpio.h"
#include "esp_modem_api.h"
#include "string.h"
#include "sys/time.h"
#include "stateMachine.hpp"
#include "gsm.h"
#include "mbedtls/base64.h"
#include "storage_nvs.h"
#include "esp_timer.h"
#include "utils.h"
static const char *TAG = "gsmTask";

#define GPIO_PIN_SEL(pin) (1ULL << pin)
#define TXD_PIN ((gpio_num_t)GPIO_NUM_14)
#define RXD_PIN ((gpio_num_t)GPIO_NUM_34)
#define GPIO_OUTPUT_PWRKEY ((gpio_num_t)GPIO_NUM_16)
#define GPIO_OUTPUT_POWER_ON ((gpio_num_t)GPIO_NUM_13)
#define GPIO_OUTPUT_RST ((gpio_num_t)GPIO_NUM_NC)

static GSMTxReq_t gsmTxReq;
static GSMTxRes_t gsmTxRes;
static GSMRx_t gsmRx;

static Isca_t *m_config;

static esp_timer_handle_t netlightTimer;
static QueueHandle_t xQueueR800CNetlight;

typedef enum
{
    EVENT_SEND_MESSAGE,
    EVENT_SEND_SAVED_MESSAGES,
    EVENT_MESSAGE_RECEIVED,
    EVENT_RECONFIGURE,
} event_id_t;

typedef struct
{
    event_id_t id;
    void *data;
} event_t;

typedef struct
{
    char imei[20];
    char mdm_model[30];
    char mdm_version[30];
    char date_time[30];
    int rssi;
    uint8_t csq;
} task_data_t;

QueueHandle_t _internal_queue;
static const char* printR800CNetlightStatus(NetlightStatus_t status);
static void power_on_modem()
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

static void power_off_modem(esp_modem_dce_t *dce)
{
    char _response[512] = {};
    R800CNetlight_t qElementNetlight;
    ESP_LOGI(TAG, "\t\t AT+CPOWD=1");
    esp_err_t ret = esp_modem_at_raw(dce, "AT+CPOWD=1\r", _response, "NORMAL POWER DOWN", "ERROR", 1500);
    ESP_LOGI(TAG, "Power off the modem | [%d] %s | %s", ret, esp_err_to_name(ret), _response);
    int64_t start = esp_timer_get_time();

    while ((esp_timer_get_time() - start) < 10000000)
    {
        if(xQueueReceive(xQueueR800CNetlight, &qElementNetlight, pdMS_TO_TICKS(3000)))
        {
            ESP_LOGI(TAG, "Got netlight status: [%d] %s | positive %lu ms | negative %lu ms",
                    qElementNetlight.status, printR800CNetlightStatus(qElementNetlight.status),
                    qElementNetlight.positivePulseWidth, qElementNetlight.negativePulseWidth);
            if(qElementNetlight.status == OFF)
                break;
        }
    }
    
    gpio_set_level(GPIO_OUTPUT_POWER_ON, false);
}

static void config_pwrkey_gpio(void)
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

char *generate_position_hex(mdm_lbs_cell_t *cells, GSMTxPayload_t *_data, size_t *size)
{
    char *hexArray = NULL;
    uint8_t numberErbs = 0;

    for (int i = 0; i < 7; i++)
    {
        if (cells[i].bcch != 0)
        {
            numberErbs++;
        }
    }
    uint16_t positionHexSize = sizeof(GSMTxPayload_t) + numberErbs * sizeof(GSMERBPacket_t);
    hexArray = calloc(positionHexSize, sizeof(uint8_t));
    *size = positionHexSize;
    _data->n_erbs = numberErbs;
    memcpy(hexArray, _data, sizeof(GSMTxPayload_t));

    for (int i = 0; i < 7; i++)
    {
        if (cells[i].bcch != 0)
        {
            uint16_t _index = sizeof(GSMTxPayload_t) + i * sizeof(GSMERBPacket_t);
            GSMERBPacket_t *erbPacket_p = (GSMERBPacket_t *)(hexArray + _index);
            erbPacket_p->id[0] = 0x00; //(cells[i].cellid>>(4*8)) * 0xff;
            erbPacket_p->id[1] = (cells[i].cellid >> (3 * 8)) * 0xff;
            erbPacket_p->id[2] = (cells[i].cellid >> (2 * 8)) * 0xff;
            erbPacket_p->id[3] = (cells[i].cellid >> (1 * 8)) * 0xff;
            erbPacket_p->id[4] = (cells[i].cellid) * 0xff;
            erbPacket_p->mcc[0] = (cells[i].mcc >> 8) & 0xFF;
            erbPacket_p->mcc[1] = (cells[i].mcc) & 0xFF;
            erbPacket_p->mnc[0] = (cells[i].mnc >> 8) & 0xFF;
            erbPacket_p->mnc[1] = (cells[i].mnc) & 0xFF;
            erbPacket_p->bsic = cells[i].bsic;
            erbPacket_p->ta = cells[i].ta;
            erbPacket_p->lac[0] = 0x00; //(cells[i].lac>>(4*8)) * 0xff;
            erbPacket_p->lac[1] = (cells[i].lac >> (3 * 8)) * 0xff;
            erbPacket_p->lac[2] = (cells[i].lac >> (2 * 8)) * 0xff;
            erbPacket_p->lac[3] = (cells[i].lac >> (1 * 8)) * 0xff;
            erbPacket_p->lac[4] = (cells[i].lac) * 0xff;
        }
    }

    uint8_t crc_calc = crc8_itu((uint8_t *)hexArray, positionHexSize);
    GSMTxPayload_t *element_p = (GSMTxPayload_t *)hexArray;
    element_p->crc = crc_calc;

    return hexArray;
}

static void app_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    if (event_base == APP_EVENT && event_id == APP_EVENT_GSM_TX_REQ)
    {
        GSMTxReq_t *element_p = (GSMTxReq_t *)event_data;
        memcpy(&gsmTxReq, element_p, sizeof(GSMTxReq_t));
        xQueueSend(_internal_queue, &gsmTxReq, 0);
    }
}

const char *printState(en_task_state state)
{
    switch (state)
    {
    case STATE_INIT:
        return "STATE_INIT";
    case STATE_DEINIT:
        return "STATE_DEINIT";
    case STATE_SYNC:
        return "STATE_SYNC";
    case STAT_CONFIG_PRMS:
        return "STAT_CONFIG_PRMS";
    case STATE_CHECK_SIGNAL:
        return "STATE_CHECK_SIGNAL";
    case STATE_CHECK_OPERATOR:
        return "STATE_CHECK_OPERATOR";
    case STATE_SCAN_NETWORKS:
        return "STATE_SCAN_NETWORKS";
    case STATE_UPDATE_CLOCK:
        return "STATE_UPDATE_CLOCK";
    case STATE_GET_IP:
        return "STATE_GET_IP";
    case STATE_GET_LBS_POSITION:
        return "STATE_GET_LBS_POSITION";
    case STATE_CHECK_IP:
        return "STATE_CHECK_IP";
    case STATE_SEND_PACKET:
        return "STATE_SEND_PACKET";
    case STATE_RECEIVE_PACKET:
        return "STATE_RECEIVE_PACKET";
    case STATE_SEND_PREV_PACKET:
        return "STATE_SEND_PREV_PACKET";
    case STATE_WAIT_EVENT:
        return "STATE_WAIT_EVENT";
    case STATE_OPEN_SOCKET:
        return "STATE_OPEN_SOCKET";
    case STATE_CLOSE_SOCKET:
        return "STATE_CLOSE_SOCKET";
    case STATE_SLEEP:
        return "STATE_SLEEP";
    case STATE_POWERON:
        return "STATE_POWERON";
    case STATE_REPORT_ERROR:
        return "STATE_REPORT_ERROR";
    }
    return "unknown state";
}

static const char* printR800CNetlightStatus(NetlightStatus_t status)
{
    switch(status)
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

static void IRAM_ATTR netlightISR()
{
    static unsigned long positivePulseStartTime = 0, negativePulseStartTime = 0;
    static R800CNetlight_t m_R800CNetlight = {OFF, OFF, 0, 0, 0};
    
    static unsigned long lastReport = 0;
    unsigned long now = micros();

    if (gpio_get_level((gpio_num_t)PIN_NUM_GSM_NETLIGHT) == HIGH) // If the change was a RISING edge
    {
        positivePulseStartTime = now; // Store the start time (in microseconds)
        if((now - m_R800CNetlight.lastNegativePulseStartTime) < NETLIGHT_OFF_TIMEOUT)
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

        if(esp_timer_is_active(netlightTimer))
        {
            esp_timer_stop(netlightTimer);
        }
        esp_timer_start_once(netlightTimer, NETLIGHT_OFF_TIMEOUT);
    }
    
    //64 - 10%
    if (m_R800CNetlight.positivePulseWidth > 58 ) 
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
        if(now - lastReport > NETLIGHT_REPORT_TIMEOUT && m_R800CNetlight.status != OFF)
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

void gsmTask(void *pvParameters)
{
    esp_err_t err = ESP_FAIL, err2Send = ESP_FAIL;
    en_task_state state, prev_state;
    task_data_t task_data = {};

    char command[128] = {};
    char response[512] = {};
    mdm_lbs_cell_t cells[7] = {};
    GSMTxReq_t gsmElement;

    m_config = (Isca_t *)pvParameters;
    ESP_LOGI(TAG, "modem task starting");

    /* Configure and create the UART DTE */
    esp_modem_dte_config_t dte_config = ESP_MODEM_DTE_DEFAULT_CONFIG();
    /* setup UART specific configuration based on kconfig options */
    dte_config.uart_config.tx_io_num = TXD_PIN;
    dte_config.uart_config.rx_io_num = RXD_PIN;
    dte_config.uart_config.rts_io_num = -1;
    dte_config.uart_config.cts_io_num = -1;
    esp_modem_dce_config_t dce_config = ESP_MODEM_DCE_DEFAULT_CONFIG(m_config->config.gsm.apn);
    esp_netif_config_t netif_ppp_config = ESP_NETIF_DEFAULT_PPP();
    esp_netif_t *esp_netif = esp_netif_new(&netif_ppp_config);
    assert(esp_netif);

    ESP_LOGI(TAG, "Initializing esp_modem for the SIM800 module...");
    esp_modem_dce_t *dce = esp_modem_new_dev(ESP_MODEM_DCE_SIM800, &dte_config, &dce_config, esp_netif);

    
    R800CNetlight_t qElementNetlight;
    
    const esp_timer_create_args_t netlightTimerArgs = {
        .callback = &netlightTimerCallback,
        .name = "netlightTimer"};

    ESP_ERROR_CHECK(esp_timer_create(&netlightTimerArgs, &netlightTimer));
    config_pwrkey_gpio();

    xQueueR800CNetlight = xQueueCreate(NETLIGHT_QUEUE_SIZE, sizeof(R800CNetlight_t));
    if(xQueueR800CNetlight == NULL)
    {
        printf("\t\t\t ERROR!\r\n");
        while(1);
    }

    pinMode(PIN_NUM_GSM_NETLIGHT, INPUT);                                                 // Set the input pin
    attachInterrupt(digitalPinToInterrupt(PIN_NUM_GSM_NETLIGHT), netlightISR, CHANGE); // Run the calcPulsewidth function on signal CHANGE

    if(digitalRead(PIN_NUM_GSM_NETLIGHT) == 0)
    {
        esp_timer_start_once(netlightTimer, NETLIGHT_OFF_TIMEOUT);
    }

    state = STATE_INIT;
    prev_state = state;
    const uint8_t zeroImei[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (memcmp(m_config->rom.imei, zeroImei, sizeof(m_config->rom.imei)) == 0)
    {
        ESP_LOGW(TAG, "Imei not set");

        uint8_t count = 0;
        do
        {
            power_on_modem();
            if(xQueueReceive(xQueueR800CNetlight, &qElementNetlight, pdMS_TO_TICKS(10000)))
            {
                ESP_LOGI(TAG, "Got netlight status: %s | positive %lu ms | negative %lu ms",
                        printR800CNetlightStatus(qElementNetlight.status),
                        qElementNetlight.positivePulseWidth, qElementNetlight.negativePulseWidth);
                
                if(qElementNetlight.status == NOT_REGISTERED) // modem turned on
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

            do
            {
                esp_err_t _ret = esp_modem_get_imei(dce, task_data.imei);
                if (_ret == ESP_OK)
                {
                    ESP_LOGI(TAG, "Modem imei: %s", task_data.imei);
                    uint64_t _imei = strtoll(task_data.imei, NULL, 10);
                    m_config->rom.imei[0] = (_imei >> (6 * 8)) & 0xFF;
                    m_config->rom.imei[1] = (_imei >> (5 * 8)) & 0xFF;
                    m_config->rom.imei[2] = (_imei >> (4 * 8)) & 0xFF;
                    m_config->rom.imei[3] = (_imei >> (3 * 8)) & 0xFF;
                    m_config->rom.imei[4] = (_imei >> (2 * 8)) & 0xFF;
                    m_config->rom.imei[5] = (_imei >> (1 * 8)) & 0xFF;
                    m_config->rom.imei[6] = (_imei) & 0xFF;
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
            }
        
        }
        else
        {
            ESP_LOGE(TAG, "Cannot power on - Going to sleep");
        }
    }

    for (;;)
    {
        ESP_LOGI(TAG, "%s", printState(state));
        
        switch (state)
        {

            case STATE_GET_LBS_POSITION:
            {
                esp_modem_at(dce, "AT+SAPBR=3,1,\"Contype\",\"GPRS\"", response, 1000);
                // Set bearer parameter
                esp_modem_at(dce, "AT+SAPBR=3,1,\"APN\",\"simplepm.algar.br\"", response, 1000);
                esp_modem_at(dce, "AT+SAPBR=3,1,\"USER\",\"algar\"", response, 1000);
                esp_modem_at(dce, "AT+SAPBR=3,1,\"PWD\",\"algar\"", response, 1000);
                // Active bearer context
                esp_modem_at(dce, "AT+SAPBR=1,1", response, 95000);
                // Read bearer parameter
                esp_modem_at(dce, "AT+SAPBR=2,1", response, 1000);
                // Get customer ID
                esp_modem_at(dce, "AT+CLBSCFG=0,1", response, 1000);
                // Get Times have used positioning command
                esp_modem_at(dce, "AT+CLBSCFG=0,2", response, 1000);
                // Get LBS server’s address
                esp_modem_at(dce, "AT+CLBSCFG=0,3", response, 1000);
                // Set LBS server’s address
                esp_modem_at(dce, "AT+CLBSCFG=1,3,\"lbs-simcom.com:3002\"", response, 1000);
                //// Get current longitude , latitude , Precision and date time
                esp_modem_at(dce, "AT+CLBS=1,1", response, 60000);
                // Deactivate bearer context
                esp_modem_at(dce, "AT+SAPBR=0,1", response, 15000);

                vTaskDelay(5000);
                prev_state = state;
                state = STATE_SCAN_NETWORKS;
            }
            break;

            case STATE_INIT:
            {
                //@ TODO - Usar funções para checar alocação do recurso
                _internal_queue = xQueueCreate(10, sizeof(GSMTxReq_t));
                esp_event_handler_instance_register(APP_EVENT, APP_EVENT_GSM_TX_REQ, &app_event_handler, NULL, NULL);
                prev_state = state;
                state = STATE_SLEEP;
            }
            break;

            case STATE_DEINIT:
            {
                vQueueDelete(_internal_queue);
                prev_state = state;
                // TODO - Desalocar recursos que foram criados nesse estado.
            }
            break;

            case STATE_SLEEP:
            {
                memset(cells, 0, sizeof(cells));
                power_off_modem(dce);
                prev_state = state;
                state = STATE_WAIT_EVENT;
            }
            break;

            case STATE_WAIT_EVENT:
            {
                prev_state = state;
                if (xQueueReceive(_internal_queue, &gsmElement, portMAX_DELAY))
                {
                    state = STATE_POWERON;
                }
            }
            break;

            case STATE_POWERON:
            {
                static uint8_t count = 0;
                prev_state = state;

                power_on_modem();

                if(xQueueReceive(xQueueR800CNetlight, &qElementNetlight, pdMS_TO_TICKS(10000)))
                {
                    ESP_LOGI(TAG, "Got netlight status: %s | positive %lu ms | negative %lu ms",
                            printR800CNetlightStatus(qElementNetlight.status),
                            qElementNetlight.positivePulseWidth, qElementNetlight.negativePulseWidth);
                    
                    if(qElementNetlight.status == NOT_REGISTERED) // modem turned on
                    {
                        count = 0;
                        state = STATE_SYNC;
                    }

                }
                else
                {
                    if (count++ < 4)
                    {
                        ESP_LOGW(TAG, "PowerOn: Timeout netlightstatus #%d | Trying again! in 3s", count);
                        vTaskDelay(pdMS_TO_TICKS(3000));
                        state = STATE_POWERON;
                    }
                    else
                    {
                        count = 0;
                        ESP_LOGE(TAG, "Error PowerOn | Going to sleep");
                        err2Send = ESP_FAIL;
                        state = STATE_REPORT_ERROR;
                    }
                }

            }
            break;

            case STATE_SYNC:
            {
                prev_state = state;
                
                // ESP_LOGI(TAG, "\t\t ATE0");
                // esp_modem_at(dce, "ATE0", response, 1000);

                // ESP_LOGI(TAG, "\t\t AT+CLTS=1");
                // esp_modem_at_raw(dce, "AT+CLTS=1\r", response, "OK", "ERROR", 1000);
                
                if (strlen(task_data.imei) == 0)
                {
                    ESP_LOGI(TAG, "\t\t AT+CGSN");
                    if (esp_modem_get_imei(dce, task_data.imei) == ESP_OK)
                    {
                        ESP_LOGI(TAG, "Modem imei: %s", task_data.imei);
                        uint64_t _imei = strtoll(task_data.imei, NULL, 10);
                        m_config->rom.imei[0] = (_imei >> (6 * 8)) & 0xFF;
                        m_config->rom.imei[1] = (_imei >> (5 * 8)) & 0xFF;
                        m_config->rom.imei[2] = (_imei >> (4 * 8)) & 0xFF;
                        m_config->rom.imei[3] = (_imei >> (3 * 8)) & 0xFF;
                        m_config->rom.imei[4] = (_imei >> (2 * 8)) & 0xFF;
                        m_config->rom.imei[5] = (_imei >> (1 * 8)) & 0xFF;
                        m_config->rom.imei[6] = (_imei) & 0xFF;
                    }
                }

                vTaskDelay(1000);
                // ESP_LOGI(TAG, "\t\t AT+COPS=0");
                // esp_modem_at(dce, "AT+COPS=0", response, 5000);
                state = STATE_CHECK_SIGNAL;
            }
            break;

            case STATE_CHECK_SIGNAL:
            {
                prev_state = state;
                static uint8_t count = 0;
                int *prms = (int *)calloc(sizeof(int), 2);
                ESP_LOGI(TAG, "\t\t AT+CSQ");
                esp_err_t ret = esp_modem_get_signal_quality(dce, &prms[0], &prms[1]);
                if (ret == ESP_OK)
                {
                    if (prms[0] > 5)
                    {
                        ESP_LOGI(TAG, "\t\t AT+CENG=4,1");
                        esp_modem_at_raw(dce, "AT+CENG=4,1\r", response, "OK", "ERROR", 500);
                        state = STATE_CHECK_OPERATOR;
                        count = 0;
                    }
                    else
                    {
                        ESP_LOGW(TAG, "\t\t CSQ: %d <= 5", prms[0]);
                        if(count++ >= 10)
                        {
                            err2Send = ret;
                            state = STATE_REPORT_ERROR;
                            count = 0;
                        }
                    }
                    
                    vTaskDelay(pdMS_TO_TICKS(3000));
                }
                else if (ret == ESP_ERR_TIMEOUT)
                {
                    err2Send = ret;
                    state = STATE_REPORT_ERROR;
                    count = 0;
                }
                else
                    vTaskDelay(3000);

                free(prms);
            }
            break;

            case STATE_CHECK_OPERATOR:
            {
                static uint8_t count = 0; 
                prev_state = state;
                char *name = (char *)calloc(64, 1);
                int *val = calloc(sizeof(int), 1);
                ESP_LOGI(TAG, "\t\t AT+COPS?");
                esp_err_t ret = esp_modem_get_operator_name(dce, name, val);
                if (ret == ESP_OK)
                {
                    ESP_LOGI(TAG, "OK. %s %i", name, *val);
                    if (strlen(name) > 0)
                    {
                        count = 0;
                        state = STATE_SCAN_NETWORKS;
                        vTaskDelay(pdMS_TO_TICKS(3000));
                    }
                }
                else if (ret == ESP_ERR_TIMEOUT)
                {
                    count = 0;
                    err2Send = ret;
                    state = STATE_REPORT_ERROR;
                }
                else
                {
                    if(count++ > 10)
                    {
                        count = 0;
                        err2Send = ESP_FAIL;
                        state = STATE_REPORT_ERROR;
                    }
                    else
                    {
                        vTaskDelay(3000);
                    }
                }

                free(name);
                free(val);
            }
            break;

            case STATE_SCAN_NETWORKS:
            {
                const char delimiter[] = "\r\n+CENG: ";
                char *token;
                int cell_i = -1;

                memset(cells, 0, sizeof(cells));
                memset(response, 0, sizeof(response));

                ESP_LOGI(TAG, "\t\t AT+CENG?");
                if (esp_modem_at_raw(dce, "AT+CENG?\r", response, "OK", "ERROR", 5000) == ESP_OK)
                {
                    ESP_LOGI(TAG, "AT+CENG? response: %s", response);

                    /* get the first token */
                    token = strtok(response, delimiter);

                    /* walk through other tokens */
                    while (token != NULL && cell_i < 7)
                    {
                        if (cell_i >= 0)
                        {
                            // ESP_LOGI(TAG, " %s", token);

                            // first cell we are connected on that
                            if (cell_i == 0)
                            {
                                int res = sscanf(token, "%d,\"%d,%d,%d,%d,%d,%d,%X,%d,%d,%X,%d,%d,%d,%d,",
                                                &cells[cell_i].cell, &cells[cell_i].bcch, &cells[cell_i].rxl,
                                                &cells[cell_i].rxq, &cells[cell_i].mcc, &cells[cell_i].mnc,
                                                &cells[cell_i].bsic, &cells[cell_i].cellid, &cells[cell_i].rla,
                                                &cells[cell_i].txp, &cells[cell_i].lac, &cells[cell_i].ta,
                                                &cells[cell_i].dbm, &cells[cell_i].c1, &cells[cell_i].c2);

                                // ESP_LOGI(TAG, "SSCANF PRMS PARSED %d", res);
                                // ESP_LOGI(TAG, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                                //         cells[cell_i].cell, cells[cell_i].bcch, cells[cell_i].rxl,
                                //         cells[cell_i].rxq, cells[cell_i].mcc, cells[cell_i].mnc,
                                //         cells[cell_i].bsic, cells[cell_i].cellid, cells[cell_i].rla,
                                //         cells[cell_i].txp, cells[cell_i].lac, cells[cell_i].ta,
                                //         cells[cell_i].dbm, cells[cell_i].c1, cells[cell_i].c2);
                            }
                            else
                            {
                                int res = sscanf(token, "%d,\"%d,%d,%d,%X,%d,%d,%X,%d,%d\"",
                                                &cells[cell_i].cell, &cells[cell_i].bcch, &cells[cell_i].rxl,
                                                &cells[cell_i].bsic, &cells[cell_i].cellid, &cells[cell_i].mcc,
                                                &cells[cell_i].mnc, &cells[cell_i].lac, &cells[cell_i].c1,
                                                &cells[cell_i].c2);

                                // ESP_LOGI(TAG, "SSCANF PRMS PARSED %d", res);
                                // ESP_LOGI(TAG, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                                //         cells[cell_i].cell, cells[cell_i].bcch, cells[cell_i].rxl,
                                //         cells[cell_i].bsic, cells[cell_i].cellid, cells[cell_i].mcc,
                                //         cells[cell_i].mnc, cells[cell_i].lac, cells[cell_i].c1,
                                //         cells[cell_i].c2);
                            }
                        }

                        token = strtok(NULL, delimiter);

                        cell_i++;
                    }
                }
                prev_state = state;
                state = STATE_UPDATE_CLOCK;
            }
            break;
            
            case STATE_UPDATE_CLOCK:
            {
                int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0, tz = 0;
                char sign = 0x00;
                struct tm tm;

                ESP_LOGI(TAG, "\t\t AT+CCLK?");
                esp_modem_at(dce, "AT+CCLK?", response, 1000);

                //{+CCLK: "24/07/08,09:31:48-12"}
                sscanf(response, "+CCLK: \"%s\"", task_data.date_time);

                sscanf(task_data.date_time, "%d/%d/%d,%d:%d:%d%c%d",
                    &year, &month, &day, &hour, &minute, &second, &sign, &tz);

                tm.tm_year = year + 2000 - 1900;
                tm.tm_mon = month - 1;
                tm.tm_mday = day;
                tm.tm_hour = hour;
                tm.tm_min = minute;
                tm.tm_sec = second;
                time_t t = mktime(&tm);

                struct timeval tv;
                tv.tv_sec = t;
                
                printf("Setting time: %s\r\n", asctime(&tm));
                settimeofday(&tv, NULL);

                struct timeval current;
                gettimeofday(&current, NULL);
                printf("Second : %llu \nMicrosecond : %06lu\r\n",
                    current.tv_sec, current.tv_usec);
                char buf[255];
                strftime(buf, sizeof(buf), "%d %b %Y %H:%M:%S", &tm);
                ESP_LOGW(TAG, "%s", buf);
                
                prev_state = state;
                state = STATE_OPEN_SOCKET;
            }
            break;

            case STATE_OPEN_SOCKET:
            {
                prev_state = state;
                esp_err_t ret = ESP_OK;
                ESP_LOGI(TAG, "\t\t AT+CIPRXGET=1");
                ret = esp_modem_at_raw(dce, "AT+CIPRXGET=1\r", response, "OK", "ERROR", 1000);
                ESP_LOGI(TAG, "AT+CIPRXGET=1 ret [%d]%s | response: %s", ret, esp_err_to_name(ret), response);

                ESP_LOGI(TAG, "\t\t AT+CSTT");
                sprintf(command, "AT+CSTT=\"%s\"\r", gsmElement.config.apn);
                ret = esp_modem_at_raw(dce, command, response, "OK", "ERROR", 1000);
                ESP_LOGI(TAG, "AT+CSTT ret [%d]%s | response: %s", ret, esp_err_to_name(ret), response);
                
                //+PDP: DEACT => 2B5044503A204445414354
                const char deactString[]={0x2b, 0x50, 0x44, 0x50, 0x3a, 0x20, 0x44, 0x45, 0x41, 0x43, 0x54, 0x00};
                char *_token;
                static uint8_t count = 0;

                // bring ip up
                ESP_LOGI(TAG, "\t\t AT+CIICR");
                ret = esp_modem_at_raw(dce, "AT+CIICR\r", response, "OK", "ERROR",30000);
                ESP_LOGI(TAG, "AT+CIICR ret [%d]%s | response: %s", ret, esp_err_to_name(ret), response);
                
                //get cases of +PDP: DEACT
                _token = strstr(response, deactString);
                if(_token!=NULL) //we have pdp deact
                {
                    // If "+PDP: DEACT" URC is reported which means the GPRS is released by the network, 
                    // then user still needs to execute "AT+CIPSHUT" command to make PDP context come back to original state
                    ESP_LOGW(TAG, "\t\tGOT <+PDP: DEACT> | token: %s", _token);
                    esp_err_t err = ESP_OK;
                    uint8_t numTries = 4;
                    do
                    {
                        ESP_LOGI(TAG, "\t\t AT+CIPSHUT");
                        err = esp_modem_at_raw(dce, "AT+CIPSHUT\r", response, "SHUT OK", "ERROR", 5000);
                        ESP_LOGI(TAG, "\t\t[AT+CIPSHUT] response: %s | err [%d] %s ",response, err,esp_err_to_name(err));
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    } while (numTries-- > 0 && err != ESP_OK);
                    
                    if (numTries == 0)
                    {
                        ESP_LOGE(TAG, "GOT <+PDP: DEACT> and failed to CIPSHUT.");
                        err2Send = err;
                        state = STATE_REPORT_ERROR;
                    }
                    else
                    {
                        ESP_LOGI(TAG, "\t\t[AT+CIPSHUT] response: %s | err [%d] %s | Trying to open socket again in 3s",response, err,esp_err_to_name(err));
                        vTaskDelay(pdMS_TO_TICKS(3000));
                        if(count++ < 4)
                            state = STATE_OPEN_SOCKET;
                        else
                        {
                            count = 0;
                            ESP_LOGE(TAG, "GOT <+PDP: DEACT> and failed to connect.");
                            err2Send = ESP_FAIL;
                            state = STATE_REPORT_ERROR;
                        }

                    }
                }
                else
                {
                    count = 0;

                    // show ip
                    ESP_LOGI(TAG, "\t\t AT+CIFSR");
                    ret = esp_modem_at_raw(dce, "AT+CIFSR\r", response, "" , "ERROR", 1000);
                    ESP_LOGI(TAG, "AT+CIFSR ret [%d]%s | response: %s", ret, esp_err_to_name(ret), response);

                    //get cases of +PDP: DEACT
                    _token = strstr(response, deactString);
                    if(_token!=NULL) //we have pdp deact
                    {
                        // If "+PDP: DEACT" URC is reported which means the GPRS is released by the network, 
                        // then user still needs to execute "AT+CIPSHUT" command to make PDP context come back to original state
                        ESP_LOGW(TAG, "\t\tGOT <+PDP: DEACT> | token: %s", _token);
                        esp_err_t err = ESP_OK;
                        uint8_t numTries = 4;
                        do
                        {
                            ESP_LOGI(TAG, "\t\t AT+CIPSHUT");
                            err = esp_modem_at_raw(dce, "AT+CIPSHUT\r", response, "SHUT OK", "ERROR", 5000);
                            ESP_LOGI(TAG, "\t\t[AT+CIPSHUT] response: %s | err [%d] %s ",response, err,esp_err_to_name(err));
                            vTaskDelay(pdMS_TO_TICKS(1000));
                        } while (numTries-- > 0 && err != ESP_OK);
                        
                        if (numTries == 0)
                        {
                            ESP_LOGE(TAG, "GOT <+PDP: DEACT> and failed to CIPSHUT.");
                            err2Send = err;
                            state = STATE_REPORT_ERROR;
                        }
                        else
                        {
                            ESP_LOGI(TAG, "\t\t[AT+CIPSHUT] response: %s | err [%d] %s | Trying again in 3s",response, err,esp_err_to_name(err));
                            vTaskDelay(pdMS_TO_TICKS(3000));
                            if(count++ < 4)
                                state = STATE_OPEN_SOCKET;
                            else
                            {
                                count = 0;
                                ESP_LOGE(TAG, "GOT <+PDP: DEACT> and failed to connect.");
                                err2Send = ESP_FAIL;
                                state = STATE_REPORT_ERROR;
                            }
                        }
                    }
                    else
                    {
                        // start socket with server
                        count = 0;
                        ESP_LOGI(TAG, "\t\t AT+CIPSTART");
                        sprintf(command, "AT+CIPSTART=\"TCP\",\"%s\",\"%d\"\r\n", gsmElement.config.server, gsmElement.config.port);
                        ret = esp_modem_at_raw(dce, command, response, "CONNECT OK", "CONNECT FAIL", 5000);
                        ESP_LOGI(TAG, "AT+CIPSTART ret [%d]%s | response: %s", ret, esp_err_to_name(ret), response);
                        if(ret == ESP_OK)
                            state = STATE_SEND_PACKET;
                        else
                        {
                            ESP_LOGI(TAG, "\t\t AT+CIPSHUT");
                            esp_modem_at(dce, "AT+CIPSHUT", response, 5000);
                            err2Send = ret;
                            state = STATE_REPORT_ERROR;
                        }
                    }
                }

            }
            break;

            case STATE_SEND_PACKET:
            {
                uint64_t _imei = strtoll(task_data.imei, NULL, 10);
                gsmElement.payload.imei[0] = (_imei >> (6 * 8)) & 0xFF;
                gsmElement.payload.imei[1] = (_imei >> (5 * 8)) & 0xFF;
                gsmElement.payload.imei[2] = (_imei >> (4 * 8)) & 0xFF;
                gsmElement.payload.imei[3] = (_imei >> (3 * 8)) & 0xFF;
                gsmElement.payload.imei[4] = (_imei >> (2 * 8)) & 0xFF;
                gsmElement.payload.imei[5] = (_imei >> (1 * 8)) & 0xFF;
                gsmElement.payload.imei[6] = (_imei) & 0xFF;
                
                size_t size;
                gsmElement.payload.flags.asBit.online = 1;
                char *hex = generate_position_hex(cells, &gsmElement.payload, &size);

                char string[ESP_MODEM_C_API_STR_MAX];
                for (uint8_t i = 0; i < size; i++)
                    sprintf(&string[i * 2], "%02X", *(hex + i));

                string[size * 2] = 0x00;

                ESP_LOGI(TAG, "To: %s:%d", gsmElement.config.server, gsmElement.config.port);
                ESP_LOGI(TAG, "buffer HEX %d bytes: %s", size, string);
                char encode64[ESP_MODEM_C_API_STR_MAX] = {0};
                size_t len_encoded = 0;

                mbedtls_base64_encode((unsigned char *)encode64, sizeof(encode64), &len_encoded, (unsigned char *)hex, size);
                free(hex);
                ESP_LOGI(TAG, "buffer base64 %d bytes: %s", len_encoded, encode64);

                memcpy(gsmTxRes.base64, encode64, len_encoded);
                gsmTxRes.size = len_encoded;
                gsmTxRes.lastState = state;
                prev_state = state;
                
                ESP_LOGI(TAG, "\t\t AT+CIPSEND=%d", len_encoded);
                sprintf(command, "AT+CIPSEND=%d\r", len_encoded);
                gsmTxRes.lastErr = esp_modem_at_raw(dce, command, response, ">", "ERROR", 500);
                ESP_LOGI(TAG, "\t\tAT+CIPSEND=%d response: %s | err [%d] %s ",len_encoded,response, gsmTxRes.lastErr,esp_err_to_name(gsmTxRes.lastErr));
                
                // send request was accepted
                if (gsmTxRes.lastErr == ESP_OK)
                {
                    ESP_LOGI(TAG, "\t\t data base64");
                    gsmTxRes.lastErr = esp_modem_at_raw(dce, encode64, response, "SEND OK", "SEND FAIL", 90 * 1000);
                    ESP_LOGI(TAG, "\t\tSEND response: %s | err [%d] %s ",response, gsmTxRes.lastErr,esp_err_to_name(gsmTxRes.lastErr));
                    if (gsmTxRes.lastErr == ESP_OK)
                    {
                        ESP_LOGI(TAG, "Success to send packet");
                        state = STATE_RECEIVE_PACKET;
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Fail to send packet");
                        state = STATE_CLOSE_SOCKET;
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "Send Request [AT+CIPSEND] Fail [%d] %s", err, esp_err_to_name(err));
                    state = STATE_CLOSE_SOCKET;
                }
            
                esp_event_post(APP_EVENT, APP_EVENT_GSM_TX_RES, &gsmTxRes, sizeof(GSMTxRes_t), 0);

            }
            break;
            
            case STATE_RECEIVE_PACKET:
            {
                prev_state = state;
                int64_t now = esp_timer_get_time();
                esp_err_t err = ESP_FAIL;

                while(esp_timer_get_time() - now < GSM_RX_WINDOW)
                {
                    ESP_LOGI(TAG, "\t\t AT+CIPRXGET=2,1460");
                    err = esp_modem_at_raw(dce, "AT+CIPRXGET=2,1460\r\n", response, "OK", "ERROR", 5000);
                    if ( err == ESP_OK )
                    {
                        if(strlen(response))
                        {
                            ESP_LOGD(TAG, "AT+CIPRXGET response: %s", response);
                            const char delimiterinit[] = "\r\n+CIPRXGET: ";
                            const char delimiter[]="\r\n";
                            char *token;
                            uint8_t mode, size, dummy;
                            token = strtok(response, delimiterinit);
                            while(token != NULL)
                            {
                                // printf("\t\t[token %d] %s\r\n", strlen(token), token);
                                sscanf(token, "%hhu", &mode);
                                // printf("\t\t mode: %d\r\n", mode);
                                if(mode == 2)
                                {
                                    sscanf(token, "%hhu,%hhu,%hhu", &mode, &size, &dummy);
                                    if(size > 0)
                                    {
                                        token = strtok(NULL, delimiter);
                                        if(token!=NULL)
                                        {
                                            // printf("\t\t[token %d] %s\r\n", strlen(token), token);
                                            if(strlen(token) == size)
                                                break;
                                        }
                                    }
                                }
                                token = strtok(NULL, delimiterinit);
                            }
                            if(token!= NULL)
                            {
                                printf("\t\t[GSM_RX]: %s\r\n\r\n", token);
                                memcpy(&gsmRx.payload, token, size);
                                gsmRx.size = size;

                                if (strcmp(gsmRx.payload, gsmTxRes.base64) == 0)
                                {
                                    esp_event_post(APP_EVENT, APP_EVENT_JIGA, &gsmRx, sizeof(GSMRx_t), 0);
                                }
                                esp_event_post(APP_EVENT, APP_EVENT_GSM_RX, &gsmRx, sizeof(GSMRx_t), 0);
                            }
                        }
                        
                    }
                    else  if (err == ESP_ERR_TIMEOUT)
                    {
                        break;
                    }

                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
                if (err == ESP_ERR_TIMEOUT)
                {
                    state = STATE_REPORT_ERROR;
                    err2Send = err;
                }
                else
                    state = STATE_SEND_PREV_PACKET;
            }
            break;

            case STATE_SEND_PREV_PACKET:
            {
                prev_state = state;
                uint32_t prevPacketQty = 0;
                gsmTxRes.lastErr = ESP_FAIL;
                do
                {
                    storage_gsm_get_id(&prevPacketQty);
                    ESP_LOGI(TAG, "[PREV_PACKETS] #%ld", prevPacketQty);
                    if(prevPacketQty)
                    {
                        char base64[ESP_MODEM_C_API_STR_MAX];
                        memset(base64, 0, sizeof(base64));
                        size_t sizePrevMem = 0;
                        esp_err_t ret = storage_gsm_get_last_position(base64, &sizePrevMem);
                        ESP_LOGI(TAG, "get last position ret [%d] %s | size %d bytes | strlen %d bytes", ret, esp_err_to_name(ret), sizePrevMem, strlen(base64));
                        if(ret== ESP_OK)
                        {
                            size_t size = (sizePrevMem < strlen(base64))?sizePrevMem:strlen(base64);
                            memcpy(&gsmTxRes.base64, base64, size);
                            gsmTxRes.size = size;
                            gsmTxRes.lastState = state;
                            ESP_LOGI(TAG, "\t\t AT+CIPSEND=%d", size);

                            sprintf(command, "AT+CIPSEND=%d\r", size);
                            gsmTxRes.lastErr = esp_modem_at_raw(dce, command, response, ">", "ERROR", 500);
                            ESP_LOGI(TAG, "\t\tAT+CIPSEND=%d response: %s | err [%d] %s ",size,response, gsmTxRes.lastErr,esp_err_to_name(gsmTxRes.lastErr));
                            // send request was accepted
                            if (gsmTxRes.lastErr == ESP_OK)
                            {
                                ESP_LOGI(TAG, "\t\t base64 data");
                                gsmTxRes.lastErr = esp_modem_at_raw(dce, base64, response, "SEND OK", "SEND FAIL", 30 * 1000);
                                ESP_LOGI(TAG, "\t\tSEND response: %s | err [%d] %s ",response, gsmTxRes.lastErr,esp_err_to_name(gsmTxRes.lastErr));
                                if (gsmTxRes.lastErr == ESP_OK)
                                {
                                    ESP_LOGI(TAG, "Success to send prev packet");
                                }
                                else
                                {
                                    ESP_LOGE(TAG, "Fail to send prev packet");
                                }
                            }
                            else
                            {
                                ESP_LOGE(TAG, "Send Request [AT+CIPSEND] prev packet Fail [%d] %s", err, esp_err_to_name(err));
                            }
                            esp_event_post(APP_EVENT, APP_EVENT_GSM_TX_RES, &gsmTxRes, sizeof(GSMTxRes_t), 0);
                        }
                    }

                } while((prevPacketQty) && gsmTxRes.lastErr == ESP_OK);
                state = STATE_CLOSE_SOCKET;
            }
            break;

            case STATE_CLOSE_SOCKET:
            {
                // fecha socket após envio.
                ESP_LOGI(TAG, "\t\t AT+CIPCLOSE");
                sprintf(command, "AT+CIPCLOSE");
                esp_modem_at(dce, command, response, 5000);
                ESP_LOGI(TAG, "\t\t AT+CIPSHUT");
                sprintf(command, "AT+CIPSHUT");
                esp_modem_at(dce, command, response, 5000);
                prev_state = state;
                state = STATE_SLEEP;
            }
            break;
            
            //Tx fail before tcp connection
            case STATE_REPORT_ERROR:
            {
                size_t size;
                char *hex = generate_position_hex(cells, &gsmElement.payload, &size);

                char string[ESP_MODEM_C_API_STR_MAX];
                for (size_t i = 0; i < size; i++)
                    sprintf(&string[i * 2], "%02X", *(hex + i));

                string[size * 2] = 0x00;

                ESP_LOGI(TAG, "buffer HEX %d bytes: %s", size, string);
                char encode64[ESP_MODEM_C_API_STR_MAX] = {0};
                size_t len_encoded = 0;

                mbedtls_base64_encode((unsigned char *)encode64, sizeof(encode64), &len_encoded, (unsigned char *)hex, size);
                free(hex);
                ESP_LOGI(TAG, "buffer base64 %d bytes: %s", len_encoded, encode64);

                memcpy(gsmTxRes.base64, encode64, len_encoded);
                gsmTxRes.size = len_encoded;
                gsmTxRes.lastErr = err2Send;
                gsmTxRes.lastState = prev_state;

                esp_event_post(APP_EVENT, APP_EVENT_GSM_TX_RES, &gsmTxRes, sizeof(GSMTxRes_t), 0);
                prev_state = state;
                state = STATE_SLEEP;
            }
            break;

            default:
            break;
        }
    }
}
