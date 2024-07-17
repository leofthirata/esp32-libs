#include "esp_log.h"
#include <esp_check.h>
#include "esp_netif.h"
#include "esp_netif_ppp.h"
#include "driver/gpio.h"
#include "esp_modem_api.h"
#include "string.h"
#include "sys/time.h"
#include "stateMachine.hpp"
#include "gsm.hpp"
#include "mbedtls/base64.h"

static const char *TAG = "gsmTask";

#define GPIO_PIN_SEL(pin) (1ULL << pin)
#define TXD_PIN ((gpio_num_t)GPIO_NUM_14)
#define RXD_PIN ((gpio_num_t)GPIO_NUM_34)
#define GPIO_OUTPUT_PWRKEY ((gpio_num_t)GPIO_NUM_16)
#define GPIO_OUTPUT_POWER_ON ((gpio_num_t)GPIO_NUM_13)
#define GPIO_OUTPUT_RST ((gpio_num_t)GPIO_NUM_NC)

static GSMTxReq_t gsmTx;
static GSMTxRes_t txDone;
static Isca_t *m_config;

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

typedef struct
{
    int cell;   // 0 The serving cell, 1-6 The index of the neighboring cell
    int bcch;   // ARFCN(Absolute radio frequency channel number) of BCCH carrier, in decimal format
    int rxl;    // Receive level, in decimal format
    int rxq;    // Receive quality, in decimal format
    int mcc;    // Mobile country code, in decimal format
    int mnc;    // Mobile network code, in decimal format
    int bsic;   // Base station identity code, in decimal format
    int cellid; // Cell id, in hexadecimal format
    int rla;    // Receive level access minimum, in decimal format
    int txp;    // Transmit power maximum CCCH, in decimal format
    int lac;    // Location area code, in hexadecimal format
    int ta;     // Timing Advance, in decimal format
    int dbm;    // Receiving level in dBm
    int c1;     // C1 value
    int c2;     // C2 value
} mdm_lbs_cell_t;

struct tm tm;

const char *printError(esp_err_t ret)
{
    switch (ret)
    {
    case ESP_OK:
        return "ESP_OK";
    case ESP_FAIL:
        return "ESP_FAIL";
    case ESP_ERR_NO_MEM:
        return "ESP_ERR_NO_MEM";
    case ESP_ERR_INVALID_ARG:
        return "ESP_ERR_INVALID_ARG";
    case ESP_ERR_INVALID_STATE:
        return "ESP_ERR_INVALID_STATE";
    case ESP_ERR_INVALID_SIZE:
        return "ESP_ERR_INVALID_SIZE";
    case ESP_ERR_NOT_FOUND:
        return "ESP_ERR_NOT_FOUND";
    case ESP_ERR_NOT_SUPPORTED:
        return "ESP_ERR_NOT_SUPPORTED";
    case ESP_ERR_TIMEOUT:
        return "ESP_ERR_TIMEOUT";
    case ESP_ERR_INVALID_RESPONSE:
        return "ESP_ERR_INVALID_RESPONSE";
    case ESP_ERR_INVALID_CRC:
        return "ESP_ERR_INVALID_CRC";
    case ESP_ERR_INVALID_VERSION:
        return "ESP_ERR_INVALID_VERSION";
    case ESP_ERR_INVALID_MAC:
        return "ESP_ERR_INVALID_MAC";
    case ESP_ERR_NOT_FINISHED:
        return "ESP_ERR_NOT_FINISHED";
    case ESP_ERR_NOT_ALLOWED:
        return "ESP_ERR_NOT_ALLOWED";
    default:
        return "UNKNOWN_ESP_ERR";
    }
}

QueueHandle_t _internal_queue;

void power_on_modem(esp_modem_dce_t *dce)
{
    ESP_LOGI(TAG, "Power on the modem");
    gpio_set_level(GPIO_OUTPUT_PWRKEY, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(GPIO_OUTPUT_PWRKEY, 1);
    vTaskDelay(pdMS_TO_TICKS(2000));
    gpio_set_level(GPIO_OUTPUT_PWRKEY, 0);
    vTaskDelay(pdMS_TO_TICKS(2700));
}

void power_off_modem(esp_modem_dce_t *dce)
{
    char _response[512] = {};
    esp_err_t ret = esp_modem_at(dce, "AT+CPOWD=1", _response,1500);
    ESP_LOGI(TAG, "Power off the modem | [%d] %s | %s", ret, printError(ret), _response);
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

    gpio_set_level(GPIO_OUTPUT_POWER_ON, true);
    gpio_set_level(GPIO_OUTPUT_PWRKEY, false);
}

//  https://github.com/whik/crc-lib-c/blob/master/crcLib.c
uint8_t crc8_itu(uint8_t *data, uint16_t length)
{
    uint8_t i;
    uint8_t crc = 0; // Initial value
    while (length--)
    {
        crc ^= *data++; // crc ^= *data; data++;
        for (i = 0; i < 8; i++)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc ^ 0x55;
}

char *generate_position_hex(mdm_lbs_cell_t *cells, GSMTxReq_t *_data, uint8_t *size)
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
    uint16_t positionHexSize = sizeof(GSMTxReq_t) + numberErbs * sizeof(GSMERBPacket_t);
    hexArray = calloc(positionHexSize, sizeof(uint8_t));
    *size = positionHexSize;
    _data->n_erbs = numberErbs;
    memcpy(hexArray, _data, sizeof(GSMTxReq_t));

    for (int i = 0; i < 7; i++)
    {
        if (cells[i].bcch != 0)
        {
            uint16_t _index = sizeof(GSMTxReq_t) + i * sizeof(GSMERBPacket_t);
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
    GSMTxReq_t *element_p = (GSMTxReq_t *)hexArray;
    element_p->crc = crc_calc;

    return hexArray;
}

static void app_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    if (event_base == APP_EVENT && event_id == APP_EVENT_GSM_TX_REQ)
    {
        GSMTxReq_t *element_p = (GSMTxReq_t *)event_data;
        memcpy(&gsmTx, element_p, sizeof(GSMTxReq_t));
        xQueueSend(_internal_queue, &gsmTx, 0);
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

void gsmTask(void *pvParameters)
{
    esp_err_t err, err2Send = ESP_FAIL;
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
    esp_modem_dce_config_t dce_config = ESP_MODEM_DCE_DEFAULT_CONFIG(m_config->gsmConfig.apn);
    esp_netif_config_t netif_ppp_config = ESP_NETIF_DEFAULT_PPP();
    esp_netif_t *esp_netif = esp_netif_new(&netif_ppp_config);
    assert(esp_netif);

    ESP_LOGI(TAG, "Initializing esp_modem for the SIM800 module...");
    esp_modem_dce_t *dce = esp_modem_new_dev(ESP_MODEM_DCE_SIM800, &dte_config, &dce_config, esp_netif);

    config_pwrkey_gpio();
    state = STATE_INIT;
    prev_state = state;
    const uint8_t zeroImei[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (memcmp(m_config->rom.imei, zeroImei, sizeof(m_config->rom.imei)) == 0)
    {
        ESP_LOGW(TAG, "Imei not set");

        uint8_t count = 0;
        do
        {
            power_on_modem(dce);
            esp_err_t _ret = esp_modem_sync(dce);

            if (_ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Error PowerOn: [%d] %s | Trying again! in 3s", _ret, printError(_ret));
                vTaskDelay(pdMS_TO_TICKS(3000));
                count++;
            }
            else
            {
                break;
            }

        } while (count < 4);

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
                    ESP_LOGE(TAG, "Error get imei: [%d] %s | Trying again! in 3s", _ret, printError(_ret));
                    vTaskDelay(pdMS_TO_TICKS(3000));
                    count++;
                }
            } while (count < 4);

            if (count >= 4)
            {
                ESP_LOGE(TAG, "Error getting IMEI - Going to sleep");
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

            power_on_modem(dce);
            esp_err_t _ret = ESP_OK;
            static uint8_t count = 0;

            prev_state = state;

            _ret = esp_modem_sync(dce);

            if (_ret == ESP_OK)
            {
                count = 0;
                state = STATE_SYNC;
            }
            else
            {
                if (count++ < 4)
                {
                    state = STATE_POWERON;
                    ESP_LOGE(TAG, "Error PowerOn: [%d] %s | Trying again! in 3s", _ret, printError(_ret));
                    vTaskDelay(pdMS_TO_TICKS(3000));
                }
                else
                {
                    count = 0;
                    ESP_LOGW(TAG, "Error PowerOn: [%d] %s | Going to sleep", _ret, printError(_ret));
                    err2Send = _ret;
                    state = STATE_REPORT_ERROR;
                }
            }
        }
        break;

        case STATE_SYNC:
        {
            prev_state = state;
            static uint8_t count = 0;
            esp_err_t ret = esp_modem_sync(dce);
            if (ret == ESP_OK)
            {
                esp_modem_at(dce, "ATE0", response, 1000);
                esp_modem_at(dce, "AT+CLTS=1", response, 1000);
                count = 0;
                if (strlen(task_data.imei) == 0)
                {
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
                esp_modem_at(dce, "AT+COPS=0", response, 5000);
                state = STATE_CHECK_SIGNAL;
            }
            else
            {
                if (count++ >= 3)
                {
                    count = 0;
                    err2Send = ret;
                    state = STATE_REPORT_ERROR;
                }
                else
                {
                    ESP_LOGW(TAG, "Error STATE_SYNC [%d]%s | Trying again in 1s", ret, printError(ret));
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
            }

        }
        break;

        case STATE_CHECK_SIGNAL:
        {
            prev_state = state;
            int *prms = (int *)calloc(sizeof(int), 2);

            esp_err_t ret = esp_modem_get_signal_quality(dce, &prms[0], &prms[1]);
            if (ret == ESP_OK)
            {
                if (prms[0] > 5)
                {
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    esp_modem_at(dce, "AT+CENG=4,1", response, 500);
                    state = STATE_CHECK_OPERATOR;
                }
            }
            else if (ret == ESP_ERR_TIMEOUT)
            {
                err2Send = ret;
                state = STATE_REPORT_ERROR;
            }
            else
                vTaskDelay(1000);

            free(prms);
        }
        break;

        case STATE_CHECK_OPERATOR:
        {
            static uint8_t count = 0; 
            prev_state = state;
            char *name = (char *)calloc(64, 1);
            int *val = calloc(sizeof(int), 1);

            esp_err_t ret = esp_modem_get_operator_name(dce, name, val);
            if (ret == ESP_OK)
            {
                ESP_LOGI(TAG, "OK. %s %i", name, *val);
                if (strlen(name) > 0)
                {
                    count = 0;
                    state = STATE_SCAN_NETWORKS;
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
                if(count++ > 20)
                {
                    count = 0;
                    err2Send = ESP_FAIL;
                    state = STATE_REPORT_ERROR;
                }
                else
                {
                    vTaskDelay(1000);
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
            char _response[512]={};
            memset(_response, 0, sizeof(_response));
            if (esp_modem_at_raw(dce, "AT+CENG?\r", _response, "OK", "ERROR", 5000) == ESP_OK)
            {
                ESP_LOGI(TAG, "AT+CENG? response: %s", _response);

                /* get the first token */
                token = strtok(_response, delimiter);

                /* walk through other tokens */
                while (token != NULL && cell_i < 7)
                {
                    if (cell_i >= 0)
                    {
                        ESP_LOGI(TAG, " %s", token);

                        // first cell we are connected on that
                        if (cell_i == 0)
                        {
                            int res = sscanf(token, "%d,\"%d,%d,%d,%d,%d,%d,%X,%d,%d,%X,%d,%d,%d,%d,",
                                             &cells[cell_i].cell, &cells[cell_i].bcch, &cells[cell_i].rxl,
                                             &cells[cell_i].rxq, &cells[cell_i].mcc, &cells[cell_i].mnc,
                                             &cells[cell_i].bsic, &cells[cell_i].cellid, &cells[cell_i].rla,
                                             &cells[cell_i].txp, &cells[cell_i].lac, &cells[cell_i].ta,
                                             &cells[cell_i].dbm, &cells[cell_i].c1, &cells[cell_i].c2);

                            ESP_LOGI(TAG, "SSCANF PRMS PARSED %d", res);
                            ESP_LOGI(TAG, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                                     cells[cell_i].cell, cells[cell_i].bcch, cells[cell_i].rxl,
                                     cells[cell_i].rxq, cells[cell_i].mcc, cells[cell_i].mnc,
                                     cells[cell_i].bsic, cells[cell_i].cellid, cells[cell_i].rla,
                                     cells[cell_i].txp, cells[cell_i].lac, cells[cell_i].ta,
                                     cells[cell_i].dbm, cells[cell_i].c1, cells[cell_i].c2);
                        }
                        else
                        {
                            int res = sscanf(token, "%d,\"%d,%d,%d,%X,%d,%d,%X,%d,%d\"",
                                             &cells[cell_i].cell, &cells[cell_i].bcch, &cells[cell_i].rxl,
                                             &cells[cell_i].bsic, &cells[cell_i].cellid, &cells[cell_i].mcc,
                                             &cells[cell_i].mnc, &cells[cell_i].lac, &cells[cell_i].c1,
                                             &cells[cell_i].c2);

                            ESP_LOGI(TAG, "SSCANF PRMS PARSED %d", res);
                            ESP_LOGI(TAG, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                                     cells[cell_i].cell, cells[cell_i].bcch, cells[cell_i].rxl,
                                     cells[cell_i].bsic, cells[cell_i].cellid, cells[cell_i].mcc,
                                     cells[cell_i].mnc, cells[cell_i].lac, cells[cell_i].c1,
                                     cells[cell_i].c2);
                        }
                    }

                    token = strtok(NULL, delimiter);

                    cell_i++;
                }
            }
            esp_modem_at(dce, "AT+CCLK?", response, 1000);

            //{+CCLK: "24/07/08,09:31:48-12"}
            sscanf(response, "+CCLK: \"%s\"", task_data.date_time);
            int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0, tz = 0;
            char sign = 0x00;

            sscanf(task_data.date_time, "%d/%d/%d,%d:%d:%d%c%d",
                   &year, &month, &day, &hour, &minute, &second, &sign, &tz);
            // tm.tm_isdst = -1;
            tm.tm_year = year + 2000 - 1900;
            tm.tm_mon = month - 1;
            tm.tm_mday = day;
            tm.tm_hour = hour;
            tm.tm_min = minute;
            tm.tm_sec = second;
            time_t t = mktime(&tm);

            struct timeval tv;
            tv.tv_sec = t;
            printf("Setting time: %s | %lld\r\n", asctime(&tm), t);

            settimeofday(&tv, NULL);
            struct timeval current;
            gettimeofday(&current, NULL);
            printf(" Second : %llu \n Microsecond : %06lu\r\n",
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
            //AT+CIPRXGET=1\r
            prev_state = state;
            esp_err_t ret = ESP_OK;
            sprintf(command, "AT+CIPRXGET=1\r\n");
            ret = esp_modem_at_raw(dce, command, response, "OK", "ERROR", 1000);
            ESP_LOGI(TAG, "AT+CIPRXGET=1 ret [%d]%s | response: %s", ret, printError(ret), response);

            sprintf(command, "AT+CSTT=\"%s\"\r\n", gsmElement.apn);
            ret = esp_modem_at_raw(dce, command, response, "OK", "ERROR", 1000);
            ESP_LOGI(TAG, "AT+CSTT ret [%d]%s | response: %s", ret, printError(ret), response);
            
            //+PDP: DEACT => 2B5044503A204445414354
            const char deactString[]={0x2b, 0x50, 0x44, 0x50, 0x3a, 0x20, 0x44, 0x45, 0x41, 0x43, 0x54, 0x00};
            char *_token;
            static uint8_t count = 0;

            // bring ip up
            sprintf(command, "AT+CIICR\r\n");
            ret = esp_modem_at_raw(dce, command, response, "OK", "ERROR",90000);
            ESP_LOGI(TAG, "AT+CIICR ret [%d]%s | response: %s", ret, printError(ret), response);
            
            //get cases of +PDP: DEACT
           _token = strstr(response, deactString);
            if(_token!=NULL) //we have pdp deact
            {
                // If "+PDP: DEACT" URC is reported which means the GPRS is released by the network, 
                // then user still needs to execute "AT+CIPSHUT" command to make PDP context come back to original state
                ESP_LOGW(TAG, "\t\tGOT <+PDP: DEACT> ");
                sprintf(command, "AT+CIPSHUT");
                esp_err_t err = esp_modem_at(dce, command, response, 5000);
                ESP_LOGI(TAG, "\t\t[AT+CIPSHUT] response: %s | err [%d] %s | Trying again in 3s",response, err,printError(err));
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
            else
            {
                count = 0;

                // show ip
                sprintf(command, "AT+CIFSR\r\n");
                ret = esp_modem_at(dce, command, response, 1000);
                ESP_LOGI(TAG, "AT+CIFSR ret [%d]%s | response: %s", ret, printError(ret), response);

                //get cases of +PDP: DEACT
                _token = strstr(response, deactString);
                if(_token!=NULL) //we have pdp deact
                {
                    // If "+PDP: DEACT" URC is reported which means the GPRS is released by the network, 
                    // then user still needs to execute "AT+CIPSHUT" command to make PDP context come back to original state
                    ESP_LOGW(TAG, "\t\tGOT <+PDP: DEACT> | token: %s", _token);
                    sprintf(command, "AT+CIPSHUT");
                    esp_err_t err = esp_modem_at(dce, command, response, 5000);
                    ESP_LOGI(TAG, "\t\t[AT+CIPSHUT] response: %s | err [%d] %s | Trying again in 3s",response, err,printError(err));
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
                else
                {
                    // start socket with server
                    count = 0;
                    sprintf(command, "AT+CIPSTART=\"TCP\",\"%s\",\"%d\"\r\n", gsmElement.server, gsmElement.port);
                    ret = esp_modem_at_raw(dce, command, response, "CONNECT OK", "ERROR", 5000);
                    ESP_LOGI(TAG, "AT+CIPSTART ret [%d]%s | response: %s", ret, printError(ret), response);
                    state = STATE_SEND_PACKET;
                }
            }

        }
        break;

        case STATE_SEND_PACKET:
        {
            uint64_t _imei = strtoll(task_data.imei, NULL, 10);
            gsmElement.imei[0] = (_imei >> (6 * 8)) & 0xFF;
            gsmElement.imei[1] = (_imei >> (5 * 8)) & 0xFF;
            gsmElement.imei[2] = (_imei >> (4 * 8)) & 0xFF;
            gsmElement.imei[3] = (_imei >> (3 * 8)) & 0xFF;
            gsmElement.imei[4] = (_imei >> (2 * 8)) & 0xFF;
            gsmElement.imei[5] = (_imei >> (1 * 8)) & 0xFF;
            gsmElement.imei[6] = (_imei) & 0xFF;
            uint8_t size;
            char *hex = generate_position_hex(cells, &gsmElement, &size);

            char string[512];
            for (uint8_t i = 0; i < size; i++)
            {
                sprintf(&string[i * 2], "%02X", *(hex + i));
            }
            string[size * 2] = 0x00;

            ESP_LOGI(TAG, "To: %s:%d", gsmElement.server, gsmElement.port);
            ESP_LOGI(TAG, "buffer HEX %d bytes: %s", size, string);
            char encode64[450] = {0};
            size_t len_encoded = 0;

            mbedtls_base64_encode((unsigned char *)encode64, sizeof(encode64), &len_encoded, (unsigned char *)hex, size);
            ESP_LOGI(TAG, "buffer base64 %d bytes: %s", len_encoded, encode64);

            sprintf(command, "AT+CIPSEND=%d\r", len_encoded);
            err = esp_modem_at_raw(dce, command, response, ">", "ERROR", 500);

            memcpy(&txDone.params, &gsmElement, sizeof(GSMTxReq_t));
            if (gsmElement.n_erbs != 0)
            {
                memcpy(&txDone.erbs, &cells, gsmElement.n_erbs * sizeof(GSMERBPacket_t));
            }
            txDone.lastState = state;
            prev_state = state;
            
            // send request was accepted
            if (err == ESP_OK)
            {
                err = esp_modem_at_raw(dce, encode64, response, "SEND OK", "SEND FAIL", 90 * 1000);

                if (err)
                {
                    ESP_LOGE(TAG, "Fail to send packet");
                    txDone.lastRet = err;
                    state = STATE_CLOSE_SOCKET;
                }
                else
                {
                    ESP_LOGI(TAG, "Success to send packet");
                    txDone.lastRet = ESP_OK;
                    state = STATE_RECEIVE_PACKET;
                }
            }
            else
            {
                ESP_LOGE(TAG, "Send Request [AT+CIPSEND] Fail [%d] %s", err, printError(err));
                txDone.lastRet = err;
                state = STATE_CLOSE_SOCKET;
            }
            free(hex);
            
            esp_event_post(APP_EVENT, APP_EVENT_GSM_TX_RES, &txDone, sizeof(GSMTxRes_t), 0);

        }
        break;
        
        case STATE_RECEIVE_PACKET:
        {
            prev_state = state;
            int64_t now = esp_timer_get_time();
            esp_err_t err = ESP_FAIL;

            while(esp_timer_get_time() - now < 5000000)
            {
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
                            printf("\t\t[GSM_RX]: %s\r\n\r\n", token);
                    }
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
                else  if (err == ESP_ERR_TIMEOUT)
                {
                    break;
                }
                
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
            state = STATE_CLOSE_SOCKET;
        }
        break;

        case STATE_CLOSE_SOCKET:
        {
            // fecha socket após envio.
            sprintf(command, "AT+CIPCLOSE");
            esp_modem_at(dce, command, response, 5000);
            sprintf(command, "AT+CIPSHUT");
            esp_modem_at(dce, command, response, 5000);
            prev_state = state;
            state = STATE_SLEEP;
        }
        break;

        case STATE_REPORT_ERROR:
        {
            memcpy(&txDone.params, &gsmElement, sizeof(GSMTxReq_t));
            if (gsmElement.n_erbs != 0)
            {
                memcpy(&txDone.erbs, &cells, gsmElement.n_erbs * sizeof(GSMERBPacket_t));
            }
            txDone.lastState = prev_state;
            txDone.lastRet = err2Send;
            esp_event_post(APP_EVENT, APP_EVENT_GSM_TX_RES, &txDone, sizeof(GSMTxRes_t), 0);
            prev_state = state;
            state = STATE_SLEEP;
        }
        break;

        default:
            break;
        }
    }
}
