#include "esp_log.h"
#include <esp_check.h>
#include <cJSON.h>
#include "esp_netif.h"
#include "esp_netif_ppp.h"
#include "driver/gpio.h"
#include "esp_modem_api.h"
#include "string.h"
#include "sys/time.h"
#include "stateMachine.hpp"
#include "modem.hpp"
 #define CORE_DEBUG_LEVEL   5
static const char *TAG = "MDM_TSK";

#define GPIO_PIN_SEL(pin) (1ULL << pin)
#define TXD_PIN ((gpio_num_t)GPIO_NUM_14)
#define RXD_PIN ((gpio_num_t)GPIO_NUM_34)
#define GPIO_OUTPUT_PWRKEY ((gpio_num_t)GPIO_NUM_16)
#define GPIO_OUTPUT_POWER_ON ((gpio_num_t)GPIO_NUM_13)
#define GPIO_OUTPUT_RST ((gpio_num_t)GPIO_NUM_NC)

#define SETTINGS_DEFAULT_GSM_APN "simplepm.algar.br"
#define SETTINGS_DEFAULT_SERVER "0.tcp.sa.ngrok.io"
#define SETTINGS_DEFAULT_SERVER_PORT 12561

#define SETTINGS_DEFAULT_SENDING_INTERVAL 60000

static GSMElementTx_t gsmTx;

typedef enum
{
    STATE_INIT,
    STATE_DEINIT,
    STATE_SYNC,
    STAT_CONFIG_PRMS,
    STATE_CHECK_SIGNAL,
    STATE_CHECK_OPERERATOR,
    STATE_SCAN_NETWORKS,
    STATE_GET_IP,
    STATE_GET_LBS_POSITION,
    STATE_CHECK_IP,
    STATE_SEND_PACKET,
    STATE_RECEIVE_PACKET,
    STATE_WAIT_EVENT,
    STATE_SLEEP,
    STATE_POWERON,
} en_task_state;

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

const char* printError(esp_err_t ret)
{
    switch(ret)
    {
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
        return "UNKNOW_ESP_ERR";
    }

}

#define CHECK_ERR(cmd, success_action)                                                     \
    do                                                                                     \
    {                                                                                      \
        esp_err_t ret = cmd;                                                               \
        if (ret == ESP_OK)                                                                 \
        {                                                                                  \
            success_action;                                                                \
        }                                                                                  \
        else                                                                               \
        {                                                                                  \
            ESP_LOGE(TAG, "Failed with %s | %d", printError(ret), ret);                    \
        }                                                                                  \
    } while (0)

QueueHandle_t _internal_queue;

void power_on_modem(esp_modem_dce_t *dce)
{
    // gpio_set_level(GPIO_OUTPUT_POWER_ON, false);
    // gpio_set_level(GPIO_OUTPUT_RST, true);

    /* Power on the modem */
    ESP_LOGI(TAG, "Power on the modem");
    //gpio_set_level(GPIO_OUTPUT_PWRKEY, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(GPIO_OUTPUT_PWRKEY, 1);
    vTaskDelay(pdMS_TO_TICKS(2000));
    gpio_set_level(GPIO_OUTPUT_PWRKEY, 0);
    vTaskDelay(pdMS_TO_TICKS(2700));
    // CHECK_ERR(esp_modem_sync(dce), ESP_LOGI(TAG, "OK"));
}

void power_off_modem()
{
    ESP_LOGI(TAG, "Power off the modem");
    // gpio_set_level(GPIO_OUTPUT_PWRKEY, 1);
    // vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(GPIO_OUTPUT_PWRKEY, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(GPIO_OUTPUT_PWRKEY, 0);
    //vTaskDelay(pdMS_TO_TICKS(2700));
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

    power_off_modem();
    vTaskDelay(2000);
}


char *generate_position_json(mdm_lbs_cell_t *cells, task_data_t *task_data)
{
    char *string = NULL;
    // esp_err_t ret;
    cJSON *root, *data;

    root = cJSON_CreateObject();

    cJSON_AddStringToObject(
        root,
        "type",
        "location");
    cJSON_AddStringToObject(
        root,
        "imei",
        task_data->imei);
    cJSON_AddStringToObject(
        root,
        "fw",
        "0.0.1");
    cJSON_AddStringToObject(
        root,
        "hw",
        "1.0");

    cJSON_AddStringToObject(
        root,
        "time",
        task_data->date_time);

    data = cJSON_AddObjectToObject(
        root,
        "flags");

    cJSON_AddBoolToObject(data, "emergency", false);
    cJSON_AddBoolToObject(data, "jammGSM", false);
    cJSON_AddBoolToObject(data, "jammLora", false);
    cJSON_AddBoolToObject(data, "lowBat", false);

    data = cJSON_AddArrayToObject(root, "erbs");

    for (int i = 0; i < 7; i++)
    {
        if (cells[i].bcch != 0)
        {
            cJSON *erb = cJSON_CreateObject();

            cJSON_AddNumberToObject(erb, "id", cells[i].cell);
            cJSON_AddNumberToObject(erb, "lac", cells[i].lac);
            cJSON_AddNumberToObject(erb, "mcc", cells[i].mcc);
            cJSON_AddNumberToObject(erb, "mnc", cells[i].mnc);
            cJSON_AddNumberToObject(erb, "bsic", cells[i].bsic);
            cJSON_AddNumberToObject(erb, "cellid", cells[i].cellid);
            cJSON_AddNumberToObject(erb, "ta", cells[i].ta);

            cJSON_AddItemToArray(data, erb);
        }
    }
    string = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return string;
}
uint8_t crc8_itu(uint8_t *data, uint16_t length)
{
    uint8_t i;
    uint8_t crc = 0;        // Initial value
    while(length--)
    {
        crc ^= *data++;        // crc ^= *data; data++;
        for ( i = 0; i < 8; i++ )
        {
            if ( crc & 0x80 )
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc ^ 0x55;
}

char *generate_position_hex(mdm_lbs_cell_t *cells, GSMElementTx_t *_data)
{
    char *string = NULL;
    uint8_t *hexArray = NULL;
    uint8_t numberErbs = 0;

    for (int i = 0; i < 7; i++)
    {
        if (cells[i].bcch != 0)
        {
            numberErbs++;
        }
    }
    uint8_t positionHexSize = sizeof(GSMElementTx_t) + numberErbs*sizeof(GSMERBPacket_t);
    hexArray = calloc(positionHexSize, sizeof(uint8_t));

    _data->n_erbs =  numberErbs;
    memcpy(hexArray, _data, sizeof(GSMElementTx_t));

    for (int i = 0; i < 7; i++)
    {
        if (cells[i].bcch != 0)
        {
            uint16_t _index = sizeof(GSMElementTx_t) + i*sizeof(GSMERBPacket_t);
            GSMERBPacket_t *erbPacket_p = (GSMERBPacket_t*)(hexArray + _index);
            erbPacket_p->id[0] = 0x00;//(cells[i].cellid>>(4*8)) * 0xff;
            erbPacket_p->id[1] = (cells[i].cellid>>(3*8)) * 0xff;
            erbPacket_p->id[2] = (cells[i].cellid>>(2*8)) * 0xff;
            erbPacket_p->id[3] = (cells[i].cellid>>(1*8)) * 0xff;
            erbPacket_p->id[4] = (cells[i].cellid) * 0xff;
            erbPacket_p->mcc[0] = (cells[i].mcc >> 8 ) &0xFF;
            erbPacket_p->mcc[1] = (cells[i].mcc ) &0xFF;
            erbPacket_p->mnc[0] = (cells[i].mnc >> 8 ) &0xFF;
            erbPacket_p->mnc[1] = (cells[i].mnc ) &0xFF;
            erbPacket_p->bsic = cells[i].bsic;
            erbPacket_p->ta = cells[i].ta;
            erbPacket_p->lac[0] = 0x00;//(cells[i].lac>>(4*8)) * 0xff;
            erbPacket_p->lac[1] = (cells[i].lac>>(3*8)) * 0xff;
            erbPacket_p->lac[2] = (cells[i].lac>>(2*8)) * 0xff;
            erbPacket_p->lac[3] = (cells[i].lac>>(1*8)) * 0xff;
            erbPacket_p->lac[4] = (cells[i].lac) * 0xff;
        }
    }
    uint8_t crc_calc=crc8_itu( hexArray, positionHexSize);
    GSMElementTx_t *element_p = (GSMElementTx_t*)hexArray;
    element_p->crc = crc_calc;


    string = calloc(positionHexSize*2+1, sizeof(uint8_t));
    for(uint8_t i = 0; i < positionHexSize; i++)
    {
        sprintf(string+i*2, "%02X", *(hexArray + i));
    }
    *(string + positionHexSize*2) = 0x00;
    free(hexArray);
    return string;
}

static void app_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == APP_EVENT && event_id == APP_EVENT_REQ_GSM_SEND)
    {
        GSMElementTx_t *element_p = (GSMElementTx_t*) event_data;
        memcpy(&gsmTx, element_p, sizeof(GSMElementTx_t));
        xQueueSend(_internal_queue, &gsmTx,0);
    }
}

void modem_task_function(void *pvParameters)
{
    esp_err_t err;
    en_task_state state;
    task_data_t task_data = {};

    char command[128] = {};
    char response[512] = {};
    mdm_lbs_cell_t cells[7] = {};
    GSMElementTx_t gsmElement;
    ESP_LOGI(TAG, "modem task starting");

    /* Configure and create the UART DTE */
    esp_modem_dte_config_t dte_config = ESP_MODEM_DTE_DEFAULT_CONFIG();
    /* setup UART specific configuration based on kconfig options */
    dte_config.uart_config.tx_io_num = TXD_PIN;
    dte_config.uart_config.rx_io_num = RXD_PIN;
    dte_config.uart_config.rts_io_num = -1;
    dte_config.uart_config.cts_io_num = -1;
    esp_modem_dce_config_t dce_config = ESP_MODEM_DCE_DEFAULT_CONFIG(SETTINGS_DEFAULT_GSM_APN);
    esp_netif_config_t netif_ppp_config = ESP_NETIF_DEFAULT_PPP();
    esp_netif_t *esp_netif = esp_netif_new(&netif_ppp_config);
    assert(esp_netif);

    ESP_LOGI(TAG, "Initializing esp_modem for the SIM800 module...");
    esp_modem_dce_t *dce = esp_modem_new_dev(ESP_MODEM_DCE_SIM800, &dte_config, &dce_config, esp_netif);

    config_pwrkey_gpio();
    state = STATE_INIT;

    for (;;)
    {
        switch (state)
        {

        case STATE_INIT:
            //@ TODO - Usar funções para checar alocação do recurso
            _internal_queue = xQueueCreate(10, sizeof(GSMElementTx_t));
            esp_event_handler_instance_register(APP_EVENT, ESP_EVENT_ANY_ID, &app_event_handler, NULL, NULL);
            state = STATE_SLEEP;
            break;

        case STATE_DEINIT:
            vQueueDelete(_internal_queue);
            // TODO - Desalocar recursos que foram criados nesse estado.
            break;

        case STATE_WAIT_EVENT:
                //vTaskDelay(pdMS_TO_TICKS(60000));
                if(xQueueReceive(_internal_queue, &gsmElement, portMAX_DELAY))
                {
                    state = STATE_POWERON;
                }
            break;

        case STATE_POWERON:
            power_on_modem(dce);
            esp_err_t ret = ESP_OK;
            ret = esp_modem_sync(dce);
            if(ret == ESP_OK)
            {
                state = STATE_SYNC;
            }
            else
            {
                ESP_LOGE(TAG, "Error PowerOn: %d", ret);
                state = STATE_WAIT_EVENT;
            }
            break;


        case STATE_SYNC:

            if (esp_modem_sync(dce) == ESP_OK)
            {
                esp_modem_at(dce, "ATE0", response, 1000);
                esp_modem_at(dce, "AT+CLTS=1", response, 1000);
                // esp_modem_set_echo(dce, false);
                state = STATE_CHECK_SIGNAL;
            }
            else
            {
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            if (esp_modem_get_imei(dce, task_data.imei) == ESP_OK)
                ESP_LOGI(TAG, "Modem imei: %s", task_data.imei);

            vTaskDelay(1000);
            esp_modem_at(dce, "AT+COPS=0", response, 5000);
            // esp_modem_at(dce, "AT+COPS=2", response, 5000);
            // esp_modem_at(dce, "AT+SIMEI=863070046562780", response, 1000);
            // esp_modem_at(dce, "AT+COPS=0", response, 5000);
            // /* code */
            break;

        case STATE_CHECK_SIGNAL:

            int *prms = (int *)calloc(sizeof(int), 2);

            CHECK_ERR(esp_modem_get_signal_quality(dce, &prms[0], &prms[1]), ESP_LOGI(TAG, "rssi: %i, bear: %i", prms[0], prms[1]));

            if (prms[0] > 5)
            {
                // esp_modem_at(dce, "AT+COPS=0", data, 500);
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_modem_at(dce, "AT+CENG=4,1", response, 500);
                // esp_modem_set_operator(dce, 0);
                state = STATE_CHECK_OPERERATOR;
            }
            else
                vTaskDelay(1000);
            /* code */
            free(prms);
            break;

        case STATE_CHECK_OPERERATOR:

            char *name = (char *)calloc(64, 1);
            int *val = calloc(sizeof(int), 1);

            CHECK_ERR(esp_modem_get_operator_name(dce, name, val), ESP_LOGI(TAG, "OK. %s %i", name, *val));

            if (strlen(name) > 0)
            {
                // CHECK_ERR(esp_modem_at(dce, "AT+CNETSCAN=1", data, 500), ESP_LOGI(TAG, "OK"));
                state = STATE_SCAN_NETWORKS;
            }

            else
                vTaskDelay(1000);
            /* code */
            free(name);
            free(val);
            break;

        case STATE_SCAN_NETWORKS:
            const char delimiter[] = "\r\n+CENG: ";
            char *token;
            int cell_i = -1;
            if (esp_modem_at_raw(dce, "AT+CENG?\r\n", response, "OK", "ERROR", 5000) == ESP_OK)
            {
                /* get the first token */
                token = strtok(response, delimiter);

                /* walk through other tokens */
                while (token != NULL)
                {
                    if (cell_i >= 0)
                    {
                        ESP_LOGI(TAG, " %s", token);

                        // first cell we are connected on that
                        if (cell_i == 0)
                        {
                            int res = sscanf(token, "%d,\"%d,%d,%d,%d,%d,%d,%X,%d,%d,%X,%d,%d,%d,%d,",
                                             &cells[cell_i].cell,
                                             &cells[cell_i].bcch,
                                             &cells[cell_i].rxl,
                                             &cells[cell_i].rxq,
                                             &cells[cell_i].mcc,
                                             &cells[cell_i].mnc,
                                             &cells[cell_i].bsic,
                                             &cells[cell_i].cellid,
                                             &cells[cell_i].rla,
                                             &cells[cell_i].txp,
                                             &cells[cell_i].lac,
                                             &cells[cell_i].ta,
                                             &cells[cell_i].dbm,
                                             &cells[cell_i].c1,
                                             &cells[cell_i].c2);

                            ESP_LOGI(TAG, "SSCANF PRMS PARSED %d", res);
                            ESP_LOGI(TAG, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                                     cells[cell_i].cell,
                                     cells[cell_i].bcch,
                                     cells[cell_i].rxl,
                                     cells[cell_i].rxq,
                                     cells[cell_i].mcc,
                                     cells[cell_i].mnc,
                                     cells[cell_i].bsic,
                                     cells[cell_i].cellid,
                                     cells[cell_i].rla,
                                     cells[cell_i].txp,
                                     cells[cell_i].lac,
                                     cells[cell_i].ta,
                                     cells[cell_i].dbm,
                                     cells[cell_i].c1,
                                     cells[cell_i].c2);
                        }
                        else
                        {
                            int res = sscanf(token, "%d,\"%d,%d,%d,%X,%d,%d,%X,%d,%d\"",
                                             &cells[cell_i].cell,
                                             &cells[cell_i].bcch,
                                             &cells[cell_i].rxl,
                                             &cells[cell_i].bsic,
                                             &cells[cell_i].cellid,
                                             &cells[cell_i].mcc,
                                             &cells[cell_i].mnc,
                                             &cells[cell_i].lac,
                                             &cells[cell_i].c1,
                                             &cells[cell_i].c2);

                            ESP_LOGI(TAG, "SSCANF PRMS PARSED %d", res);
                            ESP_LOGI(TAG, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                                     cells[cell_i].cell,
                                     cells[cell_i].bcch,
                                     cells[cell_i].rxl,
                                     cells[cell_i].bsic,
                                     cells[cell_i].cellid,
                                     cells[cell_i].mcc,
                                     cells[cell_i].mnc,
                                     cells[cell_i].lac,
                                     cells[cell_i].c1,
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

            //int32_t unixtempo = (int32_t) t;
            struct timeval tv;
            tv.tv_sec = t;
            printf("Setting time: %s | %lld\r\n", asctime(&tm), t);

            settimeofday(&tv, NULL);
            struct timeval current;
            gettimeofday(&current, NULL);
            printf(" Second : %llu \n Microsecond : %06lu\r\n",
                current.tv_sec, current.tv_usec);
            // struct timeval now = { .tv_sec = t };
            // settimeofday(&now, NULL);




            char buf[255];
            strftime(buf, sizeof(buf), "%d %b %Y %H:%M:%S", &tm);
            ESP_LOGW(TAG, "%s", buf);
            // CHECK_ERR(, ESP_LOGI(TAG, "%s", response));

            // vTaskDelay(pdMS_TO_TICKS());
            state = STATE_SEND_PACKET;
            /* code */
            break;
        case STATE_GET_IP:
            /* code */
            break;

        case STATE_CHECK_IP:
            /* code */
            break;

        case STATE_GET_LBS_POSITION:

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
            state = STATE_SCAN_NETWORKS;
            break;


        case STATE_SEND_PACKET:

            sprintf(command, "AT+CSTT=\"%s\"", SETTINGS_DEFAULT_GSM_APN);
            esp_modem_at(dce, command, response, 1000);

            // bring ip up
            sprintf(command, "AT+CIICR");
            esp_modem_at(dce, command, response, 90000);

            // show ip
            sprintf(command, "AT+CIFSR");
            esp_modem_at(dce, command, response, 1000);

            // start socket with server
            sprintf(command, "AT+CIPSTART=\"TCP\",\"%s\",\"%d\"\r\n", SETTINGS_DEFAULT_SERVER, SETTINGS_DEFAULT_SERVER_PORT);
            esp_modem_at_raw(dce, command, response, "CONNECT OK", "ERROR", 5000);

#ifdef _SEND_JSON
            char *json = generate_position_json(cells, &task_data);
            ESP_LOGI(TAG, "Sending JSON: %s", json);

            sprintf(command, "AT+CIPSEND=%d\r", strlen(json));
            err = esp_modem_at_raw(dce, command, response, ">", "ERROR", 500);

            // send request was accepted
            if (err == ESP_OK)
            {
                err = esp_modem_at_raw(dce, json, response, "SEND OK", "SEND FAIL", 90 * 1000);

                if (err)
                    ESP_LOGW(TAG, "Fail to send packet");
                else
                    ESP_LOGI(TAG, "Success to send packet");
            }
            free(json);
#else
            uint64_t _imei = strtoll(task_data.imei, NULL, 10);
            gsmElement.imei[0] = (_imei>>(6*8)) & 0xFF;
            gsmElement.imei[1] = (_imei>>(5*8)) & 0xFF;
            gsmElement.imei[2] = (_imei>>(4*8)) & 0xFF;
            gsmElement.imei[3] = (_imei>>(3*8)) & 0xFF;
            gsmElement.imei[4] = (_imei>>(2*8)) & 0xFF;
            gsmElement.imei[5] = (_imei>>(1*8)) & 0xFF;
            gsmElement.imei[6] = (_imei) & 0xFF;
            char *hex = generate_position_hex(cells, &gsmElement);
            ESP_LOGI(TAG, "Sending HEX: %s", hex);

            sprintf(command, "AT+CIPSEND=%d\r", strlen(hex));
            err = esp_modem_at_raw(dce, command, response, ">", "ERROR", 500);

            // send request was accepted
            if (err == ESP_OK)
            {
                err = esp_modem_at_raw(dce, hex, response, "SEND OK", "SEND FAIL", 90 * 1000);

                if (err)
                    ESP_LOGW(TAG, "Fail to send packet");
                else
                    ESP_LOGI(TAG, "Success to send packet");
            }
            free(hex);

#endif
            // fecha socket após envio.
            sprintf(command, "AT+CIPCLOSE");
            esp_modem_at(dce, command, response, 5000);
            
            state = STATE_SLEEP;
            break;
        case STATE_SLEEP:
            power_off_modem();
            state = STATE_WAIT_EVENT;
        break;
        default:
            break;
        }
    }
}