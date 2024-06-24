#include "esp_log.h"
#include <esp_check.h>
#include <cJSON.h>
#include "esp_netif.h"
#include "esp_netif_ppp.h"
#include "driver/gpio.h"
#include "esp_modem_api.h"
#include "string.h"

static const char *TAG = "MDM_TSK";

#define GPIO_PIN_SEL(pin) (1ULL << pin)
#define ESP_MODEM_C_API_STR_MAX=1024
#define BOARD_DEV
#ifdef BOARD_DEV
#define TXD_PIN ((gpio_num_t)GPIO_NUM_14)
#define RXD_PIN ((gpio_num_t)GPIO_NUM_34)
#define GPIO_OUTPUT_PWRKEY ((gpio_num_t)GPIO_NUM_16)
#define GPIO_OUTPUT_POWER_ON ((gpio_num_t)GPIO_NUM_13)
#define GPIO_OUTPUT_RST ((gpio_num_t)GPIO_NUM_NC)
#else
#define TXD_PIN ((gpio_num_t)27)
#define RXD_PIN ((gpio_num_t)26)
#define GPIO_OUTPUT_PWRKEY ((gpio_num_t)16)
#define GPIO_OUTPUT_POWER_ON ((gpio_num_t)23)
#define GPIO_OUTPUT_RST ((gpio_num_t)5)
#endif

// #define SETTINGS_DEFAULT_GSM_APN "iot4u.br"
// #define SETTINGS_DEFAULT_GSM_USER "arquia"
// #define SETTINGS_DEFAULT_GSM_PASSWORD "arquia"

// #define SETTINGS_DEFAULT_GSM_APN "virtueyes.com.br"
// #define SETTINGS_DEFAULT_GSM_USER "virtu"
// #define SETTINGS_DEFAULT_GSM_PASSWORD "virtu"

#define SETTINGS_DEFAULT_GSM_APN "simplepm.algar.br"
#define SETTINGS_DEFAULT_SERVER "mogno.ceabs.net"
#define SETTINGS_DEFAULT_SERVER_PORT 9015

#define SETTINGS_DEFAULT_SENDING_INTERVAL 60000

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
            ESP_LOGE(TAG, "Failed with %s", ret == ESP_ERR_TIMEOUT ? "TIMEOUT" : "ERROR"); \
        }                                                                                  \
    } while (0)

QueueHandle_t _internal_queue;

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

    vTaskDelay(10000);
}

void power_on_modem(esp_modem_dce_t *dce)
{
    gpio_set_level(GPIO_OUTPUT_POWER_ON, false);
    // gpio_set_level(GPIO_OUTPUT_RST, true);

    /* Power on the modem */
    ESP_LOGI(TAG, "Power on the modem");
    gpio_set_level(GPIO_OUTPUT_PWRKEY, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(GPIO_OUTPUT_PWRKEY, 0);
    vTaskDelay(pdMS_TO_TICKS(1700));
    gpio_set_level(GPIO_OUTPUT_PWRKEY, 1);
    vTaskDelay(pdMS_TO_TICKS(3000));
    CHECK_ERR(esp_modem_sync(dce), ESP_LOGI(TAG, "OK"));
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

void modem_task_function(void *pvParameters)
{
    esp_err_t err;
    en_task_state state = STATE_SYNC;
    task_data_t task_data = {};

    char command[128] = {};
    char response[512] = {};
    mdm_lbs_cell_t cells[7] = {};
    ESP_LOGI(TAG, "modem task starting");

    /* Configure and create the UART DTE */
    esp_modem_dte_config_t dte_config = ESP_MODEM_DTE_DEFAULT_CONFIG();
    /* setup UART specific configuration based on kconfig options */
    dte_config.uart_config.tx_io_num = TXD_PIN;
    dte_config.uart_config.rx_io_num = RXD_PIN;
    dte_config.uart_config.rts_io_num = -1;
    dte_config.uart_config.cts_io_num = -1;
    // dte_config.uart_config.rx_buffer_size = CONFIG_EXAMPLE_MODEM_UART_RX_BUFFER_SIZE;
    // dte_config.uart_config.tx_buffer_size = CONFIG_EXAMPLE_MODEM_UART_TX_BUFFER_SIZE;
    // dte_config.uart_config.event_queue_size = CONFIG_EXAMPLE_MODEM_UART_EVENT_QUEUE_SIZE;
    // dte_config.task_stack_size = CONFIG_EXAMPLE_MODEM_UART_EVENT_TASK_STACK_SIZE * 2;
    // dte_config.task_priority = CONFIG_EXAMPLE_MODEM_UART_EVENT_TASK_PRIORITY;
    // dte_config.dte_buffer_size = CONFIG_EXAMPLE_MODEM_UART_RX_BUFFER_SIZE / 2;

    // esp_modem_dce_config_t dce_config = ESP_MODEM_DCE_DEFAULT_CONFIG("simplepm.algar.br");
    esp_modem_dce_config_t dce_config = ESP_MODEM_DCE_DEFAULT_CONFIG(SETTINGS_DEFAULT_GSM_APN);
    esp_netif_config_t netif_ppp_config = ESP_NETIF_DEFAULT_PPP();
    esp_netif_t *esp_netif = esp_netif_new(&netif_ppp_config);
    assert(esp_netif);

    ESP_LOGI(TAG, "Initializing esp_modem for the SIM800 module...");
    esp_modem_dce_t *dce = esp_modem_new_dev(ESP_MODEM_DCE_SIM800, &dte_config, &dce_config, esp_netif);

    config_pwrkey_gpio();
    power_on_modem(dce);
    vTaskDelay(pdMS_TO_TICKS(2000));

    for (;;)
    {
        ESP_LOGW("MODEM", "state = %d", state);
        switch (state)
        {  

        case STATE_INIT:

            //@ TODO - Usar funções para checar alocação do recurso
            _internal_queue = xQueueCreate(10, sizeof(event_t));

            break;

        case STATE_DEINIT:

            vQueueDelete(_internal_queue);

            // TODO - Desalocar recursos que foram criados nesse estado.

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

            sscanf(response, "+CCLK: \"%s\"", task_data.date_time);
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

        case STATE_WAIT_EVENT:

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

            char *json = generate_position_json(cells, &task_data);
            ESP_LOGI(TAG, "Sending JSON: %s", json);

            sprintf(command, "AT+CIPSEND=%d\r", strlen(json));
            err = esp_modem_at_raw(dce, command, response, ">", "ERROR", 500);

            // send request was accepted
            if (err == ESP_OK)
            {
                err = esp_modem_at_raw(dce, json, response, "SEND OK", "SEND FAIL", 645 * 1000);

                if (err)
                    ESP_LOGW(TAG, "Fail to send packet");
                else
                    ESP_LOGI(TAG, "Success to send packet");
            }
            free(json);

            // fecha socket após envio.
            sprintf(command, "AT+CIPCLOSE");
            esp_modem_at(dce, command, response, 5000);
            vTaskDelay(pdMS_TO_TICKS(SETTINGS_DEFAULT_SENDING_INTERVAL));
            state = STATE_SCAN_NETWORKS;
            break;

        default:
            break;
        }
    }
}
