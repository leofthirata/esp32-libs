#include "main.h"
#include "sdkconfig.h"
#include "esp_mac.h"
#include "Button.h"
#include "AT24C02D/at24c02d.hpp"

#include "sensors.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led.hpp"
#include "lora.hpp"
#include "esp_console.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "cmd_system.h"

#include "stateMachine.hpp"
#include "esp_netif.h"
#include "esp_event.h"
#include <esp_sleep.h>
#include <rom/rtc.h>

#include "LIS2DW12Sensor.h"
#include "AT24C02D/at24c02d.hpp"
#include "testWriteOTP.h"

#include "esp_log.h"

#include "testGetImei.h"
#include "gsm.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "mbedtls/base64.h"
#include "sys/time.h"

#define AT24C02D_MEMORY_SIZE 256
#define CONFIG_SDA_GPIO GPIO_NUM_27
#define CONFIG_SCL_GPIO GPIO_NUM_26
#define CONFIG_WP_GPIO GPIO_NUM_12

typedef struct 
{
    uint8_t memVer;
    uint8_t hwVer;
    uint8_t prefixSN;
    uint8_t loraID[3];
    uint8_t devAddr[4];
    uint8_t devEUI[8];
    uint8_t appEUI[8];
    uint8_t nwSKey[16];
    uint8_t appSKey[16];
    uint8_t bleMac[6];
    uint8_t imei[7];
} OTPMemory_t;

typedef union 
{
    OTPMemory_t asParam;
    uint8_t asArray[sizeof(OTPMemory_t)];
} OTPMemoryUnion_t;

static OTPMemory_t readMemory;

uint8_t mac[6];
static const char *TAG = "jiga.cpp";

using namespace BiColorStatus;

static Isca_t m_config;
OTPMemoryUnion_t otp;
uint8_t incomingByte = 0, temp = 0;
uint8_t rcv[62];
uint8_t rcvServerPort[6];

TaskHandle_t m_otp_task;
TaskHandle_t m_acc_task;
TaskHandle_t m_ble_task;
TaskHandle_t m_r800c_task;
TaskHandle_t m_lora_task;

static GSMTxRes_t gsmTxRes;
static TaskHandle_t xTaskToNotify = NULL;

bool is_otp_ok = false;
bool is_gsm_port_ok = false;

#define NGROK_SERVER "0.tcp.sa.ngrok.io"
uint32_t ngrok_port = 0;

static void _sendPosition();

uint8_t dallas_crc8(const uint8_t *pdata, const uint32_t size)
{
    uint8_t crcr = 0;
    for (uint32_t i = 0; i < size; ++i)
    {
        uint8_t inbyte = pdata[i];
        for (uint8_t j = 0; j < 8; ++j)
        {
            uint8_t mix = (crcr ^ inbyte) & 0x01;
            crcr >>= 1;
            if (mix)
                crcr ^= 0x8C;
            inbyte >>= 1;
        }
    }
    return crcr;
}

#define GATTS_TAG "GATTS_DEMO"

/// Declare the static function
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

#define GATTS_SERVICE_UUID_TEST_A 0x00FF
#define GATTS_CHAR_UUID_TEST_A 0xFF01
#define GATTS_DESCR_UUID_TEST_A 0x3333
#define GATTS_NUM_HANDLE_TEST_A 0x11

#define TEST_DEVICE_NAME "ISCA DUT"
#define TEST_MANUFACTURER_DATA_LEN 17

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

#define PREPARE_BUF_MAX_SIZE 1024

static uint8_t char1_str[] = {0x11, 0x22, 0x33};
static esp_gatt_char_prop_t a_property = 0;
static esp_gatt_char_prop_t b_property = 0;

static esp_attr_value_t gatts_demo_char1_val =
    {
        .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
        .attr_len = sizeof(char1_str),
        .attr_value = char1_str,
};

static uint8_t adv_config_done = 0;
#define adv_config_flag (1 << 0)
#define scan_rsp_config_flag (1 << 1)

#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
    0x02, 0x01, 0x06,       // Length 2, Data Type 1 (Flags), Data 1 (LE General Discoverable Mode, BR/EDR Not Supported)
    0x02, 0xA0, 0xeb,       // Length 2, Data Type 10 (TX power leve), Data 2 (-21)
    0x03, 0x03, 0xab, 0xcd, // Length 3, Data Type 3 (Complete 16-bit Service UUIDs), Data 3 (UUID)
};
static uint8_t raw_scan_rsp_data[] = { // Length 15, Data Type 9 (Complete Local Name), Data 1 (ESP_GATTS_DEMO)
    0x0f, 0x09, 0x45, 0x53, 0x50, 0x5f, 0x47, 0x41, 0x54, 0x54, 0x53, 0x5f, 0x44,
    0x45, 0x4d, 0x4f};
#else

static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    // first uuid, 16bit, [12],[13] is the value
    0xfb,
    0x34,
    0x9b,
    0x5f,
    0x80,
    0x00,
    0x00,
    0x80,
    0x00,
    0x10,
    0x00,
    0x00,
    0xEE,
    0x00,
    0x00,
    0x00,
    // second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb,
    0x34,
    0x9b,
    0x5f,
    0x80,
    0x00,
    0x00,
    0x80,
    0x00,
    0x10,
    0x00,
    0x00,
    0xFF,
    0x00,
    0x00,
    0x00,
};

// The length of adv data must be less than 31 bytes
// static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
// adv data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006, // slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, // slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0,       // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    //.min_interval = 0x0006,
    //.max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,       // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

#endif /* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0

struct gatts_profile_inst
{
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gatts_cb = gatts_profile_a_event_handler,
        .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

typedef struct
{
    uint8_t *prepare_buf;
    int prepare_len;
} prepare_type_env_t;

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
#ifdef CONFIG_SET_RAW_ADV_DATA
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#else
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#endif
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        // advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TAG, "Advertising start failed\n");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TAG, "Advertising stop failed\n");
        }
        else
        {
            ESP_LOGI(GATTS_TAG, "Stop adv successfully\n");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(GATTS_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                 param->update_conn_params.status,
                 param->update_conn_params.min_int,
                 param->update_conn_params.max_int,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
        case ESP_GATTS_REG_EVT:
        {
            ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
            gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
            gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
            gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_A;

            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(TEST_DEVICE_NAME);
            if (set_dev_name_ret)
            {
                ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }
#ifdef CONFIG_SET_RAW_ADV_DATA
            esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
            if (raw_adv_ret)
            {
                ESP_LOGE(GATTS_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
            }
            adv_config_done |= adv_config_flag;
            esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
            if (raw_scan_ret)
            {
                ESP_LOGE(GATTS_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
            }
            adv_config_done |= scan_rsp_config_flag;
#else
            // config adv data
            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret)
            {
                ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
            }
            adv_config_done |= adv_config_flag;
            // config scan response data
            ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
            if (ret)
            {
                ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
            }
            adv_config_done |= scan_rsp_config_flag;

#endif
            esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_A);
            break;
        }
        case ESP_GATTS_READ_EVT:
        {
            ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.len = 6;
            rsp.attr_value.value[0] = 0xA0;
            rsp.attr_value.value[1] = 0xaf;
            rsp.attr_value.value[2] = 0x03;
            rsp.attr_value.value[3] = 0x01;
            rsp.attr_value.value[4] = 0xf0;
            rsp.attr_value.value[5] = 0xff;
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                        ESP_GATT_OK, &rsp);
            
            _sendPosition();
            esp_event_post(APP_EVENT, APP_EVENT_SEND_POS, NULL, NULL, 0);
            // xTaskNotify(m_otp_task, 0, eSetValueWithOverwrite);
            break;
        }
        case ESP_GATTS_MTU_EVT:
        {
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        }
        case ESP_GATTS_UNREG_EVT:
        {
            break;
        }
        case ESP_GATTS_CREATE_EVT:
        {
            ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
            gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
            gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_A;

            esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);
            a_property = ESP_GATT_CHAR_PROP_BIT_READ;
            esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
                                                            ESP_GATT_PERM_READ,
                                                            a_property,
                                                            &gatts_demo_char1_val, NULL);
            if (add_char_ret)
            {
                ESP_LOGE(GATTS_TAG, "add char failed, error code =%x", add_char_ret);
            }
            break;
        }
        case ESP_GATTS_ADD_INCL_SRVC_EVT:
        {
            break;
        }
        case ESP_GATTS_ADD_CHAR_EVT:
        {
            uint16_t length = 0;
            const uint8_t *prf_char;

            ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
                    param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
            gl_profile_tab[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;
            gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
            esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle, &length, &prf_char);
            if (get_attr_ret == ESP_FAIL)
            {
                ESP_LOGE(GATTS_TAG, "ILLEGAL HANDLE");
            }

            ESP_LOGI(GATTS_TAG, "the gatts demo char length = %x\n", length);
            for (int i = 0; i < length; i++)
            {
                ESP_LOGI(GATTS_TAG, "prf_char[%x] =%x\n", i, prf_char[i]);
            }
            esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].descr_uuid,
                                                                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
            if (add_descr_ret)
            {
                ESP_LOGE(GATTS_TAG, "add char descr failed, error code =%x", add_descr_ret);
            }
            break;
        }
        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        {
            gl_profile_tab[PROFILE_A_APP_ID].descr_handle = param->add_char_descr.attr_handle;
            ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
                    param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
            break;
        }
        case ESP_GATTS_DELETE_EVT:
        {
            break;
        }
        case ESP_GATTS_START_EVT:
        {
            ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
                    param->start.status, param->start.service_handle);
            break;
        }
        case ESP_GATTS_STOP_EVT:
        {
            break;
        }
        case ESP_GATTS_CONNECT_EVT:
        {
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20; // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10; // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;  // timeout = 400*10ms = 4000ms
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                    param->connect.conn_id,
                    param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                    param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
            gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
            // start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            break;
        }
        case ESP_GATTS_DISCONNECT_EVT:
        {
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
            break;
        }
        case ESP_GATTS_CONF_EVT:
        {
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT, status %d attr_handle %d", param->conf.status, param->conf.handle);
            if (param->conf.status != ESP_GATT_OK)
            {
                esp_log_buffer_hex(GATTS_TAG, param->conf.value, param->conf.len);
            }
            break;
        }
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        default:
            break;
        }
    }

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        }
        else
        {
            ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d\n",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do
    {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++)
        {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                gatts_if == gl_profile_tab[idx].gatts_if)
            {
                if (gl_profile_tab[idx].gatts_cb)
                {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

static void _sendPosition()
{
    static uint16_t counter = 0;
    static GSMTxReq_t gsmTx;

    gsmTx.payload.header = 0xD7;
    uint64_t serial = 0;
    serial = (uint64_t) 109*100000000 + m_config.rom.loraId;
    gsmTx.payload.serialNumber[0] = (serial >> 32) & 0xFF;
    gsmTx.payload.serialNumber[1] = (serial >> 24) & 0xFF;
    gsmTx.payload.serialNumber[2] = (serial >> 16) & 0xFF;
    gsmTx.payload.serialNumber[3] = (serial >> 8) & 0xFF;
    gsmTx.payload.serialNumber[4] = (serial) & 0xFF;
    gsmTx.payload.fw[0] = 0xDD;
    gsmTx.payload.fw[1] = 0xEE;
    gsmTx.payload.hw = m_config.rom.hwVer;
    gsmTx.payload.protocol = 0xCC;
    gsmTx.payload.counter[0] = (counter >> 8) & 0xFF;
    gsmTx.payload.counter[1] = counter & 0xFF;
    m_config.status.GSMCount = counter;
    if(counter == 0xFFFF)
        counter = 0;
    else
        counter++;
    struct timeval current;
    gettimeofday(&current, NULL);
    gsmTx.payload.timestamp[0] = (current.tv_sec >> 24) & 0xFF;
    gsmTx.payload.timestamp[1] = (current.tv_sec >> 16) & 0xFF;
    gsmTx.payload.timestamp[2] = (current.tv_sec >> 8) & 0xFF;
    gsmTx.payload.timestamp[3] = (current.tv_sec) & 0xFF;
    
    
    gsmTx.payload.type = 0x00;
    gsmTx.payload.loraID[0] = (m_config.rom.loraId >> 16) & 0xff;
    gsmTx.payload.loraID[1] = (m_config.rom.loraId >> 8) & 0xff;
    gsmTx.payload.loraID[2] = (m_config.rom.loraId) & 0xff;
    memcpy(gsmTx.payload.imei, m_config.rom.imei, sizeof(m_config.rom.imei));
    gsmTx.payload.temp = m_config.status.temperatureCelsius;
    gsmTx.payload.batteryVoltage[0] = (m_config.status.batteryMiliVolts >> 8) & 0xff;
    gsmTx.payload.batteryVoltage[1] = (m_config.status.batteryMiliVolts) & 0xff;
    gsmTx.payload.flags.asBit.bt = m_config.status.flags.asBit.bleStatus;
    gsmTx.payload.flags.asBit.emergency = m_config.status.flags.asBit.emergency;
    gsmTx.payload.flags.asBit.input = m_config.status.flags.asBit.input;
    gsmTx.payload.flags.asBit.jammerGsm = m_config.status.flags.asBit.jammer;
    gsmTx.payload.flags.asBit.jammerLoRa = m_config.status.flags.asBit.jammer;
    gsmTx.payload.flags.asBit.lowBattery = m_config.status.flags.asBit.lowBattery;
    gsmTx.payload.flags.asBit.movement = m_config.status.flags.asBit.movement;
    gsmTx.payload.flags.asBit.output = m_config.status.flags.asBit.output;
    gsmTx.payload.flags.asBit.statusBattery = m_config.status.flags.asBit.statusBattery;
    gsmTx.payload.flags.asBit.stockMode = m_config.status.flags.asBit.stockMode;
    gsmTx.payload.lastRst = 0x00;
    memcpy(&gsmTx.config.apn, m_config.config.gsm.apn, strlen(m_config.config.gsm.apn));
    gsmTx.config.port = m_config.config.gsm.port;
    memcpy(&gsmTx.config.server, m_config.config.gsm.server, strlen(m_config.config.gsm.server));
    esp_event_post(APP_EVENT, APP_EVENT_GSM_TX_REQ, &gsmTx, sizeof(GSMTxReq_t), 0);
}

static void app_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if(event_base == APP_EVENT && event_id == APP_EVENT_GSM_TX_RES)
    {
        GSMTxRes_t *gsmTxRes_p = (GSMTxRes_t*) event_data;
        memcpy(&gsmTxRes, gsmTxRes_p, sizeof(GSMTxRes_t));
        xTaskNotify(xTaskToNotify, 0x200, eSetBits);
    }
    if(event_base == APP_EVENT && event_id == APP_EVENT_JIGA)
    {
        xTaskNotify(m_otp_task, 0, eSetValueWithOverwrite);
    }
}

void otp_task(void *parameter)
{
    uint32_t event = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    printf("\r\n**********JIGA ISCA OTP BEGIN**********\r\n");

    vTaskDelete(m_ble_task);

    uint32_t idx = 0;

    while (!is_otp_ok)
    {
        while (Serial.available() > 0)
        {
            temp = incomingByte;
            incomingByte = Serial.read();
            rcv[idx] = incomingByte;
            Serial.println(incomingByte);
            idx++;
        }
        if (temp == 0xF0 && incomingByte == 0xFF)
        {
            if (rcv[0] == 0xA0 && rcv[1] == 0xAF && rcv[60] == 0xF0 && rcv[61] == 0xFF)
            {
                is_otp_ok = true;
            }
            else
            {
                // restart
                temp = 0;
                incomingByte = 0;
                idx = 0;
                memset(rcv, 0, 62);
            }
        }
        else
        {
            temp = 0;
            incomingByte = 0;
            idx = 0;
            memset(rcv, 0, 62);
        }
        vTaskDelay(100);
    }

    printf("\r\n**********JIGA ISCA OTP RECEIVED**********\r\n");

    EEPROM::EEPROM *dev = new EEPROM::EEPROM();

    i2c_port_t port = I2C_NUM_0;
    int sda = CONFIG_SDA_GPIO;
    int scl = CONFIG_SCL_GPIO;
    int wp = CONFIG_WP_GPIO;

    ESP_ERROR_CHECK(dev->init(port, sda, scl, wp, true));

    // erase entire memory
    uint8_t data = 0xFF;
    for (uint16_t data_addr = 0; data_addr < AT24C02D_MEMORY_SIZE; data_addr++)
    {
        dev->at24c02d_write_byte(data_addr, &data, 1);
    }

    esp_err_t err = ESP_OK;

    otp.asParam.memVer = rcv[2];
    otp.asParam.hwVer = rcv[3];
    otp.asParam.prefixSN = rcv[4];
    memcpy(otp.asParam.loraID, &rcv[5], 3);
    memcpy(otp.asParam.devAddr, &rcv[8], 4);
    memcpy(otp.asParam.devEUI, &rcv[12], 8);
    memcpy(otp.asParam.appEUI, &rcv[20], 8);
    memcpy(otp.asParam.nwSKey, &rcv[28], 16);
    memcpy(otp.asParam.appSKey, &rcv[44], 16);

    ESP_LOG_BUFFER_HEX("otp", otp.asArray, sizeof(OTPMemory_t)); // size 71

    uint8_t otpCrc[82];
    memset(otpCrc, 0, 82);

    for (int i = 0; i < 11; i++)
    {
        if (i == 10)
        {
            otpCrc[80] = otp.asArray[70];
            otpCrc[81] = dallas_crc8(&otp.asArray[70], 1);
            printf("otpCrc = %d", otpCrc[81]);
        }
        else
        {
            memcpy(&otpCrc[i * 8], &otp.asArray[i * 7], 7);
            otpCrc[(i + 1) * 7 + i] = dallas_crc8(&(otp.asArray[i * 7]), 7);
            printf("otpCrc = %d", dallas_crc8(&(otp.asArray[i * 7]), 7));
        }
        ESP_LOG_BUFFER_HEX("otp_left", &otp.asArray[i * 7], 7); // size 71
    }
    ESP_LOG_BUFFER_HEX("otpCrc", otpCrc, 82);

    // write otp with crc every 7 bytes
    err = dev->at24c02d_write_byte(0x00, otpCrc, 82);

    uint8_t temp256[256];
    err = dev->at24c02d_read_byte(0x00, temp256, 256);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "at24c02d_read_byte fail %s", esp_err_to_name(err));
    }
    ESP_LOG_BUFFER_HEX("temp256", temp256, sizeof(temp256));

    dev->deinit();

    printf("\r\n**********JIGA ISCA OTP DONE**********\r\n");

    vTaskDelete(NULL);
}

void acc_task(void *parameter)
{
    uint32_t event = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    printf("\r\n**********JIGA ISCA ACC BEGIN**********\r\n");

    vTaskDelete(m_otp_task);

    uint32_t acc_unchanged = 0;
    uint32_t acc_zeroed = 0;
    uint32_t temp_zeroed = 0;
    uint32_t temp_wrong = 0;

    LIS2DW12Sensor acc(I2C_NUM_0, GPIO_NUM_27, GPIO_NUM_26);
    acc.begin();
    acc.Enable_X();

    acc.Get_X_Axes(m_config.status.acc);
    acc.Get_Temperature(&m_config.status.temperatureFloat);

    int32_t acc_x = m_config.status.acc[0];
    int32_t acc_y = m_config.status.acc[1];
    int32_t acc_z = m_config.status.acc[2];
    float acc_temp = m_config.status.temperatureFloat;
    float acc_temp_mean = 0.0;

    for (int i = 0; i < 10; i++)
    {
        acc.Get_X_Axes(m_config.status.acc);
        acc.Get_Temperature(&m_config.status.temperatureFloat);

        // conferir se os valores medidos estão dentro da media
        if (m_config.status.acc[0] == acc_x && m_config.status.acc[1] == acc_y && m_config.status.acc[2] == acc_z)
            acc_unchanged++;

        if (m_config.status.acc[0] == 0 && m_config.status.acc[1] == 0 && m_config.status.acc[2] == 0)
            acc_zeroed++;

        // media
        if (m_config.status.temperatureFloat == 0)
            temp_zeroed++;
        else if (m_config.status.temperatureFloat > 40 && m_config.status.temperatureFloat < -10)
            temp_wrong++;
        else
            acc_temp_mean += m_config.status.temperatureFloat;

        ESP_LOGI(TAG, "for i = %d", i);
        ESP_LOGI(TAG, "[ACC](mg) x:%ld | y:%ld | z:%ld | [TEMP](oC) %02.2f",
                 m_config.status.acc[0], m_config.status.acc[1], m_config.status.acc[2], m_config.status.temperatureFloat);

        vTaskDelay(500);
    }

    if (acc_unchanged > 3)
    {
        ESP_LOGE(TAG, "ERROR: ACCELEROMETER AXIS NOT CHANGING (%ld times).", acc_unchanged);
    }

    if (acc_zeroed > 3)
    {
        ESP_LOGE(TAG, "ERROR: ACCELEROMETER AXIS RETURNED ZERO (%ld times).", acc_zeroed);
    }

    if (temp_wrong > 3)
    {
        ESP_LOGE(TAG, "ERROR: TEMPERATURE OUT OF RANGE (-10ºC, 40ºC) (%ld times).", temp_wrong);
    }

    if (temp_zeroed > 3)
    {
        ESP_LOGE(TAG, "ERROR: TEMPERATURE RETURNED ZERO (%ld times).", temp_zeroed);
    }

    printf("\r\n**********JIGA ISCA ACC DONE**********\r\n");

    xTaskNotify(m_ble_task, 0, eSetValueWithOverwrite);

    while (1)
        vTaskDelay(500);
}

void ble_task(void *parameter)
{
    // uint32_t event = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    printf("\r\n**********JIGA ISCA BLE BEGIN**********\r\n");

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret)
    {
        ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    uint8_t mac[6];
    ret = esp_efuse_mac_get_default(mac);
    if (ret == ESP_OK) 
    {
        Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                    mac[0], mac[1], mac[2],
                    mac[3], mac[4], mac[5]);

        memcpy(otp.asParam.bleMac, mac, 6);
    } 
    else     
    {
        Serial.println("Failed to read MAC address");
    }

    while (1)
        vTaskDelay(500);
}

void serial_receive_gsm_port()
{
    uint32_t idx = 0;

    while (!is_gsm_port_ok)
    {
        while (Serial.available() > 0)
        {
            temp = incomingByte;
            incomingByte = Serial.read();
            rcvServerPort[idx] = incomingByte;
            Serial.println(incomingByte);
            idx++;
        }
        if (temp == 0xF0 && incomingByte == 0xFF)
        {
            if (rcvServerPort[0] == 0xA0 && rcvServerPort[1] == 0xAF && rcvServerPort[4] == 0xF0 && rcvServerPort[5] == 0xFF)
            {
                ngrok_port = (rcvServerPort[2] << 8) + rcvServerPort[3];
                is_gsm_port_ok = true;
            }
            else
            {
                // restart
                temp = 0;
                incomingByte = 0;
                idx = 0;
                memset(rcvServerPort, 0, 6);
            }
        }
        else
        {
            temp = 0;
            incomingByte = 0;
            idx = 0;
            memset(rcvServerPort, 0, 6);
        }
        vTaskDelay(100);
    }
}

void r800c_task(void *parameter)
{
    // uint32_t event = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    printf("\r\n**********JIGA ISCA R800C BEGIN**********\r\n");

    serial_receive_gsm_port();
    m_config.config.gsm.port = ngrok_port;
    // m_config.config.gsm.port = 11611;

    printf("\r\n**********JIGA ISCA R800C GSM PORT RECEIVED**********\r\n");

    esp_err_t err = getImei(otp.asParam.imei);

    if (err == ESP_OK)
    {
        char print[30];
        for(int k = 0; k < sizeof(otp.asParam.imei); k++)
        {
            sprintf(print+k*3, "%02X ", otp.asParam.imei[k]);   
        }
        print[21] = 0x00;
        uint64_t imei;
        imei = 0L;
        imei += (((uint64_t) otp.asParam.imei[0])<<48);
        imei += (((uint64_t) otp.asParam.imei[1])<<40);
        imei += (((uint64_t) otp.asParam.imei[2])<<32);
        imei += (((uint64_t) otp.asParam.imei[3])<<24);
        imei += (((uint64_t) otp.asParam.imei[4])<<16);
        imei += (((uint64_t) otp.asParam.imei[5])<<8);
        imei += (((uint64_t) otp.asParam.imei[6]));
        // memcpy(otp.asParam.imei, otp.asParam.imei, 7);
        printf("imei: %lld | %s\r\n", imei, print);

        uint32_t count = 0;
        uint32_t ulNotifiedValue = 0;

        xTaskToNotify = xTaskGetCurrentTaskHandle();

        esp_event_handler_instance_register(APP_EVENT, ESP_EVENT_ANY_ID, &app_event_handler, nullptr, nullptr);

        xTaskCreate(gsmTask, "gsmTask", 8192, (void*) &m_config, uxTaskPriorityGet(NULL), NULL);
    }
    else
    {
        Serial.println("ERROR: R800C FAILED TO GET imei");
    }

    vTaskDelete(NULL);
}

void lora_task(void *parameter)
{
    uint32_t event = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    printf("\r\n**********JIGA ISCA LORA BEGIN**********\r\n");

    vTaskDelete(m_r800c_task);

    while (1)
    {
        vTaskDelay(500);
    }

    printf("\r\n**********JIGA ISCA LORA BEGIN**********\r\n");
}

// serial waits for gsm port before starting everything
// test acc
// test gsm -> show results in ble read
// write otp -> show results in ble read
// stop test when read ble char test bit shows done
void setup()
{
    Serial.begin(115200);

    printf("\r\n**********JIGA ISCA BEGIN**********\r\n");

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    BiColorStatus::init();
    BiColorStatus::turnOn();

    OTPMemory_t hardCodedROM = {
                    .memVer = 0xDD,
                    .hwVer = 0xAA,
                    .prefixSN = 0x6D,
                    .loraID = {0xbe, 0xbc, 0x25},
                    .devAddr = {0x5d, 0x1f, 0x42, 0x61},
                    .devEUI = {0x34, 0xe5, 0x51, 0xdf, 0x6d, 0xab, 0xc5, 0x6b},
                    .appEUI = {0x2c, 0xa5, 0xae, 0x9d, 0xf4, 0xaf, 0x41, 0x4a},
                    .nwSKey = {0x45, 0x5d, 0xd8, 0xb2, 0x11, 0x65, 0x3e, 0x5c, 0xbd, 0xbf, 0xb1, 0x29, 0xa3, 0x29, 0xb2, 0x99},
                    .appSKey = {0x5e, 0xc0, 0x07, 0x25, 0x72, 0x47, 0xf1, 0xb0, 0x9a, 0xad, 0x8f, 0x61, 0xe8, 0xca, 0x4d, 0xd1}
    };
    memcpy(&readMemory, &hardCodedROM, sizeof(OTPMemory_t));

    memset(&m_config, 0, sizeof(m_config));
    m_config.rom.loraId = (readMemory.loraID[0]<<16) + (readMemory.loraID[1]<<8) + (readMemory.loraID[2]);
    memcpy(m_config.rom.deviceEUI, readMemory.devEUI, 8);
    memcpy(m_config.rom.appEUI, readMemory.appEUI, 8);
    m_config.rom.devAddr = (readMemory.devAddr[0]<<24) + (readMemory.devAddr[1]<<16) +
        (readMemory.devAddr[2]<<8) + readMemory.devAddr[3];
    memcpy(m_config.rom.nwkSKey, readMemory.nwSKey, 16);
    memcpy(m_config.rom.appSKey, readMemory.appSKey, 16);
    
    printf("loraID: %ld | devAddress: %08lX\r\n", m_config.rom.loraId,m_config.rom.devAddr);
    ESP_LOG_BUFFER_HEX("deviceEUI", m_config.rom.deviceEUI, sizeof(m_config.rom.deviceEUI));
    ESP_LOG_BUFFER_HEX("appEUI", m_config.rom.appEUI, sizeof(m_config.rom.appEUI));
    ESP_LOG_BUFFER_HEX("nwSKey", m_config.rom.nwkSKey, sizeof(m_config.rom.nwkSKey));
    ESP_LOG_BUFFER_HEX("appSKey", m_config.rom.appSKey, sizeof(m_config.rom.appSKey));

    m_config.config.p2p.sf = P2P_SPREADING_FACTOR;
    m_config.config.p2p.bw = P2P_BANDWIDTH;
    m_config.config.p2p.cr = 1;
    m_config.config.p2p.txFreq = P2P_POS_FREQ;
    m_config.config.p2p.txPower = P2P_TX_POWER;
    m_config.config.p2p.rxFreq = P2P_CMD_FREQ;
    m_config.config.p2p.rxDelay = 0;
    m_config.config.p2p.rxTimeout = P2P_RX_TIMEOUT;
    m_config.config.p2p.txTimeout = P2P_TX_TIMEOUT;

    m_config.config.lrw.confirmed = false;
    m_config.config.lrw.posPort = LRW_POS_PORT;
    m_config.config.lrw.cmdPort = LRW_CMD_PORT;
    m_config.config.lrw.statusPort = LRW_STATUS_PORT;
    m_config.config.lrw.adr = false;

    ESP_ERROR_CHECK(esp_efuse_mac_get_default(mac));
    ESP_ERROR_CHECK(esp_base_mac_addr_set(mac));
    ESP_LOGW(TAG, "MAC: " MACSTR " ", MAC2STR(mac));

    memcpy(&m_config.rom.bleMac, mac, sizeof(m_config.rom.bleMac));

    memcpy(&m_config.config.gsm.apn, GSM_APN, strlen(GSM_APN));
    memcpy(&m_config.config.gsm.server, NGROK_SERVER, strlen(NGROK_SERVER));

    // xTaskCreate(acc_task, "acc_task", 4096, (void *)&m_config, 5, &m_acc_task);
    xTaskCreate(r800c_task, "r800c_task", 4096, (void *)&m_config, 5, &m_r800c_task);
    xTaskCreate(ble_task, "ble_task", 4096, (void *)&m_config, 5, &m_ble_task);
    xTaskCreate(otp_task, "otp_task", 4096, (void *)&m_config, 5, &m_otp_task);
}

void loop()
{
    vTaskDelay(100);
}

// 0A AF 12 AB 0C C6 5D BB 80 4C 28 DE A1 4E 2A 60 84 4C 94 97 7D 60 EE AF C9 EE 72 BD 75 58 87 5D A2 C8 5C FB 41 14 F3 FA 2A A1 9D CD 15 64 D6 74 27 82 84 89 D8 B1 E7 7A 6B 59 00 53 F0 FF
// 12 
// AB 
// 0C 
// C6 5D BB 
// 80 4C 28 DE 
// A1 4E 2A 60 84 4C 94 97 
// 7D 60 EE AF C9 EE 72 BD 
// 75 58 87 5D A2 C8 5C FB 41 14 F3 FA 2A A1 9D CD 
// 15 64 D6 74 27 82 84 89 D8 B1 E7 7A 6B 59 00 53

// jiga isca ordem
// 1. acelerometro
// 2. modem
// 3. ble
// 4. otp