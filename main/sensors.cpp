#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "main.h"
#include "sensors.hpp"
#include "stateMachine.hpp"

#include "LIS2DW12Sensor.h"
#include <Wire.h>

static Isca_t *m_config = NULL;

static int adc_raw[3];
static int voltage[3];
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void adc_calibration_deinit(adc_cali_handle_t handle);

static adc_cali_handle_t adc1_cali_chan0_handle = NULL;
static adc_cali_handle_t adc1_cali_chan2_handle = NULL;
static adc_cali_handle_t adc1_cali_chan4_handle = NULL;

static SensorsStatus_t _sensorStatus;
static int64_t _accCheckTimeout = 0, _statusTimeout = 0;
static int32_t _lastAcc[3], _acc[3];
static bool _movement = false;
static uint8_t _batStatus;
static float _temperature;

#define ACC_MG_DIFF 200
volatile bool accIntFlag = false;
esp_timer_handle_t debounceMovingTimer, debounceStoppingTimer;
static int64_t timeLastAccChanged, timerStopping;
static const char* TAG = "SENSORS";


TwoWire dev_i2c(0);  
LIS2DW12Sensor Acc(&dev_i2c, LIS2DW12_I2C_ADD_L);

static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void adc_calibration_deinit(adc_cali_handle_t handle)
{
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
}

static void app_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
}

const char* printBatStatus(SensorsBatStatus_t type)
{
    switch(type)
    {
        case BAT_ABSENT:
            return "BAT_ABSENT";
        case BAT_DISCHARGING:
            return "BAT_DISCHARGING";
        case BAT_CHARGING:
            return "BAT_CHARGING";
        case BAT_CHARGED:
            return "BAT_CHARGED";
        default:
            break;
    }
    return "BAT_ERROR";
}

static void debounceStoppingCallback(void *args)
{
    _movement = false;
    printf("        [ACC] I am stopped\r\n");
    esp_event_post(APP_EVENT, APP_EVENT_MOVEMENT, (void*)&_movement, sizeof(_movement), 0);
}


static void debounceMovingCallback(void *args)
{
    printf("        [ACC] debounceMovingTimeout\r\n");
    if(esp_timer_get_time() - timeLastAccChanged < (DEBOUNCE_MOVING*1000) >>2)
    {
        _movement = true;
        printf("        [ACC] I am moving\r\n");
        esp_event_post(APP_EVENT, APP_EVENT_MOVEMENT, (void*)&_movement, sizeof(_movement), 0);
    }
}

void sensorsTask(void* parameter)
{
    m_config = (Isca_t*) parameter;

    esp_event_handler_instance_register(APP_EVENT, ESP_EVENT_ANY_ID, &app_event_handler, nullptr, nullptr);
    
    const esp_timer_create_args_t debounceMovingArgs = {
        .callback = &debounceMovingCallback,
        .name = "debounceMoving"
    };

    const esp_timer_create_args_t debounceStoppingArgs = {
            .callback = &debounceStoppingCallback,
            .name = "debounceStopping"
    };

    ESP_ERROR_CHECK(esp_timer_create(&debounceMovingArgs, &debounceMovingTimer));
    ESP_ERROR_CHECK(esp_timer_create(&debounceStoppingArgs, &debounceStoppingTimer));

    dev_i2c.begin(PIN_NUM_SDA, PIN_NUM_SCL, 100000);
    Acc.begin();
    Acc.Enable_X();

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << PIN_BAT_ADC_CTRL,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config;
    config.bitwidth = ADC_BITWIDTH_DEFAULT;
    config.atten = ADC_ATTEN_DB_12;

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_2, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &config));

    //-------------ADC1 Calibration Init---------------//
   
    bool do_calibration1_chan0 = adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_0, ADC_ATTEN_DB_12, &adc1_cali_chan0_handle);
    bool do_calibration1_chan2 = adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_2, ADC_ATTEN_DB_12, &adc1_cali_chan2_handle);
    bool do_calibration1_chan4 = adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_4, ADC_ATTEN_DB_12, &adc1_cali_chan4_handle);
    char printLog[3][40] = {'0'};
    _accCheckTimeout = (-1)*SENSORS_ACC_CHECK_TIMEOUT_MS*1000;
    _statusTimeout = (-1)*SENSORS_STATUS_TIMEOUT_MS*1000;
    while(1)
    {
        int64_t now = esp_timer_get_time();

        if (accIntFlag)
        {
            accIntFlag = false;
            timeLastAccChanged = esp_timer_get_time();

            if(_movement == false && !esp_timer_is_active(debounceMovingTimer))
            {
                printf("        [ACC] debounceMovingStart\r\n");
                esp_timer_start_once(debounceMovingTimer,DEBOUNCE_MOVING*1000 );
            }
            if(_movement)
            {
                if(esp_timer_is_active(debounceStoppingTimer))
                {
                    esp_timer_stop(debounceStoppingTimer);
                }
                
                esp_timer_start_once(debounceStoppingTimer,DEBOUNCE_STOP*1000 );
            }
        }
        if(now - timerStopping > 10000*1000)
        {
            timerStopping = now;
            if(esp_timer_is_active(debounceStoppingTimer))
            {
                uint64_t expiry = 0;
                esp_timer_get_expiry_time(debounceStoppingTimer, &expiry);
                printf("        [ACC] debounceStoppingTimer left: %llus\r\n", (expiry-now)/1000000);
            }
        }

        if( now - _accCheckTimeout > SENSORS_ACC_CHECK_TIMEOUT_MS*1000)
        {
            Acc.Get_X_Axes(_acc);
            _accCheckTimeout = now;
            int32_t diffAcc[3];
            diffAcc[0] = (_lastAcc[0] >= _acc[0])?(_lastAcc[0] - _acc[0]):(_acc[0] - _lastAcc[0]);
            diffAcc[1] = (_lastAcc[1] >= _acc[1])?(_lastAcc[1] - _acc[1]):(_acc[1] - _lastAcc[1]);
            diffAcc[2] = (_lastAcc[2] >= _acc[2])?(_lastAcc[2] - _acc[2]):(_acc[2] - _lastAcc[2]);
            if((diffAcc[0] > ACC_MG_DIFF) ||  (diffAcc[1] > ACC_MG_DIFF) || (diffAcc[2] > ACC_MG_DIFF))
            {
                accIntFlag = true;
                ESP_LOGW(TAG, "[ACC]-------- accIntFlag from %d m/s^2 change detection --------\r\n", ACC_MG_DIFF);
            }
            _lastAcc[0] = _acc[0];
            _lastAcc[1] = _acc[1];
            _lastAcc[2] = _acc[2];
        }

        if(now - _statusTimeout > SENSORS_STATUS_TIMEOUT_MS*1000)
        {
            _statusTimeout = now;

            gpio_set_direction(PIN_BAT_ADC_CTRL, GPIO_MODE_OUTPUT);
            gpio_set_level(PIN_BAT_ADC_CTRL, 0);
            vTaskDelay(pdMS_TO_TICKS(200));

            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &adc_raw[0]));
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_2, &adc_raw[1]));
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &adc_raw[2]));
            
            gpio_set_direction(PIN_BAT_ADC_CTRL, GPIO_MODE_INPUT);

            if (do_calibration1_chan0) 
            {
                ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0], &voltage[0]));
                sprintf(&printLog[0][0], "[BAT_ADC]: %d mV", 2*voltage[0]);
            }
            else
            {
                sprintf(&printLog[0][0], "BAT_ADC: 0x%03X", adc_raw[0]);
            }


            if (do_calibration1_chan2) 
            {
                ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan2_handle, adc_raw[1], &voltage[1]));
                sprintf(&printLog[1][0], "[BMS_CHR]: %d mV", voltage[1]);
            }
            else
            {
                sprintf(&printLog[1][0], "[BMS_CHR]: 0x%03X", adc_raw[0]);
            }

            if (do_calibration1_chan4) 
            {
                ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan4_handle, adc_raw[2], &voltage[2]));
                sprintf(&printLog[2][0], "[BMS_DON]: %d mV", voltage[2]);
            }
            else
            {
                sprintf(&printLog[2][0], "[BMS_DON]: 0x%03X", adc_raw[2]);
            }
            
            // ESP_LOGI(TAG, "%s %s %s", printLog[0], printLog[1], printLog[2]);
            
            if(2*voltage[0] < 1000)
            {
                _sensorStatus.batStatus = BAT_ABSENT;
            }
            else if(voltage[1] > 1800 && voltage[2] > 1800)
            {
                _sensorStatus.batStatus = BAT_DISCHARGING;
            }
            else if(voltage[1] < 1800)
            {
                _sensorStatus.batStatus = BAT_CHARGING;
            }
            else
            {
                _sensorStatus.batStatus = BAT_CHARGED;
            }
            
            Acc.Get_Temperature(&_temperature);

            _sensorStatus.temperature = _temperature;
            _sensorStatus.batVoltage = 2*voltage[0];
            memcpy(&_sensorStatus.acc, _acc, sizeof(_sensorStatus.acc));

            // ESP_LOGW(TAG, "[ACC](mg) x:%ld | y:%ld | z:%ld | [TEMP](oC) %02.2f | %s %s %s | batStatus:%s", 
            //         _acc[0], _acc[1], _acc[2], _temperature,
            //         printLog[0], printLog[1], printLog[2], printBatStatus((SensorsBatStatus_t)_sensorStatus.batStatus));
            
            esp_event_post(APP_EVENT, APP_EVENT_SENSORS_STATUS, &_sensorStatus, sizeof(SensorsStatus_t), 0);
        }
        // vTaskDelay(pdMS_TO_TICKS(60000));
    }

    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));

    if (do_calibration1_chan0) 
        adc_calibration_deinit(adc1_cali_chan0_handle);

    if (do_calibration1_chan2) 
        adc_calibration_deinit(adc1_cali_chan2_handle);

    if (do_calibration1_chan4) 
        adc_calibration_deinit(adc1_cali_chan4_handle);

}