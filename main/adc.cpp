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

static int adc_raw[1][3];
static int voltage[1][3];
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void adc_calibration_deinit(adc_cali_handle_t handle);

static adc_cali_handle_t adc1_cali_chan0_handle = NULL;
static adc_cali_handle_t adc1_cali_chan2_handle = NULL;
static adc_cali_handle_t adc1_cali_chan4_handle = NULL;

static const char* TAG = "ADC";

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


void adcTask(void* parameter)
{
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
    
    while(1)
    {
        gpio_set_direction(PIN_BAT_ADC_CTRL, GPIO_MODE_OUTPUT);
        gpio_set_level(PIN_BAT_ADC_CTRL, 0);
        vTaskDelay(200);

        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &adc_raw[0][0]));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_2, &adc_raw[0][1]));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &adc_raw[0][2]));
        
        gpio_set_direction(PIN_BAT_ADC_CTRL, GPIO_MODE_INPUT);

       if (do_calibration1_chan0) 
        {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0][0], &voltage[0][0]));
            sprintf(&printLog[0][0], "[BAT_ADC]: %d | %d mV", adc_raw[0][0], 2*voltage[0][0]);
        }
        else
        {
            sprintf(&printLog[0][0], "BAT_ADC: %d", adc_raw[0][0]);
        }


        if (do_calibration1_chan2) 
        {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan2_handle, adc_raw[0][1], &voltage[0][1]));
            sprintf(&printLog[1][0], "[BMS_CHR]: %d | %d mV", adc_raw[0][1], voltage[0][1]);
        }
        else
        {
            sprintf(&printLog[1][0], "[BMS_CHR]: %d", adc_raw[0][1]);
        }


        if (do_calibration1_chan4) 
        {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan4_handle, adc_raw[0][2], &voltage[0][2]));
            sprintf(&printLog[2][0], "[BMS_DON]: %d | %d mV", adc_raw[0][2], voltage[0][2]);
        }
        else
        {
            sprintf(&printLog[2][0], "[BMS_DON]: %d", adc_raw[0][2]);
        }
        ESP_LOGI(TAG, "%s %s %s", printLog[0], printLog[1], printLog[2]);
        vTaskDelay(pdMS_TO_TICKS(5000));

    }

    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));

    if (do_calibration1_chan0) 
        adc_calibration_deinit(adc1_cali_chan0_handle);

    if (do_calibration1_chan2) 
        adc_calibration_deinit(adc1_cali_chan2_handle);

    if (do_calibration1_chan4) 
        adc_calibration_deinit(adc1_cali_chan4_handle);

}