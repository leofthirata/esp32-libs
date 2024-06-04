#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "Button.h"
#include "esp_err.h"

// Debounce duration in microseconds.
static const uint32_t debounceTime = BUTTON_DEBOUNCE_US;

// Default amount of time (in milliseconds) the button needs to stay pressed
// to emit an on_hold signal.
static const uint32_t holdTime = BUTTON_HOLD_TIME_US;

static QueueHandle_t button_evt_queue = NULL;
static button_param_t *buttonArray = NULL;

const static char *TAG = "button.c";

IRAM_ATTR void gpio_isr_handler (void* arg)
{
    button_param_t* gpio_buff = (button_param_t*) arg;

    if (gpio_buff->m_isReady) 
	{
    	gpio_buff->m_isReady = false;
    	esp_timer_start_once(gpio_buff->m_debounce_timer, debounceTime);
    }

}

static void hold_timer_callback(void *arg)
{
	button_param_t *e = (button_param_t*) arg;

	e->state = BTN_HOLD;
	xQueueSendToBackFromISR(button_evt_queue, e, NULL);
}

static  void debounce_timer_callback(void *arg)
{
	button_param_t *e = (button_param_t*) arg;

	e->state = ( gpio_get_level( e->pin) ) & 1U;
	xQueueSendToBackFromISR(button_evt_queue, e, NULL);

	e->m_isReady = true;
}

static void button_task(void* arg)
{
	button_param_t io_num;
	button_queue_t buttonInfo2cb;

	for (;;)
	{
		xQueueReceive(button_evt_queue, &io_num, portMAX_DELAY);

		switch (io_num.state) 
		{
			case BTN_PRESSED: 
			{
				buttonInfo2cb.buttonState = BTN_PRESSED;
				buttonInfo2cb.buttonPin = io_num.pin;
				(*io_num.event_handler)(buttonInfo2cb);
				esp_timer_start_once(io_num.m_hold_timer, holdTime);
				break;
			}
			case BTN_RELEASED: 
			{
				esp_timer_stop(io_num.m_hold_timer);

				if (io_num.m_isHold == false) 
				{
					buttonInfo2cb.buttonState = BTN_RELEASED;
					buttonInfo2cb.buttonPin = io_num.pin;
					(*io_num.event_handler)(buttonInfo2cb);
				}
				else 
				{
					io_num.m_isHold = false;
				}
				break;
			}
			case BTN_HOLD: 
			{
				io_num.m_isHold = true;
				buttonInfo2cb.buttonState = BTN_HOLD;
				buttonInfo2cb.buttonPin = io_num.pin;
				(*io_num.event_handler)(buttonInfo2cb);

				break;
			}

			default:
				break;
		}
	}
}

//Brian Kernighan’s Algorithm
//https://www.geeksforgeeks.org/count-set-bits-in-an-integer/
static unsigned int countSetBits(uint64_t n)
{
    unsigned int count = 0;
    while (n) 
	{
        n &= (n - 1);
        count++;
    }
    return count;
}

esp_err_t buttonInit(button_init_t* param)
{
    esp_timer_create_args_t debounceTimerArgs, holdTimerArgs;
    uint32_t io_num = 0, index = 0;
    
    if (param->pin_bit_mask == 0 ||
        param->pin_bit_mask & ~SOC_GPIO_VALID_GPIO_MASK)
    {
        ESP_LOGE(TAG, "GPIO_PIN mask error ");
        return ESP_ERR_INVALID_ARG;
    }

    buttonArray = (button_param_t*) malloc(countSetBits(param->pin_bit_mask)*sizeof(button_param_t));
    
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .pin_bit_mask = param->pin_bit_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = param->pull_up_en,
        .pull_down_en = param->pull_down_en
    };
    gpio_config(&io_conf);

    //create a queue to handle gpio event from isr
    button_evt_queue = xQueueCreate(10, sizeof(button_param_t));
    //start gpio task
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);

    //install gpio isr service
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

	debounceTimerArgs.callback = debounce_timer_callback;
	debounceTimerArgs.dispatch_method = ESP_TIMER_TASK;
	holdTimerArgs.callback = hold_timer_callback;
	holdTimerArgs.dispatch_method = ESP_TIMER_TASK;

	do
    {
       if (((param->pin_bit_mask >> io_num) & BIT(0))) 
       {
            (buttonArray + index)->m_isReady = true;
            (buttonArray + index)->m_isHold = false;
            (buttonArray + index)->state = 0;
            (buttonArray + index)->event_handler = param->buttonEventHandler;
            (buttonArray + index)->pin = io_num;

            //hook isr handler for specific gpio pin/button
            gpio_isr_handler_add((buttonArray + index)->pin, gpio_isr_handler, (void*)(buttonArray + index));
            
            debounceTimerArgs.arg = (void*)(buttonArray + index);
            ESP_ERROR_CHECK(esp_timer_create(&debounceTimerArgs, &((buttonArray + index)->m_debounce_timer)) );
            
            holdTimerArgs.arg = (void*)(buttonArray + index);
            ESP_ERROR_CHECK(esp_timer_create(&holdTimerArgs, &((buttonArray + index)->m_hold_timer)));

            index++;
       }
       io_num++;

    } while (io_num < GPIO_PIN_COUNT);
    
    return ESP_OK;
}