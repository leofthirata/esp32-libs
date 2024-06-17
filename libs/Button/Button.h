#ifndef MAIN_BUTTONTASK_H_
#define MAIN_BUTTONTASK_H_

#ifdef __cplusplus
extern "C" {
#endif
/* GPIO 19, 22, 24, 34 not recommended to use as GPIO_BUTTON_X.
 * experienced endless isr gpio triggering, for no reason.
 * */

#define ESP_INTR_FLAG_DEFAULT 0

#define BUTTON_DEBOUNCE_US (50000ULL)
#define BUTTON_HOLD_TIME_US (2000000ULL)

#include <stdint.h>
#include "esp_timer.h"
#include "driver/gpio.h"

typedef enum
{
    BTN_PRESSED = 0,  /// Map to active low (0).
    BTN_RELEASED = 1, /// Map to  active high (1).
    BTN_HOLD = 2      /// Special case for when the button is held pressed.
} buttonState_t;

typedef struct {
	uint8_t buttonPin;
	buttonState_t buttonState;
	buttonState_t buttonPrev;
}button_queue_t;

typedef void (*buttonHandler)(button_queue_t);

typedef struct {
	esp_timer_handle_t m_debounce_timer; // Debounce timer.
	esp_timer_handle_t m_hold_timer;     // Timer used to detect HOLD events.
	bool m_isReady;
	bool m_isHold;
	uint8_t state;
	uint8_t pin;
	buttonHandler event_handler;
} button_param_t;

typedef struct {
	buttonHandler buttonEventHandler;
	uint64_t pin_bit_mask;
	gpio_pullup_t pull_up_en;       /*!< GPIO pull-up                                         */
    gpio_pulldown_t pull_down_en;   /*!< GPIO pull-down                                       */
}button_init_t;


esp_err_t buttonInit(button_init_t*);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_BUTTONTASK_H_ */