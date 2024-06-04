#pragma once

#include <cstdint>

#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

/**
 * @brief LedBicolor class.
 */
class LedBicolor
{

public:
    /**
     * @brief Led state.
     */
    enum class State
    {
        STATIC, /// The led is static.
        BLINK,  /// The led is blinking.
    };

    /**
     * @brief Object constructor.
     */
    LedBicolor();

    /**
     * @brief Object destructor
     */
    ~LedBicolor();

    /**
     * @brief Initialize the led driver.
     *
     * @param[in] r Red led GPIO pin number.
     * @param[in] g Green led GPIO pin number.
     *
     * @return
     *      - ESP_OK Success.
     *      - ESP_ERR_INVALID_ARG Invalid GPIO number.
     *      - ESP_ERR_NO_MEM Not enough heap memory.
     *      - ESP_FAIL Failed.
     */
    esp_err_t begin(gpio_num_t r, gpio_num_t g);

    /**
     * @brief Get current RG color.
     *
     * @return Current led color.
     */
    uint16_t color();

    /**
     * @brief Set current led color.
     *
     * @param[in] rg New color.
     */
    void color(uint16_t rg);

    /**
     * @brief Push new color to the led, backing up the previus state.
     *
     * @param[in] rg New color.
     */
    void push(uint16_t rg);

    /**
     * @brief Pop last color from the led.
     */
    void pop();

    /**
     * @brief Blink the led.
     *
     * @param[in] rg New color.
     * @param[in] period_ms Blink period.
     */
    void blink(uint16_t rg, uint32_t period_ms);

    /**
     * @brief Pulse the desired @p state for @p duration_ms miliseconds.
     *
     * @param[in] rg Color to pulse.
     * @param[in] state Type of pulse (STATIC or BLINK).
     * @param[in] duration_ms Pulse duration.
     * @param[in] period_ms Blink period when state is BLINK.
     */
    void pulse(uint16_t rg, State state, uint32_t duration_ms,
               uint32_t period_ms);

private:
    uint32_t m_pins[2]; // GPIO pins [0] is Red, [1] is Green and [2] is Blue.

    bool m_on;             // Is ON flag.
    uint32_t m_color;      // Current color.
    uint32_t m_prevColor;  // Previous color.
    State m_state;         // Current state.
    State m_prevState;     // Previous state.
    uint32_t m_period;     // Current period.
    uint32_t m_prevPeriod; // Previous period.

    esp_timer_handle_t m_blinkTimer; // Blink timer handler.
    esp_timer_handle_t m_pulseTimer; // Pulse timer handler.

    SemaphoreHandle_t m_mutex; // Multi-thread lock mutex.
    /**

     * @brief Create PWM channels.
     */
    void _pwmCreate();

    /**
     * @brief Destroy the PWM channels.
     */
    void _pwmDestroy();

    /**
     * @brief Set PWM duty for each color channel.
     *
     * @param[in] rg New color.
     */
    void _pwmSetDuty(uint16_t rg);

    /**
     * @brief Set color (bypass mutex).
     *
     * @param[in] rg New color;
     */
    void _color(uint16_t rg);

    /**
     * @brief Blink led (bypass mutex).
     *
     * @param[in] rg New color.
     * @param[in] period_ms Blink period.
     */
    void _blink(uint16_t rg, uint32_t period_ms);

    /**
     * @brief Lock mutex.
     */
    void lock();

    /**
     * @brief Unlock mutex.
     */
    void unlock();

    /**
     * @brief Blink timer callback function.
     *
     * @param[in] param Pointer to the LedBicolor object.
     */
    friend void bicolor_blink_timer(void *param);

    /**
     * @brief Pulse timer callback function.
     *
     * @param[in] param Pointer to the LedBicolor object.
     */
    friend void bicolor_pulse_timer(void *param);
};