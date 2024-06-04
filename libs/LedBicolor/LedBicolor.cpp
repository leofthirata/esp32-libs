#include "LedBicolor.hpp"
#include "driver/ledc.h"

void bicolor_blink_timer(void *param)
{
    LedBicolor *l = static_cast<LedBicolor *>(param);

    if (l->m_on)
    {
        l->m_on = false;
        l->_pwmSetDuty(0x0000);
    }
    else
    {
        l->m_on = true;
        l->_pwmSetDuty(l->m_color);
    }
}

void bicolor_pulse_timer(void *param)
{
    LedBicolor *l = static_cast<LedBicolor *>(param);

    if (l->m_prevState == LedBicolor::State::STATIC)
    {
        l->_color(l->m_prevColor);
    }
    else
    {
        l->_blink(l->m_prevColor, l->m_prevPeriod);
    }

    xSemaphoreGiveFromISR(l->m_mutex, nullptr);
}

LedBicolor::LedBicolor()
{
    m_pins[0] = 0;
    m_pins[1] = 0;

    m_on = false;
    m_color = 0;
    m_prevColor = 0;
    m_state = State::STATIC;
    m_prevState = State::STATIC;
    m_period = 0;
    m_prevPeriod = 0;
    m_blinkTimer = nullptr;
    m_pulseTimer = nullptr;
    m_mutex = nullptr;
}

LedBicolor::~LedBicolor()
{
    if (m_blinkTimer)
    {
        esp_timer_stop(m_blinkTimer);
        esp_timer_delete(m_blinkTimer);
        m_blinkTimer = nullptr;
    }

    if (m_pulseTimer)
    {
        esp_timer_stop(m_pulseTimer);
        esp_timer_delete(m_pulseTimer);
        m_pulseTimer = nullptr;
    }

    _pwmDestroy();
}

esp_err_t LedBicolor::begin(gpio_num_t r, gpio_num_t g)
{
	m_mutex = xSemaphoreCreateBinary();
	unlock();

    esp_err_t rslt;

    esp_timer_create_args_t timer_args;

    timer_args.callback = bicolor_blink_timer;
    timer_args.arg = this;
    timer_args.name = "rgb";
    timer_args.dispatch_method = ESP_TIMER_TASK;

    rslt = esp_timer_create(&timer_args, &m_blinkTimer);

    if (rslt != ESP_OK)
        return rslt;

    timer_args.callback = bicolor_pulse_timer;
    timer_args.name = "rgp";

    rslt = esp_timer_create(&timer_args, &m_pulseTimer);

    if (rslt != ESP_OK)
        return rslt;

    m_pins[0] = (uint32_t)r;
    m_pins[1] = (uint32_t)g;

    _pwmCreate();

    m_color = 0xFFFF;

    color(m_color);

    vTaskDelay(pdMS_TO_TICKS(30));

    return ESP_OK;
}

uint16_t LedBicolor::color()
{
    return m_color;
}

void LedBicolor::color(uint16_t rg)
{
    lock();

    _color(rg);

    unlock();
}

void LedBicolor::push(uint16_t rg)
{
    lock();
    m_prevColor = m_color;
    m_prevState = m_state;
    m_prevPeriod = m_period;
    _color(rg);
}

void LedBicolor::pop()
{
    if (m_prevState == State::STATIC)
    {
        _color(m_prevColor);
    }
    else
    {
        _blink(m_prevColor, m_period);
    }

    unlock();
}

void LedBicolor::blink(uint16_t rg, uint32_t period_ms)
{
    lock();
    _blink(rg, period_ms);
    unlock();
}

void LedBicolor::pulse(uint16_t rg, State state, uint32_t duration_ms,
                   uint32_t period_ms)
{
    lock();

    m_prevColor = m_color;
    m_prevState = m_state;
    m_prevPeriod = m_period;

    if (state == State::STATIC)
        _color(rg);

    else
        _blink(rg, period_ms);


    esp_timer_start_once(m_pulseTimer, duration_ms * 1000);
}

void LedBicolor::_pwmCreate()
{
    ledc_timer_config_t timer_config;
    timer_config.duty_resolution = LEDC_TIMER_8_BIT; // resolution of PWM duty
    timer_config.freq_hz = 100000;                     // frequency of PWM signal
    timer_config.speed_mode = LEDC_HIGH_SPEED_MODE;  // timer mode
    timer_config.timer_num = LEDC_TIMER_0;           // timer index
    timer_config.clk_cfg = LEDC_AUTO_CLK; // Auto select the source clock

    ledc_timer_config(&timer_config);

    ledc_channel_config_t ledc_channel[2] = {

        {
            // Red channel.
            gpio_num : (gpio_num_t)m_pins[0],
            speed_mode : LEDC_HIGH_SPEED_MODE,
            channel : LEDC_CHANNEL_0,
            intr_type : LEDC_INTR_DISABLE,
            timer_sel : LEDC_TIMER_0,
            duty : 0xFF,
            hpoint : 0,
        },
        {
            // Red channel.
            gpio_num : (gpio_num_t)m_pins[1],
            speed_mode : LEDC_HIGH_SPEED_MODE,
            channel : LEDC_CHANNEL_1,
            intr_type : LEDC_INTR_DISABLE,
            timer_sel : LEDC_TIMER_0,
            duty : 0xFF,
            hpoint : 0,
        }};

    // Set LED Controller with previously prepared configuration
    for (int ch = 0; ch < 2; ch++)
    {
        ledc_channel_config(&ledc_channel[ch]);
    }

}

void LedBicolor::_pwmDestroy()
{
    ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 0);
}

void LedBicolor::_pwmSetDuty(uint16_t rg)
{
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, ((rg >> 8) & 0xFF));
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, ((rg ) & 0xFF));
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
}

void LedBicolor::_color(uint16_t rg)
{
    m_color = rg;
    m_state = State::STATIC;

    esp_timer_stop(m_blinkTimer);

    _pwmSetDuty(rg);
}

void LedBicolor::_blink(uint16_t rg, uint32_t period_ms)
{
    m_on = true;
    m_color = rg;
    m_state = State::BLINK;
    m_period = period_ms;

    _pwmSetDuty(rg);

    esp_timer_stop(m_blinkTimer);
    esp_timer_start_periodic(m_blinkTimer, period_ms * 1000);
}

void LedBicolor::lock()
{
    xSemaphoreTake(m_mutex, portMAX_DELAY);
}
void LedBicolor::unlock()
{
    xSemaphoreGive(m_mutex);
}