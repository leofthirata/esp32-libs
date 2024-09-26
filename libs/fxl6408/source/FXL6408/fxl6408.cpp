/******************************************************************************
 * Copyright © 2008 - 2024, F&K Group. All rights reserved.
 *
 * No part of this software may be reproduced, distributed, or transmitted in
 * any form or by any means without the prior written permission of the F&K Group
 * company.
 *
 * For permission requests, contact the company through the e-mail address
 * tbd@fkgroup.com.br with subject "Software Licence Request".
 ******************************************************************************/

/*******************************************************************************
 * F&K Group FXL6408 GpioExpander
 *
 * GpioExpander class declaration.
 *
 * @author Leonardo Hirata
 * @copyright F&K Group
 ******************************************************************************/

#include "FXL6408/fxl6408.hpp"

namespace GpioExpander
{

#define HIGH                                0x01
#define LOW                                 0x00

#define FXL6408_ADDRESS_WRITE_1             0x88
#define FXL6408_ADDRESS_READ_1              0x89

#define FXL6408_ADDRESS_WRITE_0             0x86
#define FXL6408_ADDRESS_READ_0              0x87

#define FXL6408_ADDRESS_UID_CTRL            0x01 // R/W
#define FXL6408_ADDRESS_IO_DIR              0x03 // R/W
#define FXL6408_ADDRESS_OUT_STATE           0x05 // R/W
#define FXL6408_ADDRESS_OUT_HIGHZ           0x07 // R/W
#define FXL6408_ADDRESS_IN_DEFAULT_STATE    0x09 // R/W
#define FXL6408_ADDRESS_PU_EN               0x0B // R/W
#define FXL6408_ADDRESS_PU_PD               0x0D // R/W
#define FXL6408_ADDRESS_IN_STATUS           0x0F // R
#define FXL6408_ADDRESS_IT_MASK             0x11 // R/W
#define FXL6408_ADDRESS_IT_STATUS           0x13 // R/W

#define FXL6408_ADDRESS_RESERVED_1          0x02
#define FXL6408_ADDRESS_RESERVED_2          0x04
#define FXL6408_ADDRESS_RESERVED_3          0x06
#define FXL6408_ADDRESS_RESERVED_4          0x08
#define FXL6408_ADDRESS_RESERVED_5          0x0A
#define FXL6408_ADDRESS_RESERVED_6          0x0C
#define FXL6408_ADDRESS_RESERVED_7          0x0E
#define FXL6408_ADDRESS_RESERVED_8          0x10
#define FXL6408_ADDRESS_RESERVED_9          0x12

static const char *TAG = "GPIO EXPANDER";

static TaskHandle_t thread = nullptr;
static BaseType_t xHigherPriorityTaskWoken;
static TaskHandle_t *m_tasks[8];

static QueueHandle_t gpio_evt_queue = NULL;
static GpioExpanderEnum_t bit_to_gpio(uint16_t gpio);

static void IRAM_ATTR gpio_expander_isr(void *arg)
{
    xTaskNotifyFromISR(thread, 0x01, eSetBits, &xHigherPriorityTaskWoken);
}

void gpio_expander_task(void *args)
{
    GpioExpander *expander = static_cast<GpioExpander *>(args);

    for (;;)
    {
        uint32_t event = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint8_t status = 0;
        esp_err_t err = expander->fxl6408_read_it_status(&status);

        for (uint16_t gpio = 0x01; gpio <= 0x80; gpio = gpio << 1)
            if ((status & gpio) == gpio) xTaskNotify(*m_tasks[bit_to_gpio(gpio)], 0x01, eSetBits);
    }
}

static GpioExpanderEnum_t bit_to_gpio(uint16_t gpio)
{
    GpioExpanderEnum_t idx = GPIO_EXPANDER_IO_0;

    switch (gpio)
    {
        case FXL6408_GPIO_0:
        {
            idx = GPIO_EXPANDER_IO_0;
            break;
        }
        case FXL6408_GPIO_1:
        {
            idx = GPIO_EXPANDER_IO_1;
            break;
        }
        case FXL6408_GPIO_2:
        {
            idx = GPIO_EXPANDER_IO_2;
            break;
        }
        case FXL6408_GPIO_3:
        {
            idx = GPIO_EXPANDER_IO_3;
            break;
        }
        case FXL6408_GPIO_4:
        {
            idx = GPIO_EXPANDER_IO_4;
            break;
        }
        case FXL6408_GPIO_5:
        {
            idx = GPIO_EXPANDER_IO_5;
            break;
        }
        case FXL6408_GPIO_6:
        {
            idx = GPIO_EXPANDER_IO_6;
            break;
        }
        case FXL6408_GPIO_7:
        {
            idx = GPIO_EXPANDER_IO_7;
            break;
        }
    }

    return idx;
}

esp_err_t gpio_expander_is_gpio_valid(GpioExpanderEnum_t gpio)
{
    bool ok = false;

    switch (gpio)
    {
        case GPIO_EXPANDER_IO_0:
        case GPIO_EXPANDER_IO_1:
        case GPIO_EXPANDER_IO_2:
        case GPIO_EXPANDER_IO_3:
        case GPIO_EXPANDER_IO_4:
        case GPIO_EXPANDER_IO_5:
        case GPIO_EXPANDER_IO_6:
        case GPIO_EXPANDER_IO_7:
            ok = true;
    }

    return ok ? ESP_OK : ESP_ERR_INVALID_ARG;
}

GpioExpander::GpioExpander()
{
    m_addr = 0;
    
    for (uint8_t idx = 0; idx <= 7; idx++)
        m_tasks[idx] = NULL;
}

esp_err_t GpioExpander::init(I2C::I2CMaster *i2c, int rst, int it, int addr)
{
    m_i2c = i2c;
    m_rst = rst;
    m_it = it;
    m_addr = addr;

    if (m_addr == 1)
    {
        m_addr_read = FXL6408_ADDRESS_READ_1;
        m_addr_write = FXL6408_ADDRESS_WRITE_1;
    }
    else if (m_addr == 0)
    {
        m_addr_read = FXL6408_ADDRESS_READ_0;
        m_addr_write = FXL6408_ADDRESS_WRITE_0;
    }

	esp_err_t err = ESP_OK;

    gpio_config_t gpio_rst = {};
    gpio_rst.intr_type = GPIO_INTR_DISABLE;
    gpio_rst.mode = GPIO_MODE_OUTPUT;
    gpio_rst.pin_bit_mask = (1ULL<<m_rst);
    gpio_rst.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_rst.pull_up_en = GPIO_PULLUP_DISABLE;

    err = gpio_config(&gpio_rst);
    if (err != ESP_OK) return err;

    err = gpio_set_level((gpio_num_t) m_rst, HIGH);

    gpio_config_t gpio_it = {};
    gpio_it.intr_type = GPIO_INTR_NEGEDGE;
    gpio_it.mode = GPIO_MODE_INPUT;
    gpio_it.pin_bit_mask = (1ULL<<m_it);
    gpio_it.pull_up_en = GPIO_PULLUP_ENABLE;

    err = gpio_config(&gpio_it);
    if (err != ESP_OK) return err;

    return ESP_OK;
}

esp_err_t GpioExpander::deinit()
{
    return m_i2c->deinit();
}

esp_err_t GpioExpander::fxl6408_read_ctrl(uint8_t *data)
{
    esp_err_t err = m_i2c->read_byte(m_addr_read, m_addr_write, FXL6408_ADDRESS_UID_CTRL, data, 1);
    if (err != ESP_OK) printf("failed to read ctrl from addr %Xh\r\n", FXL6408_ADDRESS_UID_CTRL);

    return err;
}

esp_err_t GpioExpander::fxl6408_software_reset()
{
    uint8_t data = 0;

    esp_err_t err = m_i2c->read_byte(m_addr_read, m_addr_write, FXL6408_ADDRESS_UID_CTRL, &data, 1);
    if (err != ESP_OK)
    {
        printf("failed to read ctrl from addr %Xh\r\n", FXL6408_ADDRESS_UID_CTRL);
        return err;
    }

    data = data + 0x01;

    err = m_i2c->write_byte(m_addr_write, FXL6408_ADDRESS_UID_CTRL, &data, 1);
    if (err != ESP_OK) printf("failed to write to addr %Xh\r\n", FXL6408_ADDRESS_UID_CTRL);

    return err;
}

esp_err_t GpioExpander::fxl6408_read_io_dir(uint8_t *dir)
{
    esp_err_t err = m_i2c->read_byte(m_addr_read, m_addr_write, FXL6408_ADDRESS_IO_DIR, dir, 1);
    if (err == ESP_OK) printf("read_dir %u\r\n", *dir);
    else printf("failed to read addr %Xh\r\n", FXL6408_ADDRESS_IO_DIR);

    return err;
}

esp_err_t GpioExpander::fxl6408_set_io_dir(uint8_t gpio, uint8_t dir)
{
    uint8_t read_dir = 0;

    esp_err_t err = fxl6408_read_io_dir(&read_dir);
    
    uint8_t new_dir = read_dir;

    if ((new_dir & gpio) != dir)
        new_dir = dir == FXL6408_GPIO_MODE_INPUT ? new_dir - gpio : new_dir + gpio;

    err = m_i2c->write_byte(m_addr_write, FXL6408_ADDRESS_IO_DIR, &new_dir, 1);
    if (err == ESP_OK) printf("wrote new_dir %u\r\n", new_dir);
    else printf("failed to write to addr %Xh\r\n", FXL6408_ADDRESS_IO_DIR);

    read_dir = 0;

    err = fxl6408_read_io_dir(&read_dir);

    if (read_dir != new_dir)
    {
        printf("read_dir %u != new_dir %u\r\n", read_dir, new_dir);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t GpioExpander::fxl6408_read_io_level(uint8_t *output)
{
    esp_err_t err = m_i2c->read_byte(m_addr_read, m_addr_write, FXL6408_ADDRESS_OUT_STATE, output, 1);
    if (err == ESP_OK) printf("output %u\r\n", *output);
    else printf("failed to read addr %Xh\r\n", FXL6408_ADDRESS_OUT_STATE);

    return err;
}

esp_err_t GpioExpander::fxl6408_set_io_level(uint8_t gpio, uint8_t output)
{
    uint8_t read_output = 0;

    esp_err_t err = fxl6408_read_io_level(&read_output);
    if (err != ESP_OK) return err;

    uint8_t new_output = read_output;

    if ((new_output & gpio) != output)
        new_output = output == FXL6408_GPIO_LEVEL_LOW ? new_output - gpio : new_output + gpio;

    err = m_i2c->write_byte(m_addr_write, FXL6408_ADDRESS_OUT_STATE, &new_output, 1);
    if (err == ESP_OK) printf("wrote new_output %u\r\n", new_output);
    else printf("failed to write to addr %Xh\r\n", FXL6408_ADDRESS_OUT_STATE);

    read_output = 0;

    err = fxl6408_read_io_level(&read_output);
    if (err != ESP_OK) return err;

    if (read_output != new_output)
    {
        printf("read_output %u != new_output %u\r\n", read_output, new_output);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t GpioExpander::fxl6408_read_io_highz(uint8_t *highz)
{
    esp_err_t err = m_i2c->read_byte(m_addr_read, m_addr_write, FXL6408_ADDRESS_OUT_HIGHZ, highz, 1);
    if (err == ESP_OK) printf("highz %u\r\n", *highz);
    else printf("failed to read addr %Xh\r\n", FXL6408_ADDRESS_OUT_HIGHZ);

    return err;
}

esp_err_t GpioExpander::fxl6408_set_io_highz(uint8_t gpio, uint8_t highz)
{
    uint8_t read_highz = 0;

    esp_err_t err = fxl6408_read_io_highz(&read_highz);
    if (err != ESP_OK) return err;

    uint8_t new_highz = read_highz;

    if ((new_highz & gpio) != highz)
        new_highz = highz == FXL6408_GPIO_LEVEL_LOW ? new_highz - gpio : new_highz + gpio;

    err = m_i2c->write_byte(m_addr_write, FXL6408_ADDRESS_OUT_HIGHZ, &new_highz, 1);
    if (err == ESP_OK) printf("wrote new_highz %u\r\n", new_highz);
    else printf("failed to write to addr %Xh\r\n", FXL6408_ADDRESS_OUT_HIGHZ);

    read_highz = 0;

    err = fxl6408_read_io_highz(&read_highz);
    if (err != ESP_OK) return err;

    if (read_highz != new_highz)
    {
        printf("read_highz %u != new_highz %u\r\n", read_highz, new_highz);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t GpioExpander::fxl6408_read_input_state(uint8_t *input)
{
    esp_err_t err = m_i2c->read_byte(m_addr_read, m_addr_write, FXL6408_ADDRESS_IN_DEFAULT_STATE, input, 1);
    if (err == ESP_OK) printf("input %u\r\n", *input);
    else printf("failed to read addr %Xh\r\n", FXL6408_ADDRESS_IN_DEFAULT_STATE);

    return err;
}

esp_err_t GpioExpander::fxl6408_set_input_state(uint8_t gpio, uint8_t state)
{
    uint8_t read_state = 0;

    esp_err_t err = fxl6408_read_input_state(&read_state);
    if (err != ESP_OK) return err;

    uint8_t new_state = read_state;

    if ((new_state & gpio) != state)
        new_state = state == FXL6408_GPIO_NO_MASK ? new_state - gpio : new_state + gpio;

    err = m_i2c->write_byte(m_addr_write, FXL6408_ADDRESS_IN_DEFAULT_STATE, &new_state, 1);
    if (err == ESP_OK) printf("wrote new_state %u\r\n", new_state);
    else printf("failed to write to addr %Xh\r\n", FXL6408_ADDRESS_IN_DEFAULT_STATE);

    read_state = 0;

    err = fxl6408_read_input_state(&read_state);
    if (err != ESP_OK) return err;

    if (read_state != new_state)
    {
        printf("read_state %u != new_state %u\r\n", read_state, new_state);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t GpioExpander::fxl6408_read_pu_en(uint8_t *pu_en)
{
    esp_err_t err = m_i2c->read_byte(m_addr_read, m_addr_write, FXL6408_ADDRESS_PU_EN, pu_en, 1);
    if (err == ESP_OK) printf("pu_en %u\r\n", *pu_en);
    else printf("failed to read addr %Xh\r\n", FXL6408_ADDRESS_PU_EN);

    return err;
}

esp_err_t GpioExpander::fxl6408_set_pu_en(uint8_t gpio, uint8_t en)
{
    uint8_t read_en = 0;

    esp_err_t err = fxl6408_read_pu_en(&read_en);
    if (err != ESP_OK) return err;

    uint8_t new_en = read_en;

    if ((new_en & gpio) != en)
        new_en = en == FXL6408_GPIO_PUPD_DISABLE ? new_en - gpio : new_en + gpio;

    err = m_i2c->write_byte(m_addr_write, FXL6408_ADDRESS_PU_EN, &new_en, 1);
    if (err == ESP_OK) printf("wrote new_en %u\r\n", new_en);
    else printf("failed to write to addr %Xh\r\n", FXL6408_ADDRESS_PU_EN);

    read_en = 0;

    err = fxl6408_read_pu_en(&read_en);
    if (err != ESP_OK) return err;

    if (read_en != new_en)
    {
        printf("read_en %u != new_en %u\r\n", read_en, new_en);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t GpioExpander::fxl6408_read_pu_pd(uint8_t *pu_pd)
{
    esp_err_t err = m_i2c->read_byte(m_addr_read, m_addr_write, FXL6408_ADDRESS_PU_PD, pu_pd, 1);
    if (err == ESP_OK) printf("pu_pd %u\r\n", *pu_pd);
    else printf("failed to read addr %Xh\r\n", FXL6408_ADDRESS_PU_PD);

    return err;
}

esp_err_t GpioExpander::fxl6408_set_pu_pd(uint8_t gpio, uint8_t pu_pd)
{
    uint8_t read_pu_pd = 0;

    esp_err_t err = fxl6408_read_pu_en(&read_pu_pd);
    if (err != ESP_OK) return err;

    uint8_t new_pu_pd = read_pu_pd;

    if ((new_pu_pd & gpio) != pu_pd)
        new_pu_pd = pu_pd == FXL6408_GPIO_PUPD_DISABLE ? new_pu_pd - gpio : new_pu_pd + gpio;

    err = m_i2c->write_byte(m_addr_write, FXL6408_ADDRESS_PU_EN, &new_pu_pd, 1);
    if (err == ESP_OK) printf("wrote new_pu_pd %u\r\n", new_pu_pd);
    else printf("failed to write to addr %Xh\r\n", FXL6408_ADDRESS_PU_EN);

    read_pu_pd = 0;

    err = fxl6408_read_pu_en(&read_pu_pd);
    if (err != ESP_OK) return err;

    if (read_pu_pd != new_pu_pd)
    {
        printf("read_pu_pd %u != new_pu_pd %u\r\n", read_pu_pd, new_pu_pd);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t GpioExpander::fxl6408_read_input_status(uint8_t *status)
{
    esp_err_t err = m_i2c->read_byte(m_addr_read, m_addr_write, FXL6408_ADDRESS_IN_STATUS, status, 1);
    if (err == ESP_OK) printf("status %u\r\n", *status);
    else printf("failed to read addr %Xh\r\n", FXL6408_ADDRESS_IN_STATUS);

    return err;
}

esp_err_t GpioExpander::fxl6408_read_it_mask(uint8_t *mask)
{
    esp_err_t err = m_i2c->read_byte(m_addr_read, m_addr_write, FXL6408_ADDRESS_IT_MASK, mask, 1);
    if (err == ESP_OK) printf("mask %u\r\n", *mask);
    else printf("failed to read addr %Xh\r\n", FXL6408_ADDRESS_IT_MASK);

    return err;
}

esp_err_t GpioExpander::fxl6408_set_it_mask(uint8_t gpio, uint8_t mask)
{
    uint8_t read_mask = 0;

    esp_err_t err = fxl6408_read_it_mask(&read_mask);
    if (err != ESP_OK) return err;

    uint8_t new_mask = read_mask;

    if ((new_mask & gpio) != mask)
        new_mask = mask == FXL6408_GPIO_NO_MASK ? new_mask - gpio : new_mask + gpio;

    err = m_i2c->write_byte(m_addr_write, FXL6408_ADDRESS_IT_MASK, &new_mask, 1);
    if (err == ESP_OK) printf("wrote new_mask %u\r\n", new_mask);
    else printf("failed to write to addr %Xh\r\n", FXL6408_ADDRESS_IT_MASK);

    read_mask = 0;

    err = fxl6408_read_it_mask(&read_mask);
    if (err != ESP_OK) return err;

    if (read_mask != new_mask)
    {
        printf("read_mask %u != new_mask %u\r\n", read_mask, new_mask);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t GpioExpander::fxl6408_read_it_status(uint8_t *status)
{
    esp_err_t err = m_i2c->read_byte(m_addr_read, m_addr_write, FXL6408_ADDRESS_IT_STATUS, status, 1);
    if (err == ESP_OK) printf("status %u\r\n", *status);
    else printf("failed to read addr %Xh\r\n", FXL6408_ADDRESS_IT_STATUS);

    return err;
}

esp_err_t GpioExpander::fxl6408_reset()
{
    esp_err_t err = gpio_set_level((gpio_num_t) m_rst, LOW);
    if (err != ESP_OK)
    {
        printf("failed to set gpio LOW\r\n");
        return err;
    }

    err = gpio_set_level((gpio_num_t) m_rst, HIGH);
    if (err != ESP_OK) printf("failed to set gpio HIGH\r\n");

    return err;
}

esp_err_t GpioExpander::fxl6408_set_task(TaskHandle_t *task, GpioExpanderEnum_t gpio)
{
    esp_err_t err = gpio_expander_is_gpio_valid(gpio);
    if (err != ESP_OK) return err;

    gpio_install_isr_service(0);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    err = gpio_isr_handler_add((gpio_num_t) m_it, gpio_expander_isr, (void *)m_it);
    if (err != ESP_OK) return err;

    auto ok = xTaskCreate(gpio_expander_task, "gpio_expander", 2048,
                        static_cast<void *>(this), 10, &thread);
    if (pdPASS != ok) return ESP_ERR_NO_MEM;

    m_tasks[gpio] = task;

    return err;
}

} // namespace GpioExpander