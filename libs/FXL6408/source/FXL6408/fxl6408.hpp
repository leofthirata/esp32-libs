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
 * F&K Group FXL6408 GPIO EXPANDER
 *
 * FXL6408 class declaration.
 *
 * @author Leonardo Hirata
 * @copyright F&K Group
 ******************************************************************************/

#pragma once

#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "../../../I2C/source/I2C/i2c.hpp"

namespace GpioExpander
{

#define FXL6408_GPIO_MODE_INPUT 0
#define FXL6408_GPIO_MODE_OUTPUT 1
#define FXL6408_GPIO_0 0x00
#define FXL6408_GPIO_1 0x02
#define FXL6408_GPIO_2 0x04
#define FXL6408_GPIO_3 0x08
#define FXL6408_GPIO_4 0x10
#define FXL6408_GPIO_5 0x20
#define FXL6408_GPIO_6 0x40
#define FXL6408_GPIO_7 0x80
#define FXL6408_GPIO_LEVEL_LOW  0
#define FXL6408_GPIO_LEVEL_HIGH 1
#define FXL6408_GPIO_LEVEL_HIGHZ 1
#define FXL6408_GPIO_PUPD_DISABLE    0
#define FXL6408_GPIO_PUPD_ENABLE    1
#define FXL6408_GPIO_NO_MASK 0
#define FXL6408_GPIO_MASK 1
#define FXL6408_GPIO_PULL_DOWN 0
#define FXL6408_GPIO_PULL_UP 1
#define FXL6408_GPIO_INPUT_DEFAULT_LOW 0
#define FXL6408_GPIO_INPUT_DEFAULT_HIGH 1

typedef enum
{
    GPIO_EXPANDER_IO_0 = 0,
    GPIO_EXPANDER_IO_1 = 1,
    GPIO_EXPANDER_IO_2 = 2,
    GPIO_EXPANDER_IO_3 = 3,
    GPIO_EXPANDER_IO_4 = 4,
    GPIO_EXPANDER_IO_5 = 5,
    GPIO_EXPANDER_IO_6 = 6,
    GPIO_EXPANDER_IO_7 = 7,
} GpioExpanderEnum_t;

/**
 * @brief GpioExpander class.
 */

class GpioExpander
{
public:
    /**
     * @brief Constructor.
     */
    GpioExpander();

    /**
     * @brief Initialize the GPIO Expander.
     *
     * @param[in] i2c   I2C master object.
     * @param[in] rst   Reset output pin.
     * @param[in] it    Interrupt input pin.
     * @param[in] addr  FXL6408 device address pin level.
     * 
     * @return
     */
    esp_err_t init(I2C::I2CMaster *i2c, int rst, int it, int addr);

    /**
     * @brief Deinitialize the GPIO Expander.
     *
     * @return
     */
    esp_err_t deinit();

    esp_err_t fxl6408_read_ctrl(uint8_t *data);
    esp_err_t fxl6408_software_reset();
    esp_err_t fxl6408_read_io_dir(uint8_t *dir);
    esp_err_t fxl6408_set_io_dir(uint8_t gpio, uint8_t dir);
    esp_err_t fxl6408_read_io_level(uint8_t *output);
    esp_err_t fxl6408_set_io_level(uint8_t gpio, uint8_t output);
    esp_err_t fxl6408_read_io_highz(uint8_t *highz);
    esp_err_t fxl6408_set_io_highz(uint8_t gpio, uint8_t highz);
    esp_err_t fxl6408_read_input_state(uint8_t *input);
    esp_err_t fxl6408_set_input_state(uint8_t gpio, uint8_t state);
    esp_err_t fxl6408_read_pu_en(uint8_t *pu_en);
    esp_err_t fxl6408_set_pu_en(uint8_t gpio, uint8_t en);
    esp_err_t fxl6408_read_pu_pd(uint8_t *pu_pd);
    esp_err_t fxl6408_set_pu_pd(uint8_t gpio, uint8_t pu_pd);
    esp_err_t fxl6408_read_input_status(uint8_t *status);
    esp_err_t fxl6408_read_it_mask(uint8_t *mask);
    esp_err_t fxl6408_set_it_mask(uint8_t gpio, uint8_t mask);
    esp_err_t fxl6408_read_it_status(uint8_t *status);
    esp_err_t fxl6408_reset();
    esp_err_t fxl6408_set_task(TaskHandle_t *task, GpioExpanderEnum_t gpio);

private:
    I2C::I2CMaster *m_i2c;
    int m_rst;
    int m_it;
    int m_addr;
    uint8_t m_addr_read;
    uint8_t m_addr_write;
    bool m_isInterrupted;

    friend void gpio_expander_task(void *args);
    friend esp_err_t gpio_expander_is_gpio_valid(GpioExpanderEnum_t gpio);
};

}