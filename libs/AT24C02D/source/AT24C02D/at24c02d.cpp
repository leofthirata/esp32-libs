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
 * F&K Group AT24C02D EEPROM
 *
 * EEPROM class declaration.
 *
 * @author Leonardo Hirata
 * @copyright F&K Group
 ******************************************************************************/

#include "AT24C02D/at24c02d.hpp"

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "esp_log.h"

#define I2C_FREQUENCY   400000
#define ACK_ENABLE      0x1     /*!< I2C master will check ack from slave*/
#define ACK_DISABLE     0x0     /*!< I2C master will not check ack from slave */
#define ACK             I2C_MASTER_ACK     /*!< I2C ack value */
#define NACK            I2C_MASTER_NACK     /*!< I2C nack value */

#define AT24C02D_ADDRESS_WRITE 0xA0
#define AT24C02D_ADDRESS_READ 0xA1
#define AT24C02D_MEMORY_SIZE ((256 * 8) - 1)

#define AT24C02D_ADDRESS_OPTIONS_WRITE 0xB0
#define AT24C02D_ADDRESS_OPTIONS_READ 0xB1
#define AT24C02D_WORD_ADDRESS_UID 0x80
#define AT24C02D_WORD_ADDRESS_SWP 0xC0
#define AT24C02D_WORD_ADDRESS_RLS 0x00

static const char *TAG = "EEPROM";

namespace EEPROM
{
    
EEPROM::EEPROM()
{
    m_port = I2C_NUM_0;
    m_sda = 0;
    m_scl = 0;
    m_mode = I2C_MODE_MASTER;
    m_wp = 0;
    m_wpEnable = false;
}

esp_err_t EEPROM::init(i2c_port_t port, int sda, int scl, int wp, bool wpEnable)
{
    if (GPIO_IS_VALID_GPIO(sda) == false)
        return ESP_ERR_INVALID_ARG;

    if (GPIO_IS_VALID_GPIO(scl) == false)
        return ESP_ERR_INVALID_ARG;

    if (wpEnable)
    {
        if (GPIO_IS_VALID_GPIO(wp) == false)
            return ESP_ERR_INVALID_ARG;
    }

    m_port = port;
    m_sda = sda;
    m_scl = scl;
    m_wp = wp;
    m_wpEnable = wpEnable;

	esp_err_t err = ESP_OK;

    gpio_config_t gpio;
    gpio.intr_type = GPIO_INTR_DISABLE;
    gpio.mode = GPIO_MODE_OUTPUT;
    gpio.pin_bit_mask = (1ULL<<m_wp);
    gpio.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio.pull_up_en = GPIO_PULLUP_DISABLE;

    err = gpio_config(&gpio);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "%s (%s)", __func__, esp_err_to_name(err));
    }

	i2c_config_t conf;
    conf.mode = m_mode;
    conf.sda_io_num = m_sda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = m_scl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQUENCY;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

	err = i2c_param_config(m_port, &conf);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "%s (%s)", __func__, esp_err_to_name(err));
    }

	err = i2c_driver_install(m_port, m_mode, 0, 0, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "%s (%s)", __func__, esp_err_to_name(err));
    }

	return err;
}

esp_err_t EEPROM::deinit()
{
    return i2c_driver_delete(m_port);
}

void EEPROM::wp_enable()
{
    if (m_wpEnable)
        gpio_set_level((gpio_num_t) m_wp, 1);
}

void EEPROM::wp_disable()
{
    if (m_wpEnable)
        gpio_set_level((gpio_num_t)  m_wp, 0);
}

esp_err_t EEPROM::read_byte(uint8_t rAddr, uint8_t wAddr, uint8_t addr, uint8_t *data, uint16_t size)
{
    esp_err_t err = ESP_OK;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // dummy write begin
    // start bit
	err = i2c_master_start(cmd);
	if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "%s (%s)", __func__, esp_err_to_name(err));
    }

    // device address
    err = i2c_master_write_byte(cmd, wAddr, ACK_ENABLE);
	if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "%s (%s)", __func__, esp_err_to_name(err));
    }

    // dummy write end
    // memory address
	err = i2c_master_write_byte(cmd, addr, ACK_ENABLE);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "%s (%s)", __func__, esp_err_to_name(err));
    }

    // random read begin
    // start bit
    err = i2c_master_start(cmd);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "%s (%s)", __func__, esp_err_to_name(err));
    }

    // device address
	err = i2c_master_write_byte(cmd, rAddr, ACK_ENABLE);
	if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "%s (%s)", __func__, esp_err_to_name(err));
    }

    // byte read
    for (int i = 0; i < size; i++)
    {
        err = i2c_master_read_byte(cmd, &data[i], (i == size - 1) ? NACK : ACK);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "%s (%s)", __func__, esp_err_to_name(err));
        }
    }

    // stop bit
	err = i2c_master_stop(cmd);
	if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "%s (%s)", __func__, esp_err_to_name(err));
    }

	err = i2c_master_cmd_begin(m_port, cmd, 1000 / portTICK_PERIOD_MS);
	if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "%s (%s)", __func__, esp_err_to_name(err));
    }

	i2c_cmd_link_delete(cmd);

    wp_enable();

	return err;
}

esp_err_t EEPROM::write_byte(uint8_t wAddr, uint8_t addr, uint8_t *data, uint16_t size)
{
    esp_err_t err = ESP_OK;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	
    // start bit
    err = i2c_master_start(cmd);
	if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "%s (%s)", "i2c_master_start", esp_err_to_name(err));
    }

    // device address
    err = i2c_master_write_byte(cmd, wAddr, ACK_ENABLE);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "%s (%s)", "i2c_master_write_byte", esp_err_to_name(err));
    }

    // memory address
	err = i2c_master_write_byte(cmd, addr, ACK_ENABLE);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "%s (%s)", "i2c_master_write_byte", esp_err_to_name(err));
    }

    // data
    for (int i = 0; i < size; i++)
    {
        err = i2c_master_write_byte(cmd, data[i], ACK_ENABLE);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "%s (%s)", "i2c_master_write_byte", esp_err_to_name(err));
        }
    }

    // stop bit
	err = i2c_master_stop(cmd);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "%s (%s)", "i2c_master_stop", esp_err_to_name(err));
    }

    // i2c_set_period

	err = i2c_master_cmd_begin(m_port, cmd, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "%s (%s)", "i2c_master_cmd_begin", esp_err_to_name(err));
    }
    
    i2c_cmd_link_delete(cmd);

	usleep(1000*2);
    
	return err;
}

esp_err_t EEPROM::at24c02d_read_byte(uint8_t addr, uint8_t *data, uint16_t size)
{
	if (addr > AT24C02D_MEMORY_SIZE) 
        return ESP_ERR_INVALID_ARG;

    if ((size - 1) > 0xFF) 
        return ESP_ERR_INVALID_ARG;

    wp_disable();

    usleep(1000*2);
    return read_byte(AT24C02D_ADDRESS_READ, AT24C02D_ADDRESS_WRITE, addr, data, size);
}

esp_err_t EEPROM::at24c02d_write_byte(uint8_t addr, uint8_t *data, uint16_t size)
{
    esp_err_t err = ESP_OK;
    
	if (addr > AT24C02D_MEMORY_SIZE) 
    {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (size > 0xFF)
    {
        return ESP_ERR_INVALID_ARG;
    }

	usleep(1000*2);
    
    wp_disable();

    if (size <= 8)
    {
        err = write_byte(AT24C02D_ADDRESS_WRITE, addr, data, size);
        wp_enable();

        return err;
    }
    else
    {
        esp_err_t err = ESP_OK;

        uint16_t temp = 0;
        uint16_t n = 8;
        uint8_t aux = addr;
        uint8_t counter = 0;

        while (size != temp)
        {   
            usleep(1000*2);
            err = write_byte(AT24C02D_ADDRESS_WRITE, aux, data + 8*counter, n);
            aux += 8;
            temp += n;
            counter++;

            if (size - temp < 8)
                n =  size - temp;
        }

        wp_enable();

        return err;
    }
}

esp_err_t EEPROM::at24c02d_read_uid(uint8_t *data)
{
    wp_disable();

    usleep(1000*2);
    return read_byte(AT24C02D_ADDRESS_OPTIONS_READ, AT24C02D_ADDRESS_OPTIONS_WRITE, AT24C02D_WORD_ADDRESS_UID, data, 16);
}

esp_err_t EEPROM::at24c02d_read_swp(uint8_t *data)
{
    wp_disable();

    usleep(1000*2);
    return read_byte(AT24C02D_ADDRESS_OPTIONS_READ, AT24C02D_ADDRESS_OPTIONS_WRITE, AT24C02D_WORD_ADDRESS_UID, data, 1);
}

esp_err_t EEPROM::at24c02d_set_swp()
{
    esp_err_t err = ESP_OK;

    uint8_t data = 0x01;

    wp_disable();

    usleep(1000*2);
    
    err = write_byte(AT24C02D_ADDRESS_OPTIONS_WRITE, AT24C02D_WORD_ADDRESS_SWP, &data, 1);

    wp_enable();

    return err;
}

esp_err_t EEPROM::at24c02d_reset_swp()
{
    esp_err_t err = ESP_OK;

    uint8_t data = 0x00;

    wp_disable();

    usleep(1000*2);
    
    err = write_byte(AT24C02D_ADDRESS_OPTIONS_WRITE, AT24C02D_WORD_ADDRESS_SWP, &data, 1);

    wp_enable();

    return err;
}

} // namespace EEPROM
