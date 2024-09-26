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
 * F&K Group I2C
 *
 * I2C APIs implementation.
 *
 * @author Leonardo Hirata
 * @copyright F&K Group
 ******************************************************************************/

#include "I2C/i2c.hpp"

namespace I2C
{

#define I2C_FREQUENCY_DEFAULT   400000
#define ACK_ENABLE              0x01     /*!< I2C master will check ack from slave*/
#define ACK_DISABLE             0x00     /*!< I2C master will not check ack from slave */
#define ACK                     I2C_MASTER_ACK     /*!< I2C ack value */
#define NACK                    I2C_MASTER_NACK     /*!< I2C nack value */

static const char *TAG = "I2C";

I2CMaster::I2CMaster()
{
    m_port = I2C_NUM_0;
    m_sda = 0;
    m_scl = 0;
    m_mode = I2C_MODE_MASTER;
    m_freq = I2C_FREQUENCY_DEFAULT;
}

esp_err_t I2CMaster::init(i2c_port_t port, uint32_t freq, int sda, int scl)
{
    if (GPIO_IS_VALID_GPIO(sda) == false) return ESP_ERR_INVALID_ARG;
    if (GPIO_IS_VALID_GPIO(scl) == false) return ESP_ERR_INVALID_ARG;

    m_port = port;
    m_sda = sda;
    m_scl = scl;

    i2c_config_t conf;
    conf.mode = m_mode;
    conf.sda_io_num = m_sda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = m_scl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = m_freq;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

	esp_err_t err = i2c_param_config(m_port, &conf);
    if (err != ESP_OK) return err;

	return i2c_driver_install(m_port, m_mode, 0, 0, 0);
}

esp_err_t I2CMaster::deinit()
{
    return i2c_driver_delete(m_port);
}

esp_err_t I2CMaster::read_byte(uint8_t rAddr, uint8_t wAddr, uint8_t addr, uint8_t *data, uint16_t size)
{
    esp_err_t err = ESP_OK;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // dummy write begin
    // start bit
	err = i2c_master_start(cmd);
	if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_master_start (%s)", esp_err_to_name(err));
        return err;
    }

    // device address
    err = i2c_master_write_byte(cmd, wAddr, ACK_ENABLE);
	if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_master_write_byte (%s)", esp_err_to_name(err));
        return err;
    }

    // dummy write end
    // memory address
	err = i2c_master_write_byte(cmd, addr, ACK_ENABLE);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_master_write_byte (%s)", esp_err_to_name(err));
        return err;
    }

    // random read begin
    // start bit
    err = i2c_master_start(cmd);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_master_start (%s)", esp_err_to_name(err));
        return err;
    }

    // device address
	err = i2c_master_write_byte(cmd, rAddr, ACK_ENABLE);
	if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_master_write_byte (%s)", esp_err_to_name(err));
        return err;
    }

    // byte read
    for (int i = 0; i < size; i++)
    {
        err = i2c_master_read_byte(cmd, &data[i], (i == size - 1) ? NACK : ACK);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "i2c_master_read_byte (%s)", esp_err_to_name(err));
            return err;
        }
    }

    // stop bit
	err = i2c_master_stop(cmd);
	if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_master_stop (%s)", esp_err_to_name(err));
        return err;
    }

	err = i2c_master_cmd_begin(m_port, cmd, 1000 / portTICK_PERIOD_MS);
	if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_master_cmd_begin (%s)", esp_err_to_name(err));
        return err;
    }

	i2c_cmd_link_delete(cmd);

	return err;
}

esp_err_t I2CMaster::write_byte(uint8_t wAddr, uint8_t addr, uint8_t *data, uint16_t size)
{
    esp_err_t err = ESP_OK;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	
    // start bit
    err = i2c_master_start(cmd);
	if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_master_start (%s)", esp_err_to_name(err));
        return err;
    }

    // device address
    err = i2c_master_write_byte(cmd, wAddr, ACK_ENABLE);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "i2c_master_write_byte (%s)", esp_err_to_name(err));
        return err;
    }

    // memory address
	err = i2c_master_write_byte(cmd, addr, ACK_ENABLE);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_master_write_byte (%s)", esp_err_to_name(err));
        return err;
    }

    // data
    for (int i = 0; i < size; i++)
    {
        err = i2c_master_write_byte(cmd, data[i], ACK_ENABLE);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "i2c_master_write_byte (%s)", esp_err_to_name(err));
        }
    }

    // stop bit
	err = i2c_master_stop(cmd);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_master_stop (%s)", esp_err_to_name(err));
        return err;
    }

    // i2c_set_period

	err = i2c_master_cmd_begin(m_port, cmd, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_master_cmd_begin (%s)", esp_err_to_name(err));
        return err;
    }
    
    i2c_cmd_link_delete(cmd);

	return err;
}

} // namespace I2C