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
 * F&K Group 24ATC02D EEPROM
 *
 * EEPROM class declaration.
 *
 * @author Leonardo Hirata
 * @copyright F&K Group
 ******************************************************************************/

#pragma once

#include <cstdint>
#include <cctype>
#include <cstdlib>
#include <string.h>

#include "esp_log.h"
#include "esp_err.h"

#include "driver/i2c.h"

namespace EEPROM
{

/**
 * @brief EEPROM class.
 */
class EEPROM
{
public:
    /**
     * @brief Constructor.
     */
    EEPROM();

    /**
     * @brief Initialize the EEPROM.
     *
     * @param[in] port I2C port.
     * @param[in] sda I2C SDA gpio pin.
     * @param[in] scl I2C SCL gpio pin.
     * @param[in] wp EEPROM write protection gpio pin.
     * @param[in] wpEnable set true to enable write protection, false to disable.
     * 
     * @return
     */
    esp_err_t init(i2c_port_t port, int sda, int scl, int wp, bool wpEnable);

    /**
     * @brief Deinitialize the EEPROM.
     *
     * @return
     */
    esp_err_t deinit();

    /**
     * @brief Enable write protection.
     */
    void wp_enable();

    /**
     * @brief Disable write protection.
     */
    void wp_disable();

    /**
     * @brief Read byte(s) from the EEPROM.
     *
     * @param[in] addr EEPROM data address.
     * @param[in] data Buffer to get stored data in EEPROM.
     * @param[in] size Number of bytes to read from EEPROM.
     * 
     * @return
     */
    esp_err_t at24c02d_read_byte(uint8_t addr, uint8_t *data, uint16_t size);

    /**
     * @brief Write byte(s) from the EEPROM.
     *
     * @param[in] addr EEPROM data address.
     * @param[in] data Data to store in EEPROM.
     * @param[in] size Number of bytes to store in EEPROM.
     * 
     * @return
     */
    esp_err_t at24c02d_write_byte(uint8_t addr, uint8_t *data, uint16_t size);

    /**
     * @brief Read EEPROM 128-bit Unique ID.
     *
     * @param[out] data EEPROM 128-bit unique ID.
     * 
     * @return
     */
    esp_err_t at24c02d_read_uid(uint8_t *data);

    /**
     * @brief Read EEPROM Software Write Protection bit.
     *
     * @param[out] data EEPROM Software Write Protection bit.
     * 
     * @return
     */
    esp_err_t at24c02d_read_swp(uint8_t *data);

    /**
     * @brief Set EEPROM Software Write Protection bit.
     *
     * @return
     */
    esp_err_t at24c02d_set_swp();

    /**
     * @brief Reset EEPROM Software Write Protection bit.
     *
     * @return
     */
    esp_err_t at24c02d_reset_swp();
    
private:
    i2c_port_t m_port;
    i2c_mode_t m_mode;
    int m_sda;
    int m_scl;
    int m_wp;
    bool m_wpEnable;

    esp_err_t read_byte(uint8_t rAddr, uint8_t wAddr, uint8_t addr, uint8_t *data, uint16_t size);
    esp_err_t write_byte(uint8_t wAddr, uint8_t addr, uint8_t *data, uint16_t size);
};

} // namespace EEPROM