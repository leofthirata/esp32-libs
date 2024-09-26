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

#pragma once

#include "esp_log.h"
#include "esp_err.h"

#include "driver/i2c.h"

namespace I2C
{

class I2CMaster
{

public:
    I2CMaster();
    ~I2CMaster();
    esp_err_t init(i2c_port_t port, uint32_t freq, int sda, int scl);
    esp_err_t deinit();
    esp_err_t read_byte(uint8_t rAddr, uint8_t wAddr, uint8_t addr, uint8_t *data, uint16_t size);
    esp_err_t write_byte(uint8_t wAddr, uint8_t addr, uint8_t *data, uint16_t size);

private:
    i2c_port_t m_port;
    i2c_mode_t m_mode;
    int m_sda;
    int m_scl;
    uint32_t m_freq;
};

} // namespace I2C