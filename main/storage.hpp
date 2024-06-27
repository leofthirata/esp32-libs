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
 * F&K Group NVS Storage
 *
 * Storage declaration.
 *
 * @author Leonardo Hirata
 * @copyright F&K Group
 ******************************************************************************/

#include "sdkconfig.h"
// #if HSSN_PRODUCT_ID == 1 || HSSN_PRODUCT_ID == 2
#pragma once

#include <cstdint>

#include "esp_err.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "main.h"

namespace Storage
{

/**
 * @brief Storage class.
 */
class Storage
{
public:
    /**
     * @brief Infrared storage version.
     *
     * Internal storage version, used to do migrations on after upgrades.
     */
    static constexpr const int8_t VERSION = 0;

    /**
     * @brief Maximum number of stored codes.
     */
    static const uint8_t MAX_CODES = 150;

    /**
     * @brief Constructor.
     */
    Storage();

    /**
     * @brief Initialize the storage.
     *
     * @return
     */
    esp_err_t init();

    /**
     * @brief Deinitialize the storage.
     *
     * @return
     */
    esp_err_t deinit();

    /**
     * @brief Store a new key in the database.
     *
     * @param[in] code IR code to store.
     * @param[out] id New ir code ID.
     *
     * @return
     */
    
    esp_err_t set_key_u8(nvs_handle_t ns, const char *key, uint8_t *data, size_t size);
    esp_err_t set_key_u16(nvs_handle_t ns, const char *key, uint16_t *data);
    esp_err_t set_key_u32(nvs_handle_t ns, const char *key, uint32_t *data);
    esp_err_t set_key_u64(nvs_handle_t ns, const char *key, uint64_t *data);
    esp_err_t status_set_key(const char *key, void *data);

    esp_err_t get_key_u8(nvs_handle_t ns, const char *key, uint8_t *data, size_t size);
    esp_err_t get_key_u16(nvs_handle_t ns, const char *key, uint16_t *data);
    esp_err_t get_key_u32(nvs_handle_t ns, const char *key, uint32_t *data);
    esp_err_t get_key_u64(nvs_handle_t ns, const char *key, uint64_t *data);
    
    esp_err_t status_get_key(const char *key, void *data);
    esp_err_t config_set_key(const char *key, void *data);

    void status_set(Isca_t *status);
    void status_get(Isca_t *status);

    void otp_set(Isca_t *otp);
    void otp_get(Isca_t *otp);

    void config_set(Isca_t *config);
    void config_get(Isca_t *config);

    void ble_lora_gsm_set(Isca_t *config);
    void ble_lora_gsm_get(Isca_t *config);

    /**
     * @brief Find an infreared code in the database.
     *
     * @param[in] id IR code ID.
     * @param[out] code IR code.
     *
     * @return
     */
    // esp_err_t find(uint32_t id, ESPP::InfraRed::Code &code);

    /**
     * @brief Remove an IR code from the database.
     *
     * @param[in] id IR code ID.
     *
     * @return
     */
    esp_err_t remove(uint32_t id);

    /**
     * @brief Erase all codes form flash.
     */
    void erase();

    /**
     * @brief Check if key exists in flash.
     *
     * @param[in] key Key string.
     * @param[in] data Key Value.
     *
     * @return true if key already exists.
     */
    bool exist(char *key, uint8_t *data);

    // TODO: list codes;

private:
    nvs_handle m_status_nvs;
    nvs_handle m_otp_nvs;
    nvs_handle m_config_nvs;
    nvs_handle m_gsm_position_nvs;

    uint32_t m_position_id;
    uint32_t m_id;

    esp_err_t readVersion(int8_t &version);
    esp_err_t readCounter();
    esp_err_t writeCounter();
    esp_err_t readId();
    esp_err_t writeId();
};

} // namespace Storage
// #endif
