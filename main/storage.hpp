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
     * @brief Maximum number of stored positions.
     */
    static const uint32_t GSM_MAX_POSITIONS = 1000;

    /**
     * @brief Number of accelerometer axis.
     */
    static const uint8_t ACC_MAX_SIZE = 3;

    /**
     * @brief Firmware version string size.
     */
    static const uint8_t FW_VER_SIZE = 10;

    /**
     * @brief GSM APN size.
     */
    static const uint8_t GSM_APN_SIZE = 64;

    /**
     * @brief GSM User size.
     */
    static const uint8_t GSM_USER_SIZE = 64;

    /**
     * @brief GSM Password size.
     */
    static const uint8_t GSM_PSWD_SIZE = 64;

    /**
     * @brief GSM Server size.
     */
    static const uint8_t GSM_SERVER_SIZE = 64;

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
     * @brief Save Isca_t status in the databse.
     *
     * @param[in] status Isca_t status.
     *
     * @return
     */
    void status_set(Isca_t *status);

    /**
     * @brief Read Isca_t status in the databse.
     *
     * @param[out] status Isca_t status.
     *
     * @return
     */
    void status_get(Isca_t *status);

    /**
     * @brief Save Isca_t OTP keys in the databse.
     *
     * @param[in] otp Isca_t OTP keys.
     *
     * @return
     */
    void otp_set(Isca_t *otp);

    /**
     * @brief Read Isca_t OTP keys in the databse.
     *
     * @param[out] otp Isca_t OTP keys.
     *
     * @return
     */
    void otp_get(Isca_t *otp);

    /**
     * @brief Save Isca_t configurations in the databse.
     *
     * @param[in] config Isca_t configurations.
     *
     * @return
     */
    void config_set(Isca_t *config);

    /**
     * @brief Read Isca_t configurations in the databse.
     *
     * @param[out] config Isca_t configurations.
     *
     * @return
     */
    void config_get(Isca_t *config);

    /**
     * @brief Save BLE, LoRa and GSM configurations in the databse.
     *
     * @param[in] config BLE, LoRa and GSM configurations.
     *
     * @return
     */
    void ble_lora_gsm_set(Isca_t *config);

    /**
     * @brief Read BLE, LoRa and GSM configurations.
     *
     * @param[out] config BLE, LoRa and GSM configurations.
     *
     * @return
     */
    void ble_lora_gsm_get(Isca_t *config);

    /**
     * @brief Save a position in the database.
     *
     * @param[in] json Position message in json format.
     * @param[out] id Position id saved in nvs.
     *
     * @return
     */
    esp_err_t gsm_save_position(char *json, uint32_t *id);
    
    /**
     * @brief Read the last position in the database. Updates the last position after reading.
     *
     * @param[out] json Last saved position message in json format.
     * @param[out] len Last saved position length.
     *
     * @return
     */
    esp_err_t gsm_get_last_position(char *json, size_t *len);

    /**
     * @brief Remove all keys from GSM namespace.
     *
     * @return
     */
    esp_err_t gsm_erase_nvs();

private:
    nvs_handle m_status_nvs;
    nvs_handle m_otp_nvs;
    nvs_handle m_config_nvs;
    nvs_handle m_gsm_position_nvs;

    uint32_t m_id;

    esp_err_t gsm_save_id();
    esp_err_t set_id(uint32_t *id);
    esp_err_t get_id(uint32_t *id);

    esp_err_t set_key_u8(nvs_handle_t ns, const char *key, uint8_t *data, size_t size);
    esp_err_t set_key_u16(nvs_handle_t ns, const char *key, uint16_t *data);
    esp_err_t set_key_u32(nvs_handle_t ns, const char *key, uint32_t *data);
    esp_err_t set_key_u64(nvs_handle_t ns, const char *key, uint64_t *data);
    esp_err_t set_key_u32_20bits(nvs_handle_t ns, const char *key, uint32_t *data);

    esp_err_t get_key_u8(nvs_handle_t ns, const char *key, uint8_t *data, size_t size);
    esp_err_t get_key_u16(nvs_handle_t ns, const char *key, uint16_t *data);
    esp_err_t get_key_u32(nvs_handle_t ns, const char *key, uint32_t *data);
    esp_err_t get_key_u64(nvs_handle_t ns, const char *key, uint64_t *data);
    esp_err_t get_key_u32_20bits(nvs_handle_t ns, const char *key, uint32_t *data);
};

} // namespace Storage
