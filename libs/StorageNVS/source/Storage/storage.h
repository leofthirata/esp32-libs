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

#pragma once

#include <stdint.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_err.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "../../test/main/main.h"

/**
 * @brief Initialize the storage.
 *
 * @return
 */
esp_err_t storage_init();

/**
 * @brief Deinitialize the storage.
 *
 * @return
 */
esp_err_t storage_deinit();

/**
 * @brief Save Isca_t status in the databse.
 *
 * @param[in] status Isca_t status.
 *
 * @return
 */
void storage_status_set(Isca_t *status);

/**
 * @brief Read Isca_t status in the databse.
 *
 * @param[out] status Isca_t status.
 *
 * @return
 */
void storage_status_get(Isca_t *status);

/**
 * @brief Save Isca_t OTP keys in the databse.
 *
 * @param[in] otp Isca_t OTP keys.
 *
 * @return
 */
void storage_otp_set(Isca_t *otp);

/**
 * @brief Read Isca_t OTP keys in the databse.
 *
 * @param[out] otp Isca_t OTP keys.
 *
 * @return
 */
void storage_otp_get(Isca_t *otp);

/**
 * @brief Save Isca_t configurations in the databse.
 *
 * @param[in] config Isca_t configurations.
 *
 * @return
 */
void storage_config_set(Isca_t *config);

/**
 * @brief Read Isca_t configurations in the databse.
 *
 * @param[out] config Isca_t configurations.
 *
 * @return
 */
void storage_config_get(Isca_t *config);

/**
 * @brief Save BLE, LoRa and GSM configurations in the databse.
 *
 * @param[in] config BLE, LoRa and GSM configurations.
 *
 * @return
 */
void storage_ble_lora_gsm_set(Isca_t *config);

/**
 * @brief Read BLE, LoRa and GSM configurations.
 *
 * @param[out] config BLE, LoRa and GSM configurations.
 *
 * @return
 */
void storage_ble_lora_gsm_get(Isca_t *config);

/**
 * @brief Save a position in the database.
 *
 * @param[in] json Position message in json format.
 * @param[out] id Position id saved in nvs.
 *
 * @return
 */
esp_err_t storage_gsm_save_position(char *json, uint32_t *id);

/**
 * @brief Read the last position in the database. Updates the last position after reading.
 *
 * @param[out] json Last saved position message in json format.
 * @param[out] len Last saved position length.
 *
 * @return
 */
esp_err_t storage_gsm_get_last_position(char *json, size_t *len);

/**
 * @brief Read last position ID.
 *
 * @return
 */
esp_err_t storage_gsm_get_id(uint32_t *id);

/**
 * @brief Remove all keys from GSM namespace.
 *
 * @return
 */
esp_err_t storage_gsm_erase_nvs();