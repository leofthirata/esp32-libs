#ifndef __STORAGE_NVS_H__
#define __STORAGE_NVS_H__

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
#ifdef __cplusplus
extern "C" {
#endif
#pragma once

#include <stdint.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_err.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "main.h"

#define GSM_PAYLOAD_MAX_SIZE ESP_MODEM_C_API_STR_MAX

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
 * @brief Save Isca_t OTP keys in the databse.
 *
 * @param[in] otp Isca_t OTP keys.
 *
 * @return
 */
esp_err_t storage_otp_set(IscaROM_t *otp);

/**
 * @brief Read Isca_t OTP keys in the databse.
 *
 * @param[out] otp Isca_t OTP keys.
 *
 * @return
 */
esp_err_t storage_otp_get(IscaROM_t *otp);

/**
 * @brief Save Isca_t configurations in the databse.
 *
 * @param[in] config Isca_t configurations.
 *
 * @return
 */
esp_err_t storage_config_set(IscaConfig_t *config);

/**
 * @brief Read Isca_t configurations in the databse.
 *
 * @param[out] config Isca_t configurations.
 *
 * @return
 */
esp_err_t storage_config_get(IscaConfig_t *config);

/**
 * @brief Save BLE, LoRa and GSM configurations in the databse.
 *
 * @param[in] config BLE, LoRa and GSM configurations.
 *
 * @return
 */
esp_err_t storage_timers_set(IscaTimers_t *timers);

/**
 * @brief Read BLE, LoRa and GSM configurations.
 *
 * @param[out] config BLE, LoRa and GSM configurations.
 *
 * @return
 */
esp_err_t storage_timers_get(IscaTimers_t *timers);

/**
 * @brief Save a position in the database.
 *
 * @param[in] json Position message in json format.
 * @param[out] id Position id saved in nvs.
 *
 * @return
 */
esp_err_t storage_gsm_save_position(char *str, uint32_t *id);

/**
 * @brief Read the last position in the database. Updates the last position after reading.
 *
 * @param[out] json Last saved position message in json format.
 * @param[out] len Last saved position length.
 *
 * @return
 */
esp_err_t storage_gsm_get_last_position(char *str, size_t *len);

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

#ifdef __cplusplus
}
#endif
#endif //__STORAGE_NVS_H__