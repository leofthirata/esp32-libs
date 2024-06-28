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
#include "storage.hpp"

#include <cctype>
#include <cstdlib>
#include <string.h>

#include "esp_log.h"

namespace Storage
{

static const char *TAG = "Storage";

Storage::Storage()
{
    m_status_nvs = 0;
    m_otp_nvs = 0;
    m_config_nvs = 0;
    m_gsm_position_nvs = 0;

    m_id = 0;
}

esp_err_t Storage::init()
{
    esp_err_t err = ESP_OK;

    err = nvs_open("status", NVS_READWRITE, &m_status_nvs);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed open 'status' namespace");
        return err;
    }

    err = nvs_open("otp", NVS_READWRITE, &m_otp_nvs);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed open 'otp' namespace");
        return err;
    }

    err = nvs_open("config", NVS_READWRITE, &m_config_nvs);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed open 'config' namespace");
        return err;
    }

    err = nvs_open("gsm_position", NVS_READWRITE, &m_gsm_position_nvs);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed open 'gsm_position' namespace");
        return err;
    }

    err = get_id(&m_id);
    if (err == ESP_ERR_NVS_NOT_FOUND) // if no id found, save init value
        err = set_id(&m_id);

    return err;
}

esp_err_t Storage::deinit()
{
    nvs_close(m_status_nvs);
    nvs_close(m_otp_nvs);
    nvs_close(m_config_nvs);
    nvs_close(m_gsm_position_nvs);

    return ESP_OK;
}

void Storage::otp_set(Isca_t *otp)
{
    set_key_u8(m_otp_nvs, "hwver", &otp->hwVer, 1);

    uint8_t loraId[3];
    loraId[0] = (otp->loraId >> 16);
    loraId[1] = (otp->loraId >> 8) & 0xFF;
    loraId[2] = (otp->loraId & 0xFF);

    uint8_t devAddr[4];
    devAddr[0] = (otp->nodeDevAddr >> 24);
    devAddr[1] = (otp->nodeDevAddr >> 16) & 0xFF;
    devAddr[2] = (otp->nodeDevAddr >> 8) & 0xFF;
    devAddr[3] = (otp->nodeDevAddr & 0xFF);

    set_key_u8(m_otp_nvs, "blemac", otp->bleMac, 6);
    set_key_u8(m_otp_nvs, "loraid", loraId, 3);
    set_key_u8(m_otp_nvs, "devaddr", devAddr, 4);
    set_key_u8(m_otp_nvs, "deveui", otp->nodeDeviceEUI, 8);
    set_key_u8(m_otp_nvs, "appeui", otp->nodeAppEUI, 8);
    set_key_u8(m_otp_nvs, "nwskey", otp->nodeNwsKey, 16);
    set_key_u8(m_otp_nvs, "appskey", otp->nodeAppsKey, 16);
}

void Storage::otp_get(Isca_t *otp)
{
    get_key_u8(m_otp_nvs, "hwver", &otp->hwVer, 1);

    uint8_t bleMac[6];
    uint8_t loraId[3];
    uint8_t devAddr[4];
    uint8_t nodeDeviceEUI[8];
    uint8_t nodeAppEUI[8];
    uint8_t nodeNwsKey[16];
    uint8_t nodeAppsKey[16];

    get_key_u8(m_otp_nvs, "blemac", bleMac, 6);
    get_key_u8(m_otp_nvs, "loraid", loraId, 3);
    get_key_u8(m_otp_nvs, "devaddr", devAddr, 4);
    get_key_u8(m_otp_nvs, "deveui", nodeDeviceEUI, 8);
    get_key_u8(m_otp_nvs, "appeui", nodeAppEUI, 8);
    get_key_u8(m_otp_nvs, "nwskey", nodeNwsKey, 16);
    get_key_u8(m_otp_nvs, "appskey", nodeAppsKey, 16);

    memcpy(otp->bleMac, bleMac, 6);
    otp->loraId = (loraId[0]<<16) + (loraId[1]<<8) + (loraId[2]);
    otp->nodeDevAddr = (devAddr[0]<<24) + (devAddr[1]<<16) +
        (devAddr[2]<<8) + devAddr[3];
    memcpy(otp->nodeDeviceEUI, nodeDeviceEUI, 8);
    memcpy(otp->nodeAppEUI, nodeDeviceEUI, 8);
    memcpy(otp->nodeNwsKey, nodeNwsKey, 16);
    memcpy(otp->nodeAppsKey, nodeAppsKey, 16);
}

void Storage::status_set(Isca_t *status)
{
    set_key_u16(m_status_nvs, "flags", (uint16_t *)&status->flags);

    esp_err_t err = nvs_erase_key(m_status_nvs, "battmv");
    int16_t aux16 = 0;
    err = nvs_get_i16(m_status_nvs, "battmv", &aux16);
    if (err == ESP_ERR_NVS_NOT_FOUND)
        ESP_LOGW("battmv", "not found in NVS, setting key. Error (%s)\n", esp_err_to_name(err));
    err = nvs_set_i16(m_status_nvs, "battmv", status->batteryMiliVolts);

    err = nvs_erase_key(m_status_nvs, "tempC");
    int8_t aux8 = 0;
    err = nvs_get_i8(m_status_nvs, "tempC", &aux8);
    if (err == ESP_ERR_NVS_NOT_FOUND)
        ESP_LOGW("tempC", "not found in NVS, setting key. Error (%s)\n", esp_err_to_name(err));
    err = nvs_set_i8(m_status_nvs, "tempC", status->temperatureCelsius);

    set_key_u32(m_status_nvs, "reset", &status->reset);
    set_key_u8(m_status_nvs, "battstatus", &status->batteryStatus, 1);
    set_key_u64(m_status_nvs, "lastP2PTick", &status->lastP2PTick);
    set_key_u64(m_status_nvs, "lastLRWTick", &status->lastLRWTick);
    set_key_u8(m_status_nvs, "P2PCount", &status->P2PCount, 1);

    err = nvs_erase_key(m_status_nvs, "acc");
    int32_t aux32[ACC_MAX_SIZE];
    err = nvs_get_blob(m_status_nvs, "acc", aux32, NULL);
    if (err == ESP_ERR_NVS_NOT_FOUND) // ESP_ERR_NVS_NOT_FOUND error occurs if key doesn't exist yet
        ESP_LOGW("acc", "not found in NVS, setting key. Error (%s)\n", esp_err_to_name(err));
    err = nvs_set_blob(m_status_nvs, "acc", status->acc, ACC_MAX_SIZE*sizeof(int32_t));

    err = nvs_erase_key(m_status_nvs, "temp");
    float temp;
    err = nvs_get_blob(m_status_nvs, "temp", &temp, NULL);
    if (err == ESP_ERR_NVS_NOT_FOUND) // ESP_ERR_NVS_NOT_FOUND error occurs if key doesn't exist yet
        ESP_LOGW("temp", "not found in NVS, setting key. Error (%s)\n", esp_err_to_name(err));
    err = nvs_set_blob(m_status_nvs, "temp", &status->temp, sizeof(float));

    nvs_commit(m_status_nvs);
}

void Storage::status_get(Isca_t *status)
{
    get_key_u16(m_status_nvs, "flags", (uint16_t *)&status->flags);
    nvs_get_i16(m_status_nvs, "battmv", &status->batteryMiliVolts);
    nvs_get_i8(m_status_nvs, "tempC", &status->temperatureCelsius);
    get_key_u32(m_status_nvs, "reset", &status->reset);
    get_key_u8(m_status_nvs, "battstatus", &status->batteryStatus, 1);
    get_key_u64(m_status_nvs, "lastP2PTick", &status->lastP2PTick);
    get_key_u64(m_status_nvs, "lastLRWTick", &status->lastLRWTick);
    get_key_u8(m_status_nvs, "P2PCount", &status->P2PCount, 1);

    size_t length = ACC_MAX_SIZE * sizeof(int32_t); // length must have the correct key size to avoid ESP_ERR_NVS_INVALID_LENGTH error
    int32_t temp[ACC_MAX_SIZE];
    nvs_get_blob(m_status_nvs, "acc", temp, &length);
    memcpy((int32_t *)status->acc, temp, length);

    length = sizeof(float);
    nvs_get_blob(m_status_nvs, "temp", &status->temp, &length);
}

void Storage::config_set(Isca_t *config)
{
    esp_err_t err = nvs_erase_key(m_status_nvs, "fwver");

    char *blob = (char *)malloc(FW_VER_SIZE*sizeof(char));
    memset(blob, 1, FW_VER_SIZE*sizeof(char));

    err = nvs_get_str(m_config_nvs, "fwver", blob, NULL);
    if (err == ESP_ERR_NVS_NOT_FOUND) // ESP_ERR_NVS_NOT_FOUND error occurs if key doesn't exist yet
    {
        ESP_LOGW("fwver", "not found in NVS, setting key. Error (%s)\n", esp_err_to_name(err));
        err = nvs_set_str(m_config_nvs, "fwver", config->fwVer);
    }
    free(blob);

    err = nvs_commit(m_config_nvs);

    set_key_u8(m_config_nvs, "fwverprot", &config->fwVerProtocol, 1);
    set_key_u8(m_config_nvs, "lrwprot", &config->lrwProtocol, 1);
    set_key_u8(m_config_nvs, "apn", (uint8_t *)config->apn, GSM_APN_SIZE);
    set_key_u8(m_config_nvs, "gsmuser", (uint8_t *)config->gsmUser, GSM_USER_SIZE);
    set_key_u8(m_config_nvs, "gsmpswd", (uint8_t *)config->gsmPswd, GSM_PSWD_SIZE);

    set_key_u16(m_config_nvs, "gsmport", &config->gsmPort);

    set_key_u8(m_config_nvs, "gsmserver", (uint8_t *)config->gsmServer, GSM_SERVER_SIZE);

    set_key_u32(m_config_nvs, "p2ptxfreq", &config->p2pTXFreq);
    set_key_u32(m_config_nvs, "p2prxfreq", &config->p2pRXFreq);

    set_key_u8(m_config_nvs, "p2pbw", &config->p2pBW, 1);
    set_key_u8(m_config_nvs, "p2psf", &config->p2pSF, 1);
    set_key_u8(m_config_nvs, "p2pcr", &config->p2pCR, 1);
    set_key_u8(m_config_nvs, "lrwsf", &config->lrwSF, 1);

    uint8_t temp = config->lrwADR ? 1 : 0;
    set_key_u8(m_config_nvs, "lrwadr", &temp, 1);
}

void Storage::config_get(Isca_t *config)
{
    size_t length = FW_VER_SIZE * sizeof(char); // length must have the correct key size to avoid ESP_ERR_NVS_INVALID_LENGTH error
    char tempStr[FW_VER_SIZE];
    nvs_get_str(m_config_nvs, "fwver", tempStr, &length);
    config->fwVer = (char *)malloc(FW_VER_SIZE*sizeof(char));
    memcpy(config->fwVer, tempStr, length);

    get_key_u8(m_config_nvs, "fwverprot", &config->fwVerProtocol, 1);
    get_key_u8(m_config_nvs, "lrwprot", &config->lrwProtocol, 1);

    uint8_t temp[64];

    get_key_u8(m_config_nvs, "apn", temp, 64);
    memcpy(config->apn, temp, 64);
    
    get_key_u8(m_config_nvs, "gsmuser", temp, 64);
    memcpy(config->gsmUser, temp, 64);
    get_key_u8(m_config_nvs, "gsmpswd", temp, 64);
    memcpy(config->gsmPswd, temp, 64);

    get_key_u16(m_config_nvs, "gsmport", &config->gsmPort);

    get_key_u8(m_config_nvs, "gsmserver", temp, 64);
    memcpy(config->gsmServer, temp, 64);

    get_key_u32(m_config_nvs, "p2ptxfreq", &config->p2pTXFreq);
    get_key_u32(m_config_nvs, "p2prxfreq", &config->p2pRXFreq);

    get_key_u8(m_config_nvs, "p2pbw", &config->p2pBW, 1);
    get_key_u8(m_config_nvs, "p2psf", &config->p2pSF, 1);
    get_key_u8(m_config_nvs, "p2pcr", &config->p2pCR, 1);
    get_key_u8(m_config_nvs, "lrwsf", &config->lrwSF, 1);

    uint8_t aux = config->lrwADR ? 1 : 0;
    get_key_u8(m_config_nvs, "lrwadr", &aux, 1);
}

void Storage::ble_lora_gsm_set(Isca_t *config)
{
    set_key_u8(m_config_nvs, "blepower", &config->blePower, 1);
    set_key_u16(m_config_nvs, "bleadvtime", &config->bleAdvTime);

    uint32_t temp;

    temp = config->p2pMovNorm;
    set_key_u32_20bits(m_config_nvs, "p2pmovnorm", &temp);
    temp = config->p2pMovEmer;
    set_key_u32_20bits(m_config_nvs, "p2pmovemer", &temp);
    temp = config->p2pStpNorm;
    set_key_u32_20bits(m_config_nvs, "p2pstpnorm", &temp);
    temp = config->p2pStpEmer;
    set_key_u32_20bits(m_config_nvs, "p2pstpemer", &temp);
    temp = config->lrwMovNorm;
    set_key_u32_20bits(m_config_nvs, "lrwmovnorm", &temp);
    temp = config->lrwMovEmer;
    set_key_u32_20bits(m_config_nvs, "lrwmovemer", &temp);
    temp = config->lrwStpNorm;
    set_key_u32_20bits(m_config_nvs, "lrwstpnorm", &temp);
    temp = config->lrwStpEmer;
    set_key_u32_20bits(m_config_nvs, "lrwstpemer", &temp);
    temp = config->gsmMovNorm;
    set_key_u32_20bits(m_config_nvs, "gsmmovnorm", &temp);
    temp = config->gsmMovEmer;
    set_key_u32_20bits(m_config_nvs, "gsmmovemer", &temp);
    temp = config->gsmStpNorm;
    set_key_u32_20bits(m_config_nvs, "gsmstpnorm", &temp);
    temp = config->gsmStpEmer;
    set_key_u32_20bits(m_config_nvs, "gsmstpemer", &temp);
}

void Storage::ble_lora_gsm_get(Isca_t *config)
{
    get_key_u8(m_config_nvs, "blepower", &config->blePower, 1);
    get_key_u16(m_config_nvs, "bleadvtime", &config->bleAdvTime);

    uint32_t temp;

    get_key_u32_20bits(m_config_nvs, "p2pmovnorm", &temp);
    config->p2pMovNorm = temp;
    get_key_u32_20bits(m_config_nvs, "p2pmovemer", &temp);
    config->p2pMovEmer = temp;
    get_key_u32_20bits(m_config_nvs, "p2pstpnorm", &temp);
    config->p2pStpNorm = temp;
    get_key_u32_20bits(m_config_nvs, "p2pstpemer", &temp);
    config->p2pStpEmer = temp;
    get_key_u32_20bits(m_config_nvs, "lrwmovnorm", &temp);
    config->lrwMovNorm = temp;
    get_key_u32_20bits(m_config_nvs, "lrwmovemer", &temp);
    config->lrwMovEmer = temp;
    get_key_u32_20bits(m_config_nvs, "lrwstpnorm", &temp);
    config->lrwStpNorm = temp;
    get_key_u32_20bits(m_config_nvs, "lrwstpemer", &temp);
    config->lrwStpEmer = temp;
    get_key_u32_20bits(m_config_nvs, "gsmmovnorm", &temp);
    config->gsmMovNorm = temp;
    get_key_u32_20bits(m_config_nvs, "gsmmovemer", &temp);
    config->gsmMovEmer = temp;
    get_key_u32_20bits(m_config_nvs, "gsmstpnorm", &temp);
    config->gsmStpNorm = temp;
    get_key_u32_20bits(m_config_nvs, "gsmstpemer", &temp);
    config->gsmStpEmer = temp;
}

esp_err_t Storage::gsm_save_position(char *json, uint32_t *id)
{
    if (m_id >= GSM_MAX_POSITIONS)
        return ESP_ERR_NO_MEM;

    uint32_t temp = 0;

    get_id(&temp);

    temp++;

    set_id(&temp);

    char key[9];
    sprintf(key, "%08lx", temp);

    char *blob = (char *)malloc(FW_VER_SIZE*sizeof(char));
    memset(blob, 1, FW_VER_SIZE*sizeof(char));

    esp_err_t err = nvs_get_str(m_gsm_position_nvs, key, blob, NULL);
    if (err == ESP_ERR_NVS_NOT_FOUND) // ESP_ERR_NVS_NOT_FOUND error occurs if key doesn't exist yet
        err = nvs_set_str(m_gsm_position_nvs, key, json);
    free(blob);

    err = nvs_commit(m_gsm_position_nvs);

    *id = temp;

    return err;
}

esp_err_t Storage::gsm_get_last_position(char *json, size_t *len)
{
    char key[9];
    uint32_t id;

    get_id(&id);

    if (id > 0)
    {
        sprintf(key, "%08lx", id);

        id--;
        set_id(&id);

        nvs_get_str(m_gsm_position_nvs, key, json, len);

        return nvs_erase_key(m_gsm_position_nvs, key);
    }
    else
        return ESP_FAIL;
}

esp_err_t Storage::gsm_save_id()
{
    uint32_t temp = 0;

    nvs_erase_key(m_gsm_position_nvs, "id");

    esp_err_t err = nvs_get_u32(m_gsm_position_nvs, "id", &temp);
    
    err = nvs_set_u32(m_gsm_position_nvs, "id", m_id);

    return nvs_commit(m_gsm_position_nvs);
}

esp_err_t Storage::set_id(uint32_t *id)
{
    m_id = *id;
    esp_err_t err = gsm_save_id();
    printf("set_id: %ld\r\n", m_id);

    return err;
}

esp_err_t Storage::get_id(uint32_t *id)
{
    uint32_t temp = 0;
    esp_err_t err = nvs_get_u32(m_gsm_position_nvs, "id", &temp);
    *id = temp;
    printf("get_id: %ld\r\n", temp);

    return err;
}

esp_err_t Storage::gsm_erase_nvs()
{
    return nvs_erase_all(m_gsm_position_nvs);
}

esp_err_t Storage::set_key_u8(nvs_handle_t ns, const char *key, uint8_t *data, size_t size)
{
    esp_err_t err = nvs_erase_key(ns, key);

    if (size > 1)
    {
        uint8_t *blob = (uint8_t *)malloc(size*sizeof(uint8_t));
        memset(blob, 1, size*sizeof(uint8_t));

        if (blob == nullptr)
            return ESP_ERR_NO_MEM;
    
        err = nvs_get_blob(ns, key, blob, NULL);
        if (err == ESP_ERR_NVS_NOT_FOUND) // ESP_ERR_NVS_NOT_FOUND error occurs if key doesn't exist yet
        {
            ESP_LOGW(key, "not found in NVS, setting key. Error (%s)\n", esp_err_to_name(err));
            err = nvs_set_blob(ns, key, data, size);
        }
        free(blob);
    }
    else
    {
        uint8_t temp = 0;

        err = nvs_get_u8(ns, key, &temp);
        if (err == ESP_ERR_NVS_NOT_FOUND)
        {
            ESP_LOGW(key, "not found in NVS, setting key. Error (%s)\n", esp_err_to_name(err));

            err = nvs_set_u8(ns, key, *data);
        }
    }

    if (err != ESP_OK) 
    {
        ESP_LOGW(key, "failed to set key. Error (%s)\n", esp_err_to_name(err));
        return err;
    }

    err = nvs_commit(ns);
    
    if (err != ESP_OK) 
        ESP_LOGW(key, "commiting failed. Error (%s)\n", esp_err_to_name(err));

    return err;
}

esp_err_t Storage::set_key_u16(nvs_handle_t ns, const char *key, uint16_t *data)
{
    uint8_t temp[2];
    temp[0] = *data >> 8;
    temp[1] = *data & 0xFF;

    return set_key_u8(ns, key, temp, 2);
}

esp_err_t Storage::set_key_u32(nvs_handle_t ns, const char *key, uint32_t *data)
{
    uint8_t temp[4];
    temp[0] = *data >> 24;
    temp[1] = (*data >> 16) & 0xFF;
    temp[2] = (*data >> 8) & 0xFF;
    temp[3] = *data & 0xFF;

    return set_key_u8(ns, key, temp, 4);
}

esp_err_t Storage::set_key_u64(nvs_handle_t ns, const char *key, uint64_t *data)
{
    uint8_t temp[8];
    temp[0] = *data >> 56;
    temp[1] = (*data >> 48) & 0xFF;
    temp[2] = (*data >> 40) & 0xFF;
    temp[3] = (*data >> 32) & 0xFF;
    temp[4] = (*data >> 24) & 0xFF;
    temp[5] = (*data >> 16) & 0xFF;
    temp[6] = (*data >> 8) & 0xFF;
    temp[7] = *data & 0xFF;

    return set_key_u8(ns, key, temp, 8);
}

esp_err_t Storage::set_key_u32_20bits(nvs_handle_t ns, const char *key, uint32_t *data)
{
    uint8_t temp[3];
    temp[0] = (*data >> 16) & 0x0F;
    temp[1] = (*data >> 8) & 0xFF;
    temp[2] = *data & 0xFF;

    return set_key_u8(ns, key, temp, 3);
}

esp_err_t Storage::get_key_u8(nvs_handle_t ns, const char *key, uint8_t *data, size_t size)
{
    esp_err_t err;
    size_t length = size; // length must have the correct key size to avoid ESP_ERR_NVS_INVALID_LENGTH error

    if (size > 1)
        err = nvs_get_blob(ns, key, data, &length);
    else
        err = nvs_get_u8(ns, key, data);
    if (err != ESP_OK) 
        ESP_LOGE(key, "Error (%s)\n", esp_err_to_name(err));

    return err;
}

esp_err_t Storage::get_key_u16(nvs_handle_t ns, const char *key, uint16_t *data)
{
    uint8_t temp[2];

    esp_err_t err = get_key_u8(ns, key, temp, 2);
    
    *data = (temp[0] << 8) + temp[1];

    return err;
}

esp_err_t Storage::get_key_u32(nvs_handle_t ns, const char *key, uint32_t *data)
{
    uint8_t temp[4];

    esp_err_t err = get_key_u8(ns, key, temp, 4);

    *data = (temp[0] << 24) + (temp[1] << 16) + (temp[2] << 8) + temp[3];

    return err;
}

esp_err_t Storage::get_key_u64(nvs_handle_t ns, const char *key, uint64_t *data)
{
    uint8_t temp[8];

    esp_err_t err = get_key_u8(ns, key, temp, 8);

    *data = ((uint64_t)temp[0] << 56) + ((uint64_t)temp[1] << 48) + ((uint64_t)temp[2] << 40) + ((uint64_t)temp[3] << 32) +
        ((uint64_t)temp[4] << 24) + ((uint64_t)temp[5] << 16) + ((uint64_t)temp[6] << 8) + temp[7];

    return err;
}

esp_err_t Storage::get_key_u32_20bits(nvs_handle_t ns, const char *key, uint32_t *data)
{
    uint8_t temp[3];

    esp_err_t err = get_key_u8(ns, key, temp, 3);

    *data = (temp[0] << 16) + (temp[1] << 8) + (temp[2]);

    return err;
}

} // namespace Storage
