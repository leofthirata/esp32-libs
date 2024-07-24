
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
#include "memory.h"
#include "esp_err.h"

static const char *TAG = "Storage";

/**
 * @brief Maximum number of stored positions.
 */
static const uint32_t GSM_MAX_POSITIONS = 1000;

/**
 * @brief Firmware version string size.
 */
static const uint8_t FW_VER_SIZE = 15;

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

static nvs_handle m_otp_nvs;
static nvs_handle m_config_nvs;
static nvs_handle m_timers_nvs;
static nvs_handle m_gsm_position_nvs;

static uint32_t m_id;

static esp_err_t gsm_save_id();
static esp_err_t set_id(uint32_t *id);
static esp_err_t get_id(uint32_t *id);

static esp_err_t set_key_u8(nvs_handle_t ns, const char *key, uint8_t *data, size_t size);
static esp_err_t set_key_u16(nvs_handle_t ns, const char *key, uint16_t *data);
static esp_err_t set_key_u32(nvs_handle_t ns, const char *key, uint32_t *data);
static esp_err_t set_key_u64(nvs_handle_t ns, const char *key, uint64_t *data);
static esp_err_t set_key_u32_20bits(nvs_handle_t ns, const char *key, uint32_t *data);

static esp_err_t get_key_u8(nvs_handle_t ns, const char *key, uint8_t *data, size_t size);
static esp_err_t get_key_u16(nvs_handle_t ns, const char *key, uint16_t *data);
static esp_err_t get_key_u32(nvs_handle_t ns, const char *key, uint32_t *data);
static esp_err_t get_key_u64(nvs_handle_t ns, const char *key, uint64_t *data);
static esp_err_t get_key_u32_20bits(nvs_handle_t ns, const char *key, uint32_t *data);

esp_err_t storage_init()
{
    m_otp_nvs = 0;
    m_config_nvs = 0;
    m_gsm_position_nvs = 0;
    m_timers_nvs = 0;

    m_id = 0;

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = get_id(&m_id);
    if (err == ESP_ERR_NVS_NOT_FOUND) // if no id found, save init value
        err = set_id(&m_id);

    return err;
}

esp_err_t storage_deinit()
{

    return ESP_OK;
}

esp_err_t storage_otp_set(IscaROM_t *otp)
{
    esp_err_t err = nvs_open("otp", NVS_READWRITE, &m_otp_nvs);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed open 'otp' namespace");
        return err;
    }

    set_key_u8(m_otp_nvs, "hwver", &otp->hwVer, 1);

    uint8_t loraId[3];
    loraId[0] = (otp->loraId >> 16);
    loraId[1] = (otp->loraId >> 8) & 0xFF;
    loraId[2] = (otp->loraId & 0xFF);

    uint8_t devAddr[4];
    devAddr[0] = (otp->devAddr >> 24);
    devAddr[1] = (otp->devAddr >> 16) & 0xFF;
    devAddr[2] = (otp->devAddr >> 8) & 0xFF;
    devAddr[3] = (otp->devAddr & 0xFF);

    set_key_u8(m_otp_nvs, "blemac", otp->bleMac, 6);
    set_key_u8(m_otp_nvs, "imei", otp->imei, 7);
    set_key_u8(m_otp_nvs, "loraid", loraId, 3);
    set_key_u8(m_otp_nvs, "devaddr", devAddr, 4);
    set_key_u8(m_otp_nvs, "deveui", otp->deviceEUI, 8);
    set_key_u8(m_otp_nvs, "appeui", otp->appEUI, 8);
    set_key_u8(m_otp_nvs, "nwkSkey", otp->nwkSKey, 16);
    set_key_u8(m_otp_nvs, "appSkey", otp->appSKey, 16);

    nvs_close(m_otp_nvs);
    return ESP_OK;
}

esp_err_t storage_otp_get(IscaROM_t *otp)
{
    if(otp == NULL)
        return ESP_ERR_INVALID_ARG;
    
    esp_err_t err = nvs_open("otp", NVS_READWRITE, &m_otp_nvs);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed open 'otp' namespace");
        return err;
    }
    
    get_key_u8(m_otp_nvs, "hwver", &otp->hwVer, 1);

    uint8_t bleMac[6];
    uint8_t loraId[3];
    uint8_t devAddr[4];
    uint8_t nodeDeviceEUI[8];
    uint8_t nodeAppEUI[8];
    uint8_t nodeNwsKey[16];
    uint8_t nodeAppsKey[16];
    uint8_t imei[7];

    get_key_u8(m_otp_nvs, "blemac", bleMac, 6);
    get_key_u8(m_otp_nvs, "loraid", loraId, 3);
    get_key_u8(m_otp_nvs, "devaddr", devAddr, 4);
    get_key_u8(m_otp_nvs, "deveui", nodeDeviceEUI, 8);
    get_key_u8(m_otp_nvs, "appeui", nodeAppEUI, 8);
    get_key_u8(m_otp_nvs, "nwkSkey", nodeNwsKey, 16);
    get_key_u8(m_otp_nvs, "appSkey", nodeAppsKey, 16);
    get_key_u8(m_otp_nvs, "imei", imei, 7);

    memcpy(otp->bleMac, bleMac, 6);
    otp->loraId = (loraId[0]<<16) + (loraId[1]<<8) + (loraId[2]);
    otp->devAddr = (devAddr[0]<<24) + (devAddr[1]<<16) +
        (devAddr[2]<<8) + devAddr[3];
    memcpy(otp->deviceEUI, nodeDeviceEUI, 8);
    memcpy(otp->appEUI, nodeDeviceEUI, 8);
    memcpy(otp->nwkSKey, nodeNwsKey, 16);
    memcpy(otp->appSKey, nodeAppsKey, 16);
    memcpy(otp->imei, imei, 7);
    
    nvs_close(m_otp_nvs);
    return ESP_OK;
}

esp_err_t storage_config_set(IscaConfig_t *config)
{
    esp_err_t err = nvs_open("config", NVS_READWRITE, &m_config_nvs);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed open 'config' namespace");
        return err;
    }

    /* IscaFWConfig_t  */
    err = nvs_erase_key(m_config_nvs, "fwver");

    // char *blob = (char *)malloc(FW_VER_SIZE*sizeof(char));
    // memset(blob, 1, FW_VER_SIZE*sizeof(char));
    char str[FW_VER_SIZE];

    err = nvs_get_str(m_config_nvs, "fwver", str, NULL);
    if (err == ESP_ERR_NVS_NOT_FOUND) // ESP_ERR_NVS_NOT_FOUND error occurs if key doesn't exist yet
    {
        ESP_LOGW("fwver", "not found in NVS, setting key. Error (%s)\n", esp_err_to_name(err));
        err = nvs_set_str(m_config_nvs, "fwver", config->fw.versionString);
    }
    // free(blob);

    err = nvs_commit(m_config_nvs);

    set_key_u8(m_config_nvs, "fwverprot", &config->fw.versionByte, 1);

    /* IscaLRWConfig_t */
    set_key_u8(m_config_nvs, "lrwprot", &config->lrw.protocol, 1);
    set_key_u8(m_config_nvs, "lrwsf", &config->lrw.sf, 1);
    
    uint8_t _adr = config->lrw.adr ? 1 : 0;
    set_key_u8(m_config_nvs, "lrwadr", &_adr, 1);

    uint8_t _confirmed = config->lrw.confirmed ? 1 : 0;
    set_key_u8(m_config_nvs, "lrwconfirmed", &_confirmed, 1);

    set_key_u8(m_config_nvs, "lrwposport", &config->lrw.posPort, 1);
    set_key_u8(m_config_nvs, "lrwcmdport", &config->lrw.cmdPort, 1);
    set_key_u8(m_config_nvs, "lrwstsport", &config->lrw.statusPort, 1);
    
    /* IscaGSMConfig_t */
    set_key_u8(m_config_nvs, "apn", (uint8_t *)config->gsm.apn, GSM_APN_SIZE);
    set_key_u8(m_config_nvs, "gsmuser", (uint8_t *)config->gsm.user, GSM_USER_SIZE);
    set_key_u8(m_config_nvs, "gsmpswd", (uint8_t *)config->gsm.pswd, GSM_PSWD_SIZE);

    set_key_u16(m_config_nvs, "gsmport", &config->gsm.port);

    set_key_u8(m_config_nvs, "gsmserver", (uint8_t *)config->gsm.server, GSM_SERVER_SIZE);

    /* IscaP2PConfig_t */
    set_key_u8(m_config_nvs, "p2psf", &config->p2p.sf, 1);
    set_key_u8(m_config_nvs, "p2pbw", &config->p2p.bw, 1);
    set_key_u8(m_config_nvs, "p2pcr", &config->p2p.cr, 1);
    set_key_u32(m_config_nvs, "p2ptxfreq", &config->p2p.txFreq);
    set_key_u32(m_config_nvs, "p2ptxtime", &config->p2p.txFreq);
    set_key_u8(m_config_nvs, "p2ptxpwr", &config->p2p.txPower, 1);
    set_key_u32(m_config_nvs, "p2prxfreq", &config->p2p.rxFreq);
    set_key_u32(m_config_nvs, "p2prxtime", &config->p2p.rxTimeout);
    set_key_u32(m_config_nvs, "p2prxdelay", &config->p2p.rxDelay);

    set_key_u8(m_config_nvs, "blepwr", &config->ble.power, 1);
    nvs_close(m_config_nvs);

    return ESP_OK;
}

esp_err_t storage_config_get(IscaConfig_t *config)
{
    esp_err_t err = nvs_open("config", NVS_READWRITE, &m_config_nvs);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed open 'config' namespace");
        return err;
    }
    // size_t length = FW_VER_SIZE * sizeof(char); // length must have the correct key size to avoid ESP_ERR_NVS_INVALID_LENGTH error
    char tempStr[FW_VER_SIZE];
    size_t length = sizeof(tempStr);
    nvs_get_str(m_config_nvs, "fwver", tempStr, &length);
    // config->fwVer = (char *)malloc(FW_VER_SIZE*sizeof(char));
    memcpy(config->fw.versionString, tempStr, length);

    get_key_u8(m_config_nvs, "fwverprot", &config->fw.versionByte, 1);
    

    get_key_u8(m_config_nvs, "lrwprot", &config->lrw.protocol, 1);

    get_key_u8(m_config_nvs, "lrwsf", &config->lrw.sf, 1);

    uint8_t aux = 0;
    get_key_u8(m_config_nvs, "lrwadr", &aux, 1);
    if(aux == 1)
        config->lrw.adr = 1;
    else 
        config->lrw.adr = 0;
    
    aux = 0;

    get_key_u8(m_config_nvs, "lrwconfirmed", &aux, 1);
    if(aux == 1)
        config->lrw.confirmed = 1;
    else 
        config->lrw.confirmed = 0;

    get_key_u8(m_config_nvs, "lrwposport", &config->lrw.posPort, 1);
    get_key_u8(m_config_nvs, "lrwcmdport", &config->lrw.cmdPort, 1);
    get_key_u8(m_config_nvs, "lrwstsport", &config->lrw.statusPort, 1);
    
    uint8_t temp[64];

    get_key_u8(m_config_nvs, "apn", temp, 64);
    memcpy(config->gsm.apn, temp, 64);
    
    get_key_u8(m_config_nvs, "gsmuser", temp, 64);
    memcpy(config->gsm.user, temp, 64);
    get_key_u8(m_config_nvs, "gsmpswd", temp, 64);
    memcpy(config->gsm.pswd, temp, 64);

    get_key_u16(m_config_nvs, "gsmport", &config->gsm.port);

    get_key_u8(m_config_nvs, "gsmserver", temp, 64);
    memcpy(config->gsm.server, temp, 64);


    get_key_u8(m_config_nvs, "p2psf", &config->p2p.sf, 1);
    get_key_u8(m_config_nvs, "p2pbw", &config->p2p.bw, 1);
    get_key_u8(m_config_nvs, "p2pcr", &config->p2p.cr, 1);
    get_key_u32(m_config_nvs, "p2ptxfreq", &config->p2p.txFreq);
    get_key_u32(m_config_nvs, "p2ptxtime", &config->p2p.txTimeout);
    get_key_u8(m_config_nvs, "p2ptxpwr", &config->p2p.txPower, 1);
    get_key_u32(m_config_nvs, "p2prxfreq", &config->p2p.rxFreq);
    get_key_u32(m_config_nvs, "p2prxtime", &config->p2p.rxTimeout);
    get_key_u32(m_config_nvs, "p2prxdelay", &config->p2p.rxDelay);

    get_key_u8(m_config_nvs, "blepwr", &config->ble.power, 1);
    
    nvs_close(m_config_nvs);

    return ESP_OK;
}

esp_err_t storage_timers_set(IscaTimers_t *timers)
{
    //m_timers_nvs
    esp_err_t err = nvs_open("timers", NVS_READWRITE, &m_timers_nvs);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed open 'timers' namespace");
        return err;
    }
    
    set_key_u16(m_timers_nvs, "bleadvtime", &timers->bleAdvTime);

    uint32_t temp;

    temp = timers->p2pMovNorm;
    set_key_u32_20bits(m_timers_nvs, "p2pmovnorm", &temp);
    temp = timers->p2pMovEmer;
    set_key_u32_20bits(m_timers_nvs, "p2pmovemer", &temp);
    temp = timers->p2pStpNorm;
    set_key_u32_20bits(m_timers_nvs, "p2pstpnorm", &temp);
    temp = timers->p2pStpEmer;
    set_key_u32_20bits(m_timers_nvs, "p2pstpemer", &temp);
    temp = timers->lrwMovNorm;
    set_key_u32_20bits(m_timers_nvs, "lrwmovnorm", &temp);
    temp = timers->lrwMovEmer;
    set_key_u32_20bits(m_timers_nvs, "lrwmovemer", &temp);
    temp = timers->lrwStpNorm;
    set_key_u32_20bits(m_timers_nvs, "lrwstpnorm", &temp);
    temp = timers->lrwStpEmer;
    set_key_u32_20bits(m_timers_nvs, "lrwstpemer", &temp);
    temp = timers->gsmMovNorm;
    set_key_u32_20bits(m_timers_nvs, "gsmmovnorm", &temp);
    temp = timers->gsmMovEmer;
    set_key_u32_20bits(m_timers_nvs, "gsmmovemer", &temp);
    temp = timers->gsmStpNorm;
    set_key_u32_20bits(m_timers_nvs, "gsmstpnorm", &temp);
    temp = timers->gsmStpEmer;
    set_key_u32_20bits(m_timers_nvs, "gsmstpemer", &temp);

    nvs_close(m_timers_nvs);

    return ESP_OK;
}

esp_err_t storage_timers_get(IscaTimers_t *timers)
{

    esp_err_t err = nvs_open("timers", NVS_READWRITE, &m_timers_nvs);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed open 'timers' namespace");
        return err;
    }
    get_key_u16(m_timers_nvs, "bleadvtime", &timers->bleAdvTime);

    uint32_t temp;

    get_key_u32_20bits(m_timers_nvs, "p2pmovnorm", &temp);
    timers->p2pMovNorm = temp;
    get_key_u32_20bits(m_timers_nvs, "p2pmovemer", &temp);
    timers->p2pMovEmer = temp;
    get_key_u32_20bits(m_timers_nvs, "p2pstpnorm", &temp);
    timers->p2pStpNorm = temp;
    get_key_u32_20bits(m_timers_nvs, "p2pstpemer", &temp);
    timers->p2pStpEmer = temp;
    get_key_u32_20bits(m_timers_nvs, "lrwmovnorm", &temp);
    timers->lrwMovNorm = temp;
    get_key_u32_20bits(m_timers_nvs, "lrwmovemer", &temp);
    timers->lrwMovEmer = temp;
    get_key_u32_20bits(m_timers_nvs, "lrwstpnorm", &temp);
    timers->lrwStpNorm = temp;
    get_key_u32_20bits(m_timers_nvs, "lrwstpemer", &temp);
    timers->lrwStpEmer = temp;
    get_key_u32_20bits(m_timers_nvs, "gsmmovnorm", &temp);
    timers->gsmMovNorm = temp;
    get_key_u32_20bits(m_timers_nvs, "gsmmovemer", &temp);
    timers->gsmMovEmer = temp;
    get_key_u32_20bits(m_timers_nvs, "gsmstpnorm", &temp);
    timers->gsmStpNorm = temp;
    get_key_u32_20bits(m_timers_nvs, "gsmstpemer", &temp);
    timers->gsmStpEmer = temp;
    
    nvs_close(m_timers_nvs);

    return ESP_OK;
}

esp_err_t storage_gsm_save_position(char *str, uint32_t *id)
{
    if (m_id >= GSM_MAX_POSITIONS)
        return ESP_ERR_NO_MEM;

    uint32_t temp = 0;

    get_id(&temp);

    temp++;

    set_id(&temp);

    char key[9];
    sprintf(key, "%08lx", temp);

    char blob[GSM_PAYLOAD_MAX_SIZE]={'0'};
    memset(blob, 1, GSM_PAYLOAD_MAX_SIZE);

    esp_err_t err = nvs_open("gsm_position", NVS_READWRITE, &m_gsm_position_nvs);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed open 'gsm_position' namespace");
        return err;
    }
    printf("\t\t [SET STR] %d bytes: %s\r\n", strlen(str), str);
    
    for(int i = 0; i< strlen(str); i++)
        printf("%02x", (unsigned int)*(str+i));
    printf("\r\n");
    err = nvs_get_str(m_gsm_position_nvs, key, blob, NULL);
    if (err == ESP_ERR_NVS_NOT_FOUND) // ESP_ERR_NVS_NOT_FOUND error occurs if key doesn't exist yet
        err = nvs_set_str(m_gsm_position_nvs, key, str);

    err = nvs_commit(m_gsm_position_nvs);

    *id = temp;
    nvs_close(m_gsm_position_nvs);
    return err;
}

esp_err_t storage_gsm_get_last_position(char *str, size_t *len)
{
    char key[9];
    uint32_t id;
    esp_err_t err = ESP_FAIL;
    get_id(&id);

    if (id > 0)
    {
        sprintf(key, "%08lx", id);

        id--;
        set_id(&id);
        err = nvs_open("gsm_position", NVS_READWRITE, &m_gsm_position_nvs);
        if (err != ESP_OK) 
        {
            ESP_LOGE(TAG, "Failed open 'gsm_position' namespace");
            return err;
        }
        err = nvs_get_str(m_gsm_position_nvs, key, NULL, len);
        ESP_LOGW(TAG, "[GET STR] [%d length]", *len);
        //*len = strlen(str);
        err = nvs_get_str(m_gsm_position_nvs, key, str, len);
        ESP_LOGW(TAG, "[GET STR] [%d bytes] %s", strlen(str), str);
        if(err != ESP_OK)
            return err;
        err = nvs_erase_key(m_gsm_position_nvs, key);
        nvs_close(m_gsm_position_nvs);
    }
    else
        err = ESP_FAIL;
    
    
    return err;
}

esp_err_t gsm_save_id()
{
    uint32_t temp = 0;
    
    esp_err_t err = nvs_open("gsm_position", NVS_READWRITE, &m_gsm_position_nvs);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed open 'gsm_position' namespace");
        return err;
    }
    nvs_erase_key(m_gsm_position_nvs, "id");

    nvs_get_u32(m_gsm_position_nvs, "id", &temp);
    
    nvs_set_u32(m_gsm_position_nvs, "id", m_id);
    err = nvs_commit(m_gsm_position_nvs);
    
    nvs_close(m_gsm_position_nvs);
    
    return err;
}

esp_err_t storage_gsm_get_id(uint32_t *id)
{
    return get_id(id);
}

esp_err_t storage_gsm_erase_nvs()
{
    return nvs_erase_all(m_gsm_position_nvs);
}

static esp_err_t set_id(uint32_t *id)
{
    m_id = *id;
    return gsm_save_id();
}

static esp_err_t get_id(uint32_t *id)
{
    uint32_t temp = 0;

    esp_err_t err = nvs_open("gsm_position", NVS_READWRITE, &m_gsm_position_nvs);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed open 'gsm_position' namespace");
        return err;
    }
    err = nvs_get_u32(m_gsm_position_nvs, "id", &temp);
    *id = temp;
    nvs_close(m_gsm_position_nvs);
    return err;
}

static esp_err_t set_key_u8(nvs_handle_t ns, const char *key, uint8_t *data, size_t size)
{
    esp_err_t err = nvs_erase_key(ns, key);

    if (size > 1)
    {
        uint8_t *blob = (uint8_t *)malloc(size*sizeof(uint8_t));
        memset(blob, 1, size*sizeof(uint8_t));

        if (blob == NULL)
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

static esp_err_t set_key_u16(nvs_handle_t ns, const char *key, uint16_t *data)
{
    uint8_t temp[2];
    temp[0] = *data >> 8;
    temp[1] = *data & 0xFF;

    return set_key_u8(ns, key, temp, 2);
}

static esp_err_t set_key_u32(nvs_handle_t ns, const char *key, uint32_t *data)
{
    uint8_t temp[4];
    temp[0] = *data >> 24;
    temp[1] = (*data >> 16) & 0xFF;
    temp[2] = (*data >> 8) & 0xFF;
    temp[3] = *data & 0xFF;

    return set_key_u8(ns, key, temp, 4);
}

static esp_err_t set_key_u64(nvs_handle_t ns, const char *key, uint64_t *data)
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

static esp_err_t set_key_u32_20bits(nvs_handle_t ns, const char *key, uint32_t *data)
{
    uint8_t temp[3];
    temp[0] = (*data >> 16) & 0x0F;
    temp[1] = (*data >> 8) & 0xFF;
    temp[2] = *data & 0xFF;

    return set_key_u8(ns, key, temp, 3);
}

static esp_err_t get_key_u8(nvs_handle_t ns, const char *key, uint8_t *data, size_t size)
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

static esp_err_t get_key_u16(nvs_handle_t ns, const char *key, uint16_t *data)
{
    uint8_t temp[2];

    esp_err_t err = get_key_u8(ns, key, temp, 2);
    
    *data = (temp[0] << 8) + temp[1];

    return err;
}

static esp_err_t get_key_u32(nvs_handle_t ns, const char *key, uint32_t *data)
{
    uint8_t temp[4];

    esp_err_t err = get_key_u8(ns, key, temp, 4);

    *data = (temp[0] << 24) + (temp[1] << 16) + (temp[2] << 8) + temp[3];

    return err;
}

static esp_err_t get_key_u64(nvs_handle_t ns, const char *key, uint64_t *data)
{
    uint8_t temp[8];

    esp_err_t err = get_key_u8(ns, key, temp, 8);

    *data = ((uint64_t)temp[0] << 56) + ((uint64_t)temp[1] << 48) + ((uint64_t)temp[2] << 40) + ((uint64_t)temp[3] << 32) +
        ((uint64_t)temp[4] << 24) + ((uint64_t)temp[5] << 16) + ((uint64_t)temp[6] << 8) + temp[7];

    return err;
}

static esp_err_t get_key_u32_20bits(nvs_handle_t ns, const char *key, uint32_t *data)
{
    uint8_t temp[3];

    esp_err_t err = get_key_u8(ns, key, temp, 3);

    *data = (temp[0] << 16) + (temp[1] << 8) + (temp[2]);

    return err;
}