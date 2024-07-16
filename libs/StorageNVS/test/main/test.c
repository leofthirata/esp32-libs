#include "main.h"
#include "sdkconfig.h"
#include "esp_mac.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_log.h"

#include "Storage/storage.h"
#include <cJSON.h>
#include "esp_timer.h"

uint8_t mac[6];
static const char* TAG = "test.c";

Isca_t config;

char *generate_position_json(const char *hw);

void app_main(void)
{
    memset(&config, 0, sizeof(config));

    uint8_t bleMac[6] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };
    config.loraId = 13000123;
    uint8_t devEui[8] = { 0xde, 0xa1, 0x4e, 0x2a, 0x60, 0x84, 0x4c, 0x94 };
    uint8_t appEui[8] = { 0x97, 0x7d, 0x60, 0xee, 0xaf, 0xc9, 0xee, 0x72 };
    config.nodeDevAddr = 0x00804c28;
    uint8_t nwsKey[16] = { 0xbd, 0x75, 0x58, 0x87, 0x5d, 0xa2, 0xc8, 0x5c, 0xfb, 0x41, 0x14, 0xf3, 0xfa, 0x2a, 0xa1, 0x9d };
    uint8_t appsKey[16] = { 0xcd, 0x15, 0x64, 0xd6, 0x74, 0x27, 0x82, 0x84, 0x89, 0xd8, 0xb1, 0xe7, 0x7a, 0x6b, 0x59, 0x53 };

    memcpy(config.bleMac, bleMac, 6);
    memcpy(config.nodeDeviceEUI, devEui, 8);
    memcpy(config.nodeAppEUI, appEui, 8);
    memcpy(config.nodeNwsKey, nwsKey, 16);
    memcpy(config.nodeAppsKey, appsKey, 16);

    printf("[BLE] MAC: ");
	for(int k = 5; k >= 0; k--)
	{
		printf("%02X", config.bleMac[k]);
		if(k > 0)
			printf(":");
		else
			printf("\r\n");
	}
    printf("loraID: %ld | devAddress: %08lX\r\n", config.loraId,config.nodeDevAddr);
    ESP_LOG_BUFFER_HEX("deviceEUI", config.nodeDeviceEUI, sizeof(config.nodeDeviceEUI));
    ESP_LOG_BUFFER_HEX("appEUI", config.nodeAppEUI, sizeof(config.nodeAppEUI));
    ESP_LOG_BUFFER_HEX("nwSKey", config.nodeNwsKey, sizeof(config.nodeNwsKey));
    ESP_LOG_BUFFER_HEX("appSKey", config.nodeAppsKey, sizeof(config.nodeAppsKey));

    ESP_LOGW(TAG, "MAC: " MACSTR " ", MAC2STR(mac));

    ESP_ERROR_CHECK(storage_init());

    Isca_t temp;
    config.fwVerProtocol = 123;
    config.hwVer = 222;
    config.lrwProtocol = 101;

    storage_otp_set(&config);
    storage_otp_get(&temp);

	printf("[BLE] MAC: ");
	for(int k = 5; k >= 0; k--)
	{
		printf("%02X", temp.bleMac[k]);
		if(k > 0)
			printf(":");
		else
			printf("\r\n");
	}
    printf("loraID: %ld | devAddress: %08lX\r\n", temp.loraId,temp.nodeDevAddr);
    ESP_LOG_BUFFER_HEX("deviceEUI", temp.nodeDeviceEUI, sizeof(temp.nodeDeviceEUI));
    ESP_LOG_BUFFER_HEX("appEUI", temp.nodeAppEUI, sizeof(temp.nodeAppEUI));
    ESP_LOG_BUFFER_HEX("nwSKey", temp.nodeNwsKey, sizeof(temp.nodeNwsKey));
    ESP_LOG_BUFFER_HEX("appSKey", temp.nodeAppsKey, sizeof(temp.nodeAppsKey));

    config.flags.asBit.bleStatus = 0;
    config.flags.asBit.lastResetReason = 0b10111;
    config.flags.asBit.reserved = 1;
    config.flags.asBit.emergency = 0;
    config.flags.asBit.lowBattery = 0;
    config.flags.asBit.jammer = 0;
    config.flags.asBit.movement = 1;
    config.flags.asBit.stockMode = 1;
    config.flags.asBit.output = 1;
    config.flags.asBit.input = 1;
    config.batteryMiliVolts = 200;
    config.temperatureCelsius = 15;
    config.reset = 123;
    config.batteryStatus = 3;
    config.lastP2PTick = esp_timer_get_time();
    config.lastLRWTick = esp_timer_get_time();
    config.P2PCount = 55;
    config.acc[0] = 111;
    config.acc[1] = 222;
    config.acc[2] = 333;
    config.temp = -5.2;

	printf("Temp: 0x%02X = %d | ", config.temperatureCelsius, config.temperatureCelsius);
	printf("Battery: 0x%04X = %d mV | ", config.batteryMiliVolts, config.batteryMiliVolts);
	printf("Reset: %ld\r\n", config.reset);
	printf("batteryStatus: %d\r\n", config.batteryStatus);
	printf("lastP2PTick: %lld\r\n", config.lastP2PTick);
	printf("lastLRWTick: %lld\r\n", config.lastLRWTick);
	printf("P2PCount: %d\r\n", config.P2PCount);
	printf("Flags: %02X %02X\r\n", config.flags.asArray[0], config.flags.asArray[1]);
    ESP_LOGI(TAG, "[ACC](mg) x:%ld | y:%ld | z:%ld | [TEMP](oC) %02.2f", 
                config.acc[0], config.acc[1], config.acc[2], config.temp);
        
    storage_status_set(&config);
    storage_status_get(&temp);

	printf("Temp: 0x%02X = %d | ", temp.temperatureCelsius, temp.temperatureCelsius);
	printf("Battery: 0x%04X = %d mV | ", temp.batteryMiliVolts, temp.batteryMiliVolts);
	printf("Reset: %ld\r\n", temp.reset);
	printf("batteryStatus: %d\r\n", temp.batteryStatus);
	printf("lastP2PTick: %lld\r\n", temp.lastP2PTick);
	printf("lastLRWTick: %lld\r\n", temp.lastLRWTick);
	printf("P2PCount: %d\r\n", temp.P2PCount);
	printf("Flags: %02X %02X\r\n", temp.flags.asArray[0], temp.flags.asArray[1]);
    ESP_LOGI(TAG, "[ACC](mg) x:%ld | y:%ld | z:%ld | [TEMP](oC) %02.2f", 
                temp.acc[0], temp.acc[1], temp.acc[2], temp.temp);

    config.blePower = 32;
    config.bleAdvTime = 1000;
    config.p2pMovNorm = 3000;
    config.p2pMovEmer = 3001;
    config.p2pStpNorm = 3002;
    config.p2pStpEmer = 3003;
    config.lrwMovNorm = 3004;
    config.lrwMovEmer = 3005;
    config.lrwStpNorm = 3006;
    config.lrwStpEmer = 3007;
    config.gsmMovNorm = 3008;
    config.gsmMovEmer = 3009;
    config.gsmStpNorm = 3010;
    config.gsmStpEmer = 3011;

	printf("BLE Power: 0x%02X = %d | ", config.blePower, config.blePower);
	printf("BLE AdvTime = %d \r\n", config.bleAdvTime);
	printf("\tP2P_MN: %d | P2P_ME: %d | P2P_SN: %d | P2P_SE: %d\r\n", config.p2pMovNorm, config.p2pMovEmer, config.p2pStpNorm, config.p2pStpEmer);
	printf("\tLRW_MN: %d | LRW_ME: %d | LRW_SN: %d | LRW_SE: %d\r\n", config.lrwMovNorm, config.lrwMovEmer, config.lrwStpNorm, config.lrwStpEmer);
	printf("\tGSM_MN: %d | GSM_ME: %d | GSM_SN: %d | GSM_SE: %d\r\n", config.gsmMovNorm, config.gsmMovEmer, config.gsmStpNorm, config.gsmStpEmer);

    storage_ble_lora_gsm_set(&config);
    storage_ble_lora_gsm_get(&temp);

    printf("BLE Power: 0x%02X = %d | ", temp.blePower, temp.blePower);
	printf("BLE AdvTime = %d \r\n", temp.bleAdvTime);
	printf("\tP2P_MN: %d | P2P_ME: %d | P2P_SN: %d | P2P_SE: %d\r\n", temp.p2pMovNorm, temp.p2pMovEmer, temp.p2pStpNorm, temp.p2pStpEmer);
	printf("\tLRW_MN: %d | LRW_ME: %d | LRW_SN: %d | LRW_SE: %d\r\n", temp.lrwMovNorm, temp.lrwMovEmer, temp.lrwStpNorm, temp.lrwStpEmer);
	printf("\tGSM_MN: %d | GSM_ME: %d | GSM_SN: %d | GSM_SE: %d\r\n", temp.gsmMovNorm, temp.gsmMovEmer, temp.gsmStpNorm, temp.gsmStpEmer);

    config.fwVer = "1.0.0-RC2";
    config.fwVerProtocol = 102;
    config.lrwProtocol = 103;

    const char *tmp1 = "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA";
    const char *tmp2 = "BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB";
    const char *tmp3 = "CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC";
    const char *tmp4 = "DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD";
    memset(config.apn, 0, 64);
    strncpy(config.apn, tmp1, 64);
    memset(config.gsmUser, 0, 64);
    strncpy(config.gsmUser, tmp2, 64);
    memset(config.gsmPswd, 0, 64);
    strncpy(config.gsmPswd, tmp3, 64);
    memset(config.gsmServer, 0, 64);
    strncpy(config.gsmServer, tmp4, 64);

    config.p2pTXFreq = 5555;
    config.p2pRXFreq = 6666;
    config.p2pBW = 50;
    config.p2pSF = 49;
    config.p2pCR = 48;
    config.lrwSF = 47;
    config.lrwADR = 1;

    ESP_LOGE(TAG, "fwVer (%s)\n", config.fwVer);
    printf("fwVerProtocol: %d | ", config.fwVerProtocol);
    printf("lrwProtocol: %d \r\n", config.lrwProtocol);
    ESP_LOGE(TAG, "apn (%s)\n", config.apn);
    ESP_LOGE(TAG, "gsmUser (%s)\n", config.gsmUser);
    ESP_LOGE(TAG, "gsmPswd (%s)\n", config.gsmPswd);
    ESP_LOGE(TAG, "gsmServer (%s)\n", config.gsmServer);
    printf("p2pTXFreq: %ld \r\n", config.p2pTXFreq);
    printf("p2pRXFreq: %ld \r\n", config.p2pRXFreq);
    printf("p2pBW: %d \r\n", config.p2pBW);
    printf("p2pSF: %d \r\n", config.p2pSF);
    printf("p2pCR: %d \r\n", config.p2pCR);
    printf("lrwSF: %d \r\n", config.lrwSF);

    storage_config_set(&config);
    storage_config_get(&temp);

    ESP_LOGE(TAG, "fwVer (%s)\n", temp.fwVer);
    printf("fwVerProtocol: %d | ", temp.fwVerProtocol);
    printf("lrwProtocol: %d \r\n", temp.lrwProtocol);
    ESP_LOGE(TAG, "apn (%s)\n", temp.apn);
    ESP_LOGE(TAG, "gsmUser (%s)\n", temp.gsmUser);
    ESP_LOGE(TAG, "gsmPswd (%s)\n", temp.gsmPswd);
    ESP_LOGE(TAG, "gsmServer (%s)\n", temp.gsmServer);
    printf("p2pTXFreq: %ld \r\n", temp.p2pTXFreq);
    printf("p2pRXFreq: %ld \r\n", temp.p2pRXFreq);
    printf("p2pBW: %d \r\n", temp.p2pBW);
    printf("p2pSF: %d \r\n", temp.p2pSF);
    printf("p2pCR: %d \r\n", temp.p2pCR);
    printf("lrwSF: %d \r\n", temp.lrwSF);

    config.lrwProtocol = 5;
    config.p2pTXFreq = 7777;
    config.gsmPort = 7;
    const char *tmp5 = "ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ";
    memset(config.apn, 0, 64);
    strncpy(config.apn, tmp5, 64);

    config.flags.asBit.bleStatus = 1;
    config.flags.asBit.lastResetReason = 0b00100;
    config.flags.asBit.reserved = 1;
    config.flags.asBit.emergency = 0;
    config.flags.asBit.lowBattery = 0;
    config.flags.asBit.jammer = 0;
    config.flags.asBit.movement = 1;
    config.flags.asBit.stockMode = 1;
    config.flags.asBit.output = 1;
    config.flags.asBit.input = 1;
    config.batteryMiliVolts = 300;
    config.temperatureCelsius = 20;
    config.reset = 100;
    config.batteryStatus = 2;
    config.lastP2PTick = esp_timer_get_time();
    config.lastLRWTick = esp_timer_get_time();
    config.P2PCount = 66;
    config.acc[0] = 100;
    config.acc[1] = 99;
    config.acc[2] = 98;
    config.temp = -21.3;

    storage_config_set(&config);
    storage_status_set(&config);
    storage_config_get(&temp);
    storage_status_get(&temp);

    printf("lrwProtocol: %d \r\n", temp.lrwProtocol);
    ESP_LOGE(TAG, "apn (%s)\n", temp.apn);
    printf("p2pTXFreq: %ld \r\n", temp.p2pTXFreq);
    printf("gsmPort: %d \r\n", temp.gsmPort);

    printf("Temp: 0x%02X = %d | ", temp.temperatureCelsius, temp.temperatureCelsius);
	printf("Battery: 0x%04X = %d mV | ", temp.batteryMiliVolts, temp.batteryMiliVolts);
	printf("Reset: %ld\r\n", temp.reset);
	printf("batteryStatus: %d\r\n", temp.batteryStatus);
	printf("lastP2PTick: %lld\r\n", temp.lastP2PTick);
	printf("lastLRWTick: %lld\r\n", temp.lastLRWTick);
	printf("P2PCount: %d\r\n", temp.P2PCount);
	printf("Flags: %02X %02X\r\n", temp.flags.asArray[0], temp.flags.asArray[1]);
    ESP_LOGI(TAG, "[ACC](mg) x:%ld | y:%ld | z:%ld | [TEMP](oC) %02.2f", 
                temp.acc[0], temp.acc[1], temp.acc[2], temp.temp);

    char *gsmJson = generate_position_json("0.1");
    ESP_LOGI(TAG, "gsmJson (%s)\n", gsmJson);

    uint32_t id = 0;
    storage_gsm_save_position(gsmJson, &id);

    char gsmJsonRead[150];
    size_t len = 200;
    storage_gsm_get_last_position(gsmJsonRead, &len);
    ESP_LOGI(TAG, "gsmJsonRead (%s)\n", gsmJsonRead);
    printf("len: %d\r\n", len);

    gsmJson = generate_position_json("0.2");
    storage_gsm_save_position(gsmJson, &id);

    gsmJson = generate_position_json("0.3");
    storage_gsm_save_position(gsmJson, &id);

    gsmJson = generate_position_json("0.4");
    storage_gsm_save_position(gsmJson, &id);

    char gsmJsonRead2[200];
    storage_gsm_get_last_position(gsmJsonRead2, &len);
    ESP_LOGI(TAG, "gsmJsonRead2 (%s)\n", gsmJsonRead2);
    printf("len: %d\r\n", len);

    // char gsmJsonRead3[200];
    // err = storage_gsm_get_last_position(gsmJsonRead3, &len);
    // ESP_LOGI(TAG, "gsmJsonRead3 (%s)\n", gsmJsonRead3);
    // printf("len: %d\r\n", len);

    // char gsmJsonRead4[200];
    // err = storage_gsm_get_last_position(gsmJsonRead4, &len);
    // ESP_LOGI(TAG, "gsmJsonRead4 (%s)\n", gsmJsonRead4);
    // printf("len: %d\r\n", len);

    // char gsmJsonRead5[200];
    // err = storage_gsm_get_last_position(gsmJsonRead5, &len);
    // if (err != ESP_OK)
    //     ESP_LOGE(TAG, "Error (%s)\n", esp_err_to_name(err));

    // ESP_LOGI(TAG, "gsmJsonRead5 (%s)\n", gsmJsonRead5);
    // printf("len: %d\r\n", len);

    storage_deinit();

    ESP_ERROR_CHECK(storage_init());

    char gsmJsonRead3[200];
    storage_gsm_get_last_position(gsmJsonRead3, &len);
    ESP_LOGI(TAG, "gsmJsonRead3 (%s)\n", gsmJsonRead3);
    printf("len: %d\r\n", len);
}

char *generate_position_json(const char *hw)
{
    char *string = NULL;
    // esp_err_t ret;
    cJSON *root, *data;

    root = cJSON_CreateObject();

    cJSON_AddStringToObject(
        root,
        "type",
        "location");
    cJSON_AddStringToObject(
        root,
        "imei",
        "356303486904368");
    cJSON_AddStringToObject(
        root,
        "fw",
        "0.0.1");
    cJSON_AddStringToObject(
        root,
        "hw",
        hw);

    cJSON_AddStringToObject(
        root,
        "time",
        "28/06/2024");

    data = cJSON_AddObjectToObject(
        root,
        "flags");

    cJSON_AddBoolToObject(data, "emergency", false);
    cJSON_AddBoolToObject(data, "jammGSM", false);
    cJSON_AddBoolToObject(data, "jammLora", false);
    cJSON_AddBoolToObject(data, "lowBat", false);

    data = cJSON_AddArrayToObject(root, "erbs");

    string = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return string;
}