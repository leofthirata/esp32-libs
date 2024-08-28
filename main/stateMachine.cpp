#include "stateMachine.hpp"
#include "esp_timer.h"
#include "lora.hpp"
#include "sensors.hpp"
#include "gsm.h"
#include "sys/time.h"
#include "mbedtls/base64.h"
#include "storage_nvs.h"
#include "utils.h"

static Isca_t *m_isca;
static bool resetRequested = false;
static const char *TAG = "stateTask";
static Isca_SM_t state;
static TaskHandle_t xTaskToNotify = NULL;
static QueueHandle_t xQueueP2PRx, xQueueLRWRx, xQueueGSMRx;

static void p2pTimerCallback(void* arg);
static void lrwTimerCallback(void* arg);
static void gsmTimerCallback(void* arg);
esp_timer_handle_t p2pTimer, lrwTimer, gsmTimer;

static LoRaLRWTxReq_t lrwTx;
static LoRaLRWRx_t lrwRx;
static LoRaP2PRx_t p2pRx;
static LoRaP2PReq_t p2pTx, p2pTxRes;
static LoRaLRWTxRes_t lrwTxRes;
static SensorsStatus_t _sensorStatus; 
static GSMTxReq_t gsmTx;
static GSMTxRes_t gsmTxRes;
static GSMRx_t gsmRx;
// APP event base declaration
ESP_EVENT_DECLARE_BASE(APP_EVENT);

#define BIT_LRW_TX_REQ  0x01
#define BIT_LRW_RX  0x02
#define BIT_P2P_TX_REQ  0x04
#define BIT_P2P_RX  0x08
#define BIT_P2P_TX_RES 0x10
#define BIT_LRW_TX_RES 0x20
#define BIT_SENSORS_STATUS 0x40
#define BIT_LRW_REQ_TX_STATUS 0x80
#define BIT_GSM_TX_REQ  0x100
#define BIT_GSM_TX_RES 0x200
#define BIT_GSM_RX 0x400

void stopTimers();
void lorawanRX(LoRaLRWRx_t* _lrwRx);

void setMachineState(Isca_SM_t new_state)
{
	state = new_state;
}

void turnOnBLE()
{
	printf("[SM] Warning! Turn On BLE not implemented\r\n");
}

void turnOffBLE()
{
	printf("[SM] Warning! Turn Off BLE not implemented\r\n");
}

void turnOnOutput()
{
	printf("[SM] Warning! Turn Off Output not implemented\r\n");
}

void turnOffOutput()
{
	printf("[SM] Warning! Turn Off Output not implemented\r\n");
}

void changeBLEPower()
{
	printf("[SM] Warning! change BLE Power not implemented\r\n");
}

void changeBLETime()
{
	printf("[SM] Warning! Change BLE Time not implemented\r\n");
}

void enableLed()
{
	printf("[SM] Warning! Enable LED not implemented\r\n");
}

void disableLed()
{
	printf("[SM] Warning! Disable LED not implemented\r\n");
}

void enterEmergency()
{
    // setMachineState(SM_ENTER_EMERGENCY);
    // xTaskNotifyGive(xTaskToNotify);
    printf("[SM] WARNING! Emergency ON!\r\n");
}

void exitEmergency()
{
    // setMachineState(SM_EXIT_EMERGENCY);	
    // xTaskNotifyGive(xTaskToNotify);
    printf("[SM] WARNING! Emergency OFF!\r\n");
}

void enterStockMode()
{
	if(!m_isca->status.flags.asBit.emergency)
	{
		m_isca->status.flags.asBit.stockMode = true;
		printf("[SM] Warning! Enter Stock Mode\r\n");
		stopTimers();
		// /bleDisableConnectableDevice();
		printf("[DEQUEUE] Start Queue flush to enter stock mode\r\n");
		// while(numberQueueElements())
		// {
		// 	SendQueue_t sendCommand = QUEUE_NONE;
		// 	sendCommand = dequeueSend();
		// }
		printf("[DEQUEUE] Finish Queue flush to enter stock mode\r\n");
	}
	else
	{
		m_isca->status.flags.asBit.stockMode = false;
		printf("[SM] Warning! Not Entering Stock Mode, We are in Emergency\r\n");
	}

}

void resetRequest()
{
	resetRequested = true;
	printf("[SM] Warning! Reset Requested\r\n");
}

void sendStatus()
{
	printf("[SM] Warning! Status Requested\r\n");
    esp_event_post(APP_EVENT, APP_EVENT_REQ_LRW_SEND_STATUS, (void*)NULL, 0, 0);
	//queueSend(QUEUE_SEND_STATUS);
}

void stopTimers()
{
    if(esp_timer_is_active(p2pTimer))
	{
		uint64_t expiryTime = 0;
		esp_timer_get_expiry_time(p2pTimer, &expiryTime);

		printf("[TIMER] Stopping Active P2P Timer %llus |\r\n", expiryTime/1000000);
		esp_timer_stop(p2pTimer);
        m_isca->status.P2PAlarm = 0;
    }

    if(esp_timer_is_active(lrwTimer))
	{
		uint64_t expiryTime = 0;
		esp_timer_get_expiry_time(lrwTimer, &expiryTime);

		printf("[TIMER] Stopping Active LRW Timer %llus |\r\n", expiryTime/1000000);
		esp_timer_stop(lrwTimer);
        m_isca->status.LRWAlarm = 0;
    }

    if(esp_timer_is_active(gsmTimer))
	{
		uint64_t expiryTime = 0;
		esp_timer_get_expiry_time(gsmTimer, &expiryTime);

		printf("[TIMER] Stopping Active GSM Timer %llus |\r\n", expiryTime/1000000);
		esp_timer_stop(gsmTimer);
        m_isca->status.GSMAlarm = 0;
    }
}

void updateGSMTimer(uint32_t newTime)
{
	int64_t now;
    if(esp_timer_is_active(gsmTimer))
	{
		uint64_t expiryTime = 0;
		esp_timer_get_expiry_time(gsmTimer, &expiryTime);

		printf("[TIMER] Stopping Active GSM Timer %llus |\r\n", expiryTime/1000000);
		esp_timer_stop(gsmTimer);
		printf("[TIMER] Restarting GSM Timer new timeout %lus |\r\n", newTime);
        now = esp_timer_get_time();
		uint8_t ret_val = esp_timer_start_periodic(gsmTimer, (uint64_t)newTime*1000000);
		if(ret_val != ESP_OK)
		{
			printf("[TIMER] Error restarting GSM Timer new timeout\r\n");
			while(1);
		}
	}
    else
    {
        printf("[TIMER] Starting GSM Timer new timeout %lus |\r\n", newTime);
        now = esp_timer_get_time();
        uint8_t ret_val = esp_timer_start_periodic(gsmTimer, (uint64_t)newTime*1000000);
        if(ret_val != ESP_OK)
		{
			printf("[TIMER] Error starting GSM Timer new timeout\r\n");
			while(1);
		}
    }
    m_isca->status.GSMAlarm = now + (newTime*1000000);
}

void updateP2PTimer(uint32_t newTime)
{
	int64_t now;
    if(esp_timer_is_active(p2pTimer))
	{
		uint64_t expiryTime = 0;
		esp_timer_get_expiry_time(p2pTimer, &expiryTime);

		printf("[TIMER] Stopping Active P2P Timer %llus |\r\n", expiryTime/1000000);
		esp_timer_stop(p2pTimer);
		printf("[TIMER] Restarting P2P Timer new timeout %lus |\r\n", newTime);
        now = esp_timer_get_time();
		uint8_t ret_val = esp_timer_start_periodic(p2pTimer, (uint64_t)newTime*1000000);
		if(ret_val != ESP_OK)
		{
			printf("[TIMER] Error restarting P2P Timer new timeout\r\n");
			while(1);
		}
	}
    else
    {
        printf("[TIMER] Starting P2P Timer new timeout %lus |\r\n", newTime);
        now = esp_timer_get_time();
        uint8_t ret_val = esp_timer_start_periodic(p2pTimer, (uint64_t)newTime*1000000);
        if(ret_val != ESP_OK)
		{
			printf("[TIMER] Error starting P2P Timer new timeout\r\n");
			while(1);
		}
    }
    m_isca->status.P2PAlarm = now + (newTime*1000000);
}

void updateLRWTimer(uint32_t newTime)
{
    int64_t now;
    if(esp_timer_is_active(lrwTimer))
	{
		uint64_t expiryTime = 0;
		esp_timer_get_expiry_time(lrwTimer, &expiryTime);

		printf("[TIMER] Stopping Active LRW Timer %llus |\r\n", expiryTime/1000000);
		esp_timer_stop(lrwTimer);
		printf("[TIMER] Restarting LRW Timer new timeout %lus |\r\n", newTime);
        now = esp_timer_get_time();
		uint8_t ret_val = esp_timer_start_periodic(lrwTimer, (uint64_t)newTime*1000000);
		if(ret_val != ESP_OK)
		{
			printf("[TIMER] Error restarting LRW Timer new timeout\r\n");
			while(1);
		}
	}
    else
    {
        printf("[TIMER] Starting LRW Timer new timeout %lus |\r\n", newTime);
        now = esp_timer_get_time();
        uint8_t ret_val = esp_timer_start_periodic(lrwTimer, (uint64_t)newTime*1000000);
        if(ret_val != ESP_OK)
		{
			printf("[TIMER] Error starting LRW Timer new timeout\r\n");
			while(1);
		}
    }
    m_isca->status.LRWAlarm = now + (newTime*1000000);
}

void changeLoRaTimes(uint32_t *timeArray)
{
	m_isca->timers.p2pMovNorm = *timeArray;
	m_isca->timers.p2pMovEmer = *(timeArray + 1);
	m_isca->timers.p2pStpNorm = *(timeArray + 2);
	m_isca->timers.p2pStpEmer = *(timeArray + 3);
	m_isca->timers.lrwMovNorm = *(timeArray + 4);
	m_isca->timers.lrwMovEmer = *(timeArray + 5);
	m_isca->timers.lrwStpNorm = *(timeArray + 6);
	m_isca->timers.lrwStpEmer = *(timeArray + 7);
	printf("[SM Warning! Changed LoRa Times\r\n");
	printf("\tP2P_MN: %d | P2P_ME: %d | P2P_SN: %d | P2P_SE: %d\r\n", 
        m_isca->timers.p2pMovNorm, m_isca->timers.p2pMovEmer, 
        m_isca->timers.p2pStpNorm, m_isca->timers.p2pStpEmer);
	printf("\tLRW_MN: %d | LRW_ME: %d | LRW_SN: %d | LRW_SE: %d\r\n", 
        m_isca->timers.lrwMovNorm, m_isca->timers.lrwMovEmer, 
        m_isca->timers.lrwStpNorm, m_isca->timers.lrwStpEmer);
}

const char* printTimeType(TimeType_t type)
{
    switch (type)
    {
        case P2P_SN_Time:
            return "P2P_SN_Time";
        case P2P_SE_Time:
            return "P2P_SE_Time";
        case P2P_MN_Time:
            return "P2P_MN_Time";
        case P2P_ME_Time:
            return "P2P_ME_Time";
        case LRW_SN_Time:
            return "LRW_SN_Time";
        case LRW_SE_Time:
            return "LRW_SE_Time";
        case LRW_MN_Time:
            return "LRW_MN_Time";
        case LRW_ME_Time:
            return "LRW_ME_Time";
        default:
            break;
    }
    return "Unknow error";
}

void changeTime(TimeType_t type, uint32_t time)
{
    printf("[SM] Warning! Change %s\r\n", printTimeType(type));
	
    if(time > 0xFFFFF)
		time = 0xFFFFF;
    
    switch(type)
    {
        case P2P_SN_Time:
            m_isca->timers.p2pStpNorm = time;
            updateP2PTimer(time);
        break;
        case P2P_SE_Time:
        	m_isca->timers.p2pStpEmer = time;
            updateP2PTimer(time);
        break;
        case P2P_MN_Time:
        break;
        case P2P_ME_Time:
        break;
        case LRW_SN_Time:
        	m_isca->timers.lrwStpNorm = time;
            updateLRWTimer(time);
        break;
        case LRW_SE_Time:
        	m_isca->timers.lrwStpEmer = time;
	        updateLRWTimer(time);
        break;
        case LRW_MN_Time:
        break;
        case LRW_ME_Time:
        break;
    }

}
static void gsmTimerCallback(void* arg)
{
    int64_t now = esp_timer_get_time();
    ESP_LOGW(TAG, "----------- GSM TIMEOUT ------------\r\n");
    m_isca->status.lastGSMTick = now;
    uint64_t gsmExpiryTime;
    esp_err_t ret = esp_timer_get_period(gsmTimer, &gsmExpiryTime);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Get GSM TIMER failed %d", ret);
    }
    m_isca->status.GSMAlarm = now + gsmExpiryTime;
    xTaskNotify(xTaskToNotify, BIT_GSM_TX_REQ, eSetBits);
}

static void p2pTimerCallback(void* arg)
{
    int64_t now = esp_timer_get_time();
    ESP_LOGW(TAG, "----------- P2P TIMEOUT ------------\r\n");
    m_isca->status.lastP2PTick = now;
    uint64_t p2pExpiryTime;
    esp_err_t ret = esp_timer_get_period(p2pTimer, &p2pExpiryTime);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Get P2P TIMER failed %d", ret);
    }
    m_isca->status.P2PAlarm = now + p2pExpiryTime;
    xTaskNotify(xTaskToNotify, BIT_P2P_TX_REQ, eSetBits);
}

static void lrwTimerCallback(void* arg)
{
    int64_t now = esp_timer_get_time();
    ESP_LOGW(TAG, "------------ LRWTIMEOUT ------------\r\n");
    m_isca->status.lastLRWTick = now;
    uint64_t lrwExpiryTime;
    esp_err_t ret = esp_timer_get_period(lrwTimer, &lrwExpiryTime);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Get LRW TIMER failed %d", ret);
    }
    m_isca->status.LRWAlarm = now + lrwExpiryTime;
    xTaskNotify(xTaskToNotify, BIT_LRW_TX_REQ, eSetBits);
}

static void app_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == APP_EVENT && event_id == APP_EVENT_P2P_RX)
    {
        LoRaP2PRx_t *p2pRx_p = (LoRaP2PRx_t*) event_data;
        memcpy(&p2pRx, p2pRx_p, sizeof(LoRaP2PRx_t));
        xQueueSend(xQueueP2PRx, &p2pRx, 0);
        xTaskNotify(xTaskToNotify, BIT_P2P_RX, eSetBits);
    }

    if (event_base == APP_EVENT && event_id == APP_EVENT_LRW_RX)
    {
        LoRaLRWRx_t *lrwRX_p = (LoRaLRWRx_t*) event_data;
        memcpy(&lrwRx, lrwRX_p, sizeof(LoRaLRWRx_t));
        xQueueSend(xQueueLRWRx, &lrwRx, 0);
        xTaskNotify(xTaskToNotify, BIT_LRW_RX, eSetBits);
    }

    if (event_base == APP_EVENT && event_id == APP_EVENT_GSM_RX)
    {
        GSMRx_t *gsmRX_p = (GSMRx_t*) event_data;
        memcpy(&gsmRx, gsmRX_p, sizeof(GSMRx_t));
        xQueueSend(xQueueGSMRx, &gsmRx, 0);
        xTaskNotify(xTaskToNotify, BIT_GSM_RX, eSetBits);
    }

    if(event_base == APP_EVENT && event_id == APP_EVENT_P2P_TX_RES)
    {
        LoRaP2PReq_t *p2pTxRes_p = (LoRaP2PReq_t*) event_data;
        memcpy(&p2pTxRes, p2pTxRes_p, sizeof(LoRaP2PReq_t));
        xTaskNotify(xTaskToNotify, BIT_P2P_TX_RES, eSetBits);
    }

    if(event_base == APP_EVENT && event_id == APP_EVENT_LRW_TX_RES)
    {
        LoRaLRWTxRes_t *lrwTxRes_p = (LoRaLRWTxRes_t*) event_data;
        memcpy(&lrwTxRes, lrwTxRes_p, sizeof(LoRaLRWTxRes_t));
        xTaskNotify(xTaskToNotify, BIT_LRW_TX_RES, eSetBits);
    }
    if(event_base == APP_EVENT && event_id == APP_EVENT_GSM_TX_RES)
    {
        GSMTxRes_t *gsmTxRes_p = (GSMTxRes_t*) event_data;
        memcpy(&gsmTxRes, gsmTxRes_p, sizeof(GSMTxRes_t));
        xTaskNotify(xTaskToNotify, BIT_GSM_TX_RES, eSetBits);
    }

    if(event_base == APP_EVENT && event_id == APP_EVENT_REQ_LRW_SEND_STATUS)
    {
        xTaskNotify(xTaskToNotify, BIT_LRW_REQ_TX_STATUS, eSetBits);
    }

    if(event_base == APP_EVENT && event_id == APP_EVENT_SEND_POS)
    {
        xTaskNotify(xTaskToNotify, BIT_LRW_TX_REQ | BIT_GSM_TX_REQ | BIT_P2P_TX_REQ, eSetBits);
    }

    if(event_base == APP_EVENT && event_id == APP_EVENT_SENSORS_STATUS)
    {
        SensorsStatus_t *_sensorStatus_p = (SensorsStatus_t*) event_data;
        memcpy(&_sensorStatus,_sensorStatus_p, sizeof(SensorsStatus_t) );
        xTaskNotify(xTaskToNotify, BIT_SENSORS_STATUS, eSetBits);
    }

    if(event_base == APP_EVENT && event_id == APP_EVENT_MOVEMENT)
    {
        bool *mov_p = (bool*) event_data;
        m_isca->status.flags.asBit.movement = *mov_p;
        if(*mov_p == true)
            printf("        [STATE] MOVEMENT: TRUE\r\n");
        else
            printf("        [STATE] MOVEMENT: FALSE\r\n");
    }

}

void stateTask (void* pvParameters)
{
    xTaskToNotify = xTaskGetCurrentTaskHandle();
    
    m_isca = (Isca_t *)pvParameters;
    
    xQueueP2PRx = xQueueCreate(QUEUE_P2P_RX_SIZE, sizeof(LoRaP2PRx_t));
    xQueueLRWRx = xQueueCreate(QUEUE_LRW_RX_SIZE, sizeof(LoRaLRWRx_t));
    xQueueGSMRx = xQueueCreate(QUEUE_GSM_RX_SIZE, sizeof(GSMRx_t));

    esp_event_handler_instance_register(APP_EVENT, ESP_EVENT_ANY_ID, &app_event_handler, nullptr, nullptr);
    
    const esp_timer_create_args_t p2pTimerArgs = {
            .callback = &p2pTimerCallback,
            .name = "p2pTimer"
    };

    const esp_timer_create_args_t lrwTimerArgs = {
            .callback = &lrwTimerCallback,
            .name = "lrwTimer"
    };

    const esp_timer_create_args_t gsmTimerArgs = {
            .callback = &gsmTimerCallback,
            .name = "gsmTimer"
    };

    ESP_ERROR_CHECK(esp_timer_create(&p2pTimerArgs, &p2pTimer));
    ESP_ERROR_CHECK(esp_timer_create(&lrwTimerArgs, &lrwTimer));
    ESP_ERROR_CHECK(esp_timer_create(&gsmTimerArgs, &gsmTimer));
        
    m_isca->timers.p2pStpEmer = 0x12C;
    m_isca->timers.p2pStpNorm = 0x12C;
    m_isca->timers.lrwStpEmer = 0x12C;
    m_isca->timers.lrwStpNorm = 0x12C;
    m_isca->timers.gsmStpEmer = 0x12C;
    m_isca->timers.gsmStpNorm = 0x12C;

    m_isca->timers.p2pMovEmer = 0xB4;
    m_isca->timers.p2pMovNorm = 0xB4;
    m_isca->timers.lrwMovEmer = 0xB4;
    m_isca->timers.lrwMovNorm = 0xB4;
    m_isca->timers.gsmMovEmer = 0xB4;
    m_isca->timers.gsmMovNorm = 0xB4;
    
    uint32_t ulNotifiedValue = 0;
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS( TIMEOUT_STATE_MACHINE );
    
    state = SM_UPDATE_TIMERS;

    while(1)
    {
        switch(state)
        {
            case SM_WAIT_FOR_EVENT:
            {             
                if(ulNotifiedValue == 0 && (uxQueueSpacesAvailable(xQueueLRWRx) == QUEUE_LRW_RX_SIZE) &&
                            (uxQueueSpacesAvailable(xQueueP2PRx) == QUEUE_P2P_RX_SIZE) && 
                            (uxQueueSpacesAvailable(xQueueGSMRx) == QUEUE_GSM_RX_SIZE))
                {
                    xTaskNotifyWait( pdFALSE, pdFALSE, &ulNotifiedValue, xMaxBlockTime );
                }

                int64_t now = esp_timer_get_time();

                // ESP_LOGI(TAG, "flags: %02lX | Timeout P2P: %03llds | Timeout LRW: %03llds | Timeout GSM: %03llds", 
                //                 ulNotifiedValue,
                //                 (int64_t)((m_isca->status.P2PAlarm - now))/1000000, 
                //                 (int64_t)((m_isca->status.LRWAlarm - now))/1000000,
                //                 (int64_t)((m_isca->status.GSMAlarm - now))/1000000);

                if(uxQueueSpacesAvailable(xQueueP2PRx) != QUEUE_P2P_RX_SIZE)
                {
                    state = SM_P2P_RX;
                } 
                else if(uxQueueSpacesAvailable(xQueueLRWRx) != QUEUE_LRW_RX_SIZE)
                {
                    state = SM_LRW_RX;
                }
                else if(uxQueueSpacesAvailable(xQueueGSMRx) != QUEUE_GSM_RX_SIZE)
                {
                    state = SM_GSM_RX;
                }
                else if( ( ulNotifiedValue & BIT_P2P_TX_REQ ) != 0 )
                {
                    state = SM_P2P_TX_REQ;
                    ulTaskNotifyValueClear( xTaskToNotify, BIT_P2P_TX_REQ );
                    ulNotifiedValue &= ~BIT_P2P_TX_REQ;
                } 
                else if( ( ulNotifiedValue & BIT_P2P_TX_RES ) != 0 )
                {
                    state = SM_P2P_TX_RES;
                    ulTaskNotifyValueClear( xTaskToNotify, BIT_P2P_TX_RES );
                    ulNotifiedValue &= ~BIT_P2P_TX_RES;
                }
                else if( ( ulNotifiedValue & BIT_P2P_RX ) != 0 )
                {
                    state = SM_P2P_RX;
                    ulTaskNotifyValueClear( xTaskToNotify, BIT_P2P_RX );
                    ulNotifiedValue &= ~BIT_P2P_RX;
                } 
                else if( ( ulNotifiedValue & BIT_LRW_TX_REQ ) != 0 )
                {
                    state = SM_LRW_TX_REQ;
                    ulTaskNotifyValueClear( xTaskToNotify, BIT_LRW_TX_REQ );
                    ulNotifiedValue &= ~BIT_LRW_TX_REQ;
                }
                else if ((ulNotifiedValue & BIT_LRW_REQ_TX_STATUS) != 0)
                {
                    state = SM_SEND_LRW_STATUS;
                    ulTaskNotifyValueClear( xTaskToNotify, BIT_LRW_REQ_TX_STATUS );
                    ulNotifiedValue &= ~BIT_LRW_REQ_TX_STATUS;
                }
                else if( ( ulNotifiedValue & BIT_LRW_TX_RES ) != 0 )
                {
                    state = SM_LRW_TX_RES;
                    ulTaskNotifyValueClear( xTaskToNotify, BIT_LRW_TX_RES );
                    ulNotifiedValue &= ~BIT_LRW_TX_RES;
                }
                else if( ( ulNotifiedValue & BIT_LRW_RX ) != 0 )
                {
                    state = SM_LRW_RX;
                    ulTaskNotifyValueClear( xTaskToNotify, BIT_LRW_RX );
                    ulNotifiedValue &= ~BIT_LRW_RX;
                } 
                else if( ( ulNotifiedValue & BIT_SENSORS_STATUS) != 0 )
                {
                    state = SM_SENSORS_STATUS;
                    ulTaskNotifyValueClear( xTaskToNotify, BIT_SENSORS_STATUS );
                    ulNotifiedValue &= ~BIT_SENSORS_STATUS;
                } 
                else if(( ulNotifiedValue & BIT_GSM_TX_REQ) != 0)
                {
                    state = SM_GSM_TX_REQ;
                    ulTaskNotifyValueClear( xTaskToNotify, BIT_GSM_TX_REQ );
                    ulNotifiedValue &= ~BIT_GSM_TX_REQ;
                }
                else if(( ulNotifiedValue & BIT_GSM_TX_RES) != 0)
                {
                    state = SM_GSM_TX_RES;
                    ulTaskNotifyValueClear( xTaskToNotify, BIT_GSM_TX_RES );
                    ulNotifiedValue &= ~BIT_GSM_TX_RES;
                }
                else if(( ulNotifiedValue & BIT_GSM_RX) != 0)
                {
                    state = SM_GSM_RX;
                    ulTaskNotifyValueClear( xTaskToNotify, BIT_GSM_RX );
                    ulNotifiedValue &= ~BIT_GSM_RX;
                }
            }
            break;
            
            case SM_P2P_TX_REQ:
            {
                static uint8_t counter = 0;
                // PositionP2PUnion_t pos;
                // memset(&pos, 0, sizeof(PositionP2PUnion_t));

                // float P2PBattery = (float)(m_isca->status.batteryMiliVolts / 20.0);

                // pos.param.loraId[0] = (uint8_t) m_isca->rom.loraId & 0xFFFFFF;
                // pos.param.loraId[1] = (uint8_t) (m_isca->rom.loraId >> 8) & 0xFFFF;
                // pos.param.loraId[2] = (uint8_t) (m_isca->rom.loraId >> 16) & 0xFF;
                // pos.param.packetType = 80;
                // pos.param.flags.headingGps = 511;
                // pos.param.flags.batteryVoltageInfos = 2;

                // pos.param.batteryVoltage = (uint8_t)(P2PBattery < 0 ? (P2PBattery - 0.5) : (P2PBattery + 0.5));;
                // pos.param.flags.accelerometerStatus = m_isca->status.flags.asBit.movement;
                // pos.param.flags.criticalBatteryStatus = m_isca->status.flags.asBit.lowBattery;
                // pos.param.flags.powerSupplyStatus = m_isca->status.flags.asBit.powerSupply;
                // pos.param.flags.emergencyStatus = m_isca->status.flags.asBit.emergency;
                // pos.param.header.sequenceNumber = counter;

                // m_isca->status.P2PCount = counter;
                // if (counter++ > 63)
                //     counter = 0;

                // pos.array[5] = dallas_crc8((const uint8_t*) (pos.array),
                //     sizeof(PositionP2P_t));
                
                CommandP2PUnion_t cmd;
                memset(&cmd, 0, sizeof(CommandP2PUnion_t));

                float P2PBattery = (float)(m_isca->status.batteryMiliVolts / 20.0);

                cmd.param.loraIdGw[0] = (uint8_t) m_isca->rom.loraId & 0xFFFFFF;
                cmd.param.loraIdGw[1] = (uint8_t) (m_isca->rom.loraId >> 8) & 0xFFFF;
                cmd.param.loraIdGw[2] = (uint8_t) (m_isca->rom.loraId >> 16) & 0xFF;
                cmd.param.packetType = 0xFF;
                cmd.param.loraIdReceiveCommand[2] = 0xFF;
                cmd.param.loraIdReceiveCommand[1] = 0xFF;
                cmd.param.loraIdReceiveCommand[0] = 0xFF;
                cmd.param.header.sequenceNumber = 0x20;
                cmd.param.param_desc1 = 0x01;
                cmd.param.param_desc2 = 0x01;
                cmd.param.param_desc3 = 0x00;
                cmd.param.param_desc4 = 0x01;
                cmd.param.loraEmergencyCommand = 0x01;

                cmd.param.crc8 = dallas_crc8((const uint8_t*) (cmd.array),
                        sizeof(CommandP2PUnion_t));
                        
                memset(&p2pTx.payload.buffer, 0, sizeof(p2pTx.payload.buffer));
                memcpy(&p2pTx.payload.buffer, cmd.array, sizeof(CommandP2P_t));
                p2pTx.payload.size = sizeof(CommandP2P_t);
                
                p2pTx.params.txFreq = m_isca->config.p2p.txFreq;
                p2pTx.params.txPower = m_isca->config.p2p.txPower;
                p2pTx.params.BW = m_isca->config.p2p.bw;
                p2pTx.params.SF = m_isca->config.p2p.sf;
                p2pTx.params.CR = m_isca->config.p2p.cr;
                p2pTx.params.rxFreq = m_isca->config.p2p.rxFreq;
                p2pTx.params.rxDelay = m_isca->config.p2p.rxDelay;
                p2pTx.params.rxTimeout = m_isca->config.p2p.rxTimeout;
                p2pTx.params.txTimeout = m_isca->config.p2p.txTimeout;
                esp_event_post(APP_EVENT, APP_EVENT_P2P_TX_REQ, (void*)&p2pTx, sizeof(LoRaP2PReq_t), 0);

                state = SM_WAIT_FOR_EVENT;
            }
            break;

            case SM_P2P_TX_RES:
            {
                char payload[256];
                for(int i = 0; i < p2pTxRes.payload.size; i++)
                {
                    snprintf(&payload[i*3], 256, "%02X ",p2pTxRes.payload.buffer[i]);
                }
                payload[p2pTxRes.payload.size * 3] = 0x00;
                printf("[LORA] P2P TX: %s\r\n", payload);

                PositionP2PUnion_t pos;
                memcpy(&pos, p2pTxRes.payload.buffer, sizeof(PositionP2PUnion_t));
                printf("[LORA] PARSE P2P TX Counter: %02u | ", pos.param.header.sequenceNumber);
                printf("LoraID: %.2X%.2X%.2X | ", pos.param.loraId[0], pos.param.loraId[1], pos.param.loraId[2]);
                printf("Batt: %.2X | ", pos.param.batteryVoltage);
                printf("Accelerometer: %s | ", (pos.param.flags.accelerometerStatus) ? "ON" : "OFF");
                printf("criticalBatteryStatus: %s | ", (pos.param.flags.criticalBatteryStatus) ? "ON" : "OFF");
                printf("powerSupplyStatus: %s | ", (pos.param.flags.powerSupplyStatus) ? "ON" : "OFF");
                printf("Emergency: %s\r\n", (pos.param.flags.emergencyStatus) ? "ON" : "OFF");
                state = SM_WAIT_FOR_EVENT;
            }
            break;

            case SM_P2P_RX:
            {
                LoRaP2PRx_t _rx;
                if(xQueueReceive(xQueueP2PRx, &_rx, 10) == pdPASS)
                {
                    char payload[256];
					for(int i = 0; i < _rx.payload.size; i++)
					{
						snprintf(&payload[i*3], 256, "%02X ", _rx.payload.buffer[i]);
					}
					payload[_rx.payload.size * 3] = 0x00;
                    printf("[LORA] P2P RCV: %s\r\n", payload);

                    if(_rx.payload.size == sizeof(CommandP2P_t))
                    {
                        CommandP2PUnion_t commandReceived;
                        memcpy(commandReceived.array, _rx.payload.buffer, _rx.payload.size);

                        uint8_t lCrc = commandReceived.param.crc8;
                        commandReceived.param.crc8 = 0;
                        uint8_t crcValidation = dallas_crc8(commandReceived.array, sizeof(CommandP2P_t));

                        if(crcValidation == lCrc)
                        {
                            uint32_t idLora = 0;
                            idLora += commandReceived.param.loraIdReceiveCommand[0];
                            idLora += (commandReceived.param.loraIdReceiveCommand[1] << 8);
                            idLora += (commandReceived.param.loraIdReceiveCommand[2] << 16);

                            if (idLora == m_isca->rom.loraId)
                            {
                                printf("[LORA] Message for me from %02X%02X%02X | rssi: %d | snr: %d | ",
                                                            commandReceived.param.loraIdGw[2],
                                                            commandReceived.param.loraIdGw[1],
                                                            commandReceived.param.loraIdGw[0],
                                                            _rx.params.rssi, _rx.params.snr);

                                if (commandReceived.param.packetType == 0xFF && 
                                        commandReceived.param.loraIdGw[0] == 0xFF &&
                                        commandReceived.param.loraIdGw[1] == 0xFF &&
                                        commandReceived.param.loraIdGw[2] == 0xFF)
                                {
                                    printf("\r\n**********JIGA ISCA LORA DONE**********\r\n");
                                    m_isca->jiga.canSendP2P = false;
                                }

                            }
                            else
                            {
                                printf("[LORA] No P2P_RX for me =(\r\n");
                            }
                        }
                    }
                }
                state = SM_WAIT_FOR_EVENT;
            }
            break;

            case SM_LRW_TX_REQ:
            {                
                lrwTx.params.port = m_isca->config.lrw.posPort;
                lrwTx.params.confirmed = m_isca->config.lrw.confirmed;
               
                memset(&lrwTx.payload.buffer, 0, sizeof(lrwTx.payload.buffer));
                lrwTx.payload.buffer[0] = m_isca->config.lrw.protocol;
                lrwTx.payload.buffer[1] = (m_isca->rom.loraId >> 16) & 0xFF;
                lrwTx.payload.buffer[2] = (m_isca->rom.loraId >> 8) & 0xFF;
                lrwTx.payload.buffer[3] = m_isca->rom.loraId & 0xFF;
                lrwTx.payload.buffer[4] = m_isca->status.temperatureCelsius;
                lrwTx.payload.buffer[5] = (uint8_t)((m_isca->status.batteryMiliVolts & 0xFF00)>>8);
                lrwTx.payload.buffer[6] = (uint8_t)(m_isca->status.batteryMiliVolts & 0x00FF);
                lrwTx.payload.buffer[7] = m_isca->status.flags.asArray[0];
                lrwTx.payload.buffer[8] = m_isca->status.flags.asArray[1];
                lrwTx.payload.size = 9;               
                
                esp_event_post(APP_EVENT, APP_EVENT_LRW_TX_REQ, &lrwTx, sizeof(LoRaLRWTxReq_t), 0);

                state = SM_WAIT_FOR_EVENT;
            }
            break;

            case SM_SEND_LRW_STATUS:
            {

                lrwTx.params.port = m_isca->config.lrw.posPort;
                lrwTx.params.confirmed = m_isca->config.lrw.confirmed;

                memset(&lrwTx.payload.buffer, 0, sizeof(lrwTx.payload.buffer));

                DoubleWordArrayUnion_t timeArray[4];

                timeArray[0].doubleWord = (uint64_t)(m_isca->timers.p2pMovNorm)<<20 | m_isca->timers.p2pMovEmer;
                timeArray[1].doubleWord = (uint64_t)(m_isca->timers.p2pStpNorm)<<20 | m_isca->timers.p2pStpEmer;
                timeArray[2].doubleWord = (uint64_t)(m_isca->timers.lrwMovNorm)<<20 | m_isca->timers.lrwMovEmer;
                timeArray[3].doubleWord = (uint64_t)(m_isca->timers.lrwStpNorm)<<20 | m_isca->timers.lrwStpEmer;

                uint8_t timesPayload[20] = {};

                int x = 0, y = 4;
                for (int z = 0; z < 20; z++)
                {
                    x = z/5;
                    timesPayload[z] = timeArray[x].array[y];
                    if(--y < 0)
                        y = 4;
                }

                lrwTx.payload.buffer[0] = m_isca->config.lrw.protocol;
                lrwTx.payload.buffer[1] = m_isca->rom.hwVer;
                lrwTx.payload.buffer[2] = m_isca->config.fw.versionByte;
                lrwTx.payload.buffer[3] = (m_isca->rom.loraId >> 16) & 0xFF;
                lrwTx.payload.buffer[4] = (m_isca->rom.loraId >> 8) & 0xFF;
                lrwTx.payload.buffer[5] = m_isca->rom.loraId & 0xFF;
                lrwTx.payload.buffer[6] = m_isca->status.temperatureCelsius;
                lrwTx.payload.buffer[7] = (uint8_t)((m_isca->status.batteryMiliVolts & 0xFF00)>>8);
                lrwTx.payload.buffer[8] = (uint8_t)(m_isca->status.batteryMiliVolts & 0x00FF);
                lrwTx.payload.buffer[9] = m_isca->status.flags.asArray[0];
                lrwTx.payload.buffer[10] = m_isca->status.flags.asArray[1];

                for(int k = 0; k < 20; k++)
                {
                    lrwTx.payload.buffer[k + 11] = timesPayload[k];
                }
                lrwTx.payload.buffer[31] = m_isca->config.ble.power;
                lrwTx.payload.buffer[32] = (m_isca->timers.bleAdvTime & 0xff00) >> 8;
                lrwTx.payload.buffer[33] = (m_isca->timers.bleAdvTime & 0xff);

                lrwTx.payload.size = 34;

                esp_event_post(APP_EVENT, APP_EVENT_LRW_TX_REQ, &lrwTx, sizeof(LoRaLRWTxReq_t), 0);

                state = SM_WAIT_FOR_EVENT;
            }
            break;

            case SM_LRW_TX_RES:
            {
                char payload[256];
                for(int i = 0; i < lrwTxRes.payload.size; i++)
                {
                    snprintf(&payload[i*3], 256, "%02X ",lrwTxRes.payload.buffer[i]);
                }
                payload[lrwTxRes.payload.size * 3] = 0x00;
                printf("[LORA] LRW TX: %s\r\n", payload);

                printf("[LORA] LRW TX Uplink: %ld | channel: %d | length: %d \r\n", lrwTxRes.done.upLinkCounter, lrwTxRes.done.channel, lrwTxRes.done.length);
				
                if(lrwTxRes.payload.size == 9)
                {
                    printf("[LORA] PARSE TX LRW Protocol Version: %02X | ", lrwTxRes.payload.buffer[0]);
                    printf("LoRaID: %02X %02X %02X | ", lrwTxRes.payload.buffer[1], lrwTxRes.payload.buffer[2], lrwTxRes.payload.buffer[3]);
                    printf("Temp: 0x%02X = %d | ", lrwTxRes.payload.buffer[4], lrwTxRes.payload.buffer[4]);
                    printf("Battery: %d mV | ", (int)((lrwTxRes.payload.buffer[5]<<8) + lrwTxRes.payload.buffer[6]));
                    printf("Flags: %02X %02X\r\n", lrwTxRes.payload.buffer[7], lrwTxRes.payload.buffer[8]);

                }
                else if (lrwTxRes.payload.size == 34)
                {
                    printf("[LORA] PARSE LRW Status | Protocol Version: 0x%02X | ", lrwTxRes.payload.buffer[0]);
                    printf("HW Ver: 0x%02X | FW Ver: 0x%02X |\r\n", lrwTxRes.payload.buffer[1], lrwTxRes.payload.buffer[2]);
                    printf("| LoRaID: 0x%02X%02X%02X = %d | ", lrwTxRes.payload.buffer[3], lrwTxRes.payload.buffer[4], \
                                                                lrwTxRes.payload.buffer[5], 
                                                                ( (lrwTxRes.payload.buffer[3]<<16) + (lrwTxRes.payload.buffer[4]<<8) + lrwTxRes.payload.buffer[5]));
                    printf("Temp: 0x%02X = %d | ", lrwTxRes.payload.buffer[6], lrwTxRes.payload.buffer[6]);
                    printf("Battery: 0x%02X%02X = %d mV | ", lrwTxRes.payload.buffer[7], lrwTxRes.payload.buffer[8],
                                                    (lrwTxRes.payload.buffer[7]<<8) + lrwTxRes.payload.buffer[8]);
                    printf("Flags: 0x%02X%02X |\r\n", lrwTxRes.payload.buffer[9], lrwTxRes.payload.buffer[10]);
                    printf("| Emergency: %s | LowBattery: %s | Jammer: %s | Movement: %s |\r\n",
                            m_isca->status.flags.asBit.emergency?"ON":"OFF", m_isca->status.flags.asBit.lowBattery?"ON":"OFF",
                            m_isca->status.flags.asBit.jammer?"ON":"OFF", m_isca->status.flags.asBit.movement?"ON":"OFF");
                    printf("| BLE: %s | StockMode: %s | Output: %s | Input: %s |\r\n",
                            m_isca->status.flags.asBit.bleStatus?"ON":"OFF", m_isca->status.flags.asBit.stockMode?"ON":"OFF",
                            m_isca->status.flags.asBit.output?"ON":"OFF", m_isca->status.flags.asBit.input?"ON":"OFF");
                    printf("| StatusBattery: 0x%02X = %d | LastResetReason: 0x%02X = %d | PowerSupply: 0x%02X = %d |\r\n",
                            m_isca->status.flags.asBit.statusBattery, m_isca->status.flags.asBit.statusBattery,
                            m_isca->status.flags.asBit.lastResetReason, m_isca->status.flags.asBit.lastResetReason,
                            m_isca->status.flags.asBit.powerSupply, m_isca->status.flags.asBit.powerSupply);
                    printf("| p2pMovNormal: 0x%05X = %d | p2pMovEmerg: 0x%05X = %d | p2pStpNormal: 0x%05X = %d | p2pStpEmerg: 0x%05X = %d |\r\n",\
                            m_isca->timers.p2pMovNorm, m_isca->timers.p2pMovNorm, m_isca->timers.p2pMovEmer, m_isca->timers.p2pMovEmer,
                            m_isca->timers.p2pStpNorm, m_isca->timers.p2pStpNorm, m_isca->timers.p2pStpEmer, m_isca->timers.p2pStpEmer);
                    printf("| lrwMovNormal: 0x%05X = %d | lrwMovEmerg: 0x%05X = %d | lrwStpNormal: 0x%05X = %d | lrwStpEmerg: 0x%05X = %d |\r\n",\
                            m_isca->timers.lrwMovNorm, m_isca->timers.lrwMovNorm, m_isca->timers.lrwMovEmer, m_isca->timers.lrwMovEmer,
                            m_isca->timers.lrwStpNorm, m_isca->timers.lrwStpNorm, m_isca->timers.lrwStpEmer, m_isca->timers.lrwStpEmer);
                    printf("| BLE Power: 0x%02X = %d | BLE Time: 0x%04X = %d | \r\n", m_isca->config.ble.power, m_isca->config.ble.power,
                        m_isca->timers.bleAdvTime, m_isca->timers.bleAdvTime);

                }
                
                state = SM_WAIT_FOR_EVENT;
            }
            break;

            case SM_LRW_RX:
            {
                LoRaLRWRx_t _rx;
                if(xQueueReceive(xQueueLRWRx, &_rx, 10) == pdPASS)
                {
                    lorawanRX(&_rx);
                }
                state = SM_WAIT_FOR_EVENT;
            }
            break;
            
            case SM_UPDATE_TIMERS:
            {
                if(m_isca->status.flags.asBit.emergency)
                {
                    stopTimers();
                    // updateP2PTimer(m_isca->timers.p2pStpEmer);
                    // updateLRWTimer(m_isca->timers.lrwStpEmer);
                    // updateGSMTimer(m_isca->timers.gsmStpEmer);
                }
                else
                {
                    stopTimers();
                    // updateP2PTimer(m_isca->timers.p2pStpNorm);
                    // updateLRWTimer(m_isca->timers.lrwStpNorm);
                    // updateGSMTimer(m_isca->timers.gsmStpNorm);

                }
                state =SM_WAIT_FOR_EVENT;
            }
            break;

            case SM_ENTER_EMERGENCY:
                m_isca->status.flags.asBit.emergency = true;
                state = SM_UPDATE_TIMERS;
            break;

            case SM_EXIT_EMERGENCY:
                m_isca->status.flags.asBit.emergency = false;
                state = SM_UPDATE_TIMERS;
            break;

            case SM_SENSORS_STATUS:
            {
                memcpy(m_isca->status.acc, _sensorStatus.acc, sizeof(m_isca->status.acc));
                m_isca->status.temperatureFloat = _sensorStatus.temperature;
                m_isca->status.temperatureCelsius = (int8_t)(_sensorStatus.temperature < 0 ? (_sensorStatus.temperature - 0.5) : (_sensorStatus.temperature + 0.5));
                m_isca->status.flags.asBit.statusBattery = _sensorStatus.batStatus;
                m_isca->status.batteryMiliVolts = _sensorStatus.batVoltage;
                if(_sensorStatus.batVoltage <= BATT_CRITICAL_VOLTAGE)
                {
                    m_isca->status.flags.asBit.lowBattery = 1;
                }
                else
                {
                    m_isca->status.flags.asBit.lowBattery = 0;
                }
                
                if(m_isca->status.flags.asBit.statusBattery == BAT_DISCHARGING)
                {
                    m_isca->status.flags.asBit.powerSupply = 0;   
                }
                else
                {
                    m_isca->status.flags.asBit.powerSupply = 1;
                }

                ESP_LOGI(TAG, "[ACC](mg) x:%ld | y:%ld | z:%ld | [TEMP]: %02.2foC | [batVoltage]: %04dmV | [batStatus]: %s", 
                    m_isca->status.acc[0], m_isca->status.acc[1],m_isca->status.acc[2],  m_isca->status.temperatureFloat, 
                    m_isca->status.batteryMiliVolts,
                    printBatStatus((SensorsBatStatus_t)m_isca->status.flags.asBit.statusBattery));
                
                state = SM_WAIT_FOR_EVENT;
            }
            break;

            case SM_GSM_TX_REQ:
            {
                static uint16_t counter = 0;
                gsmTx.payload.header = 0xD7;
                uint64_t serial = 0;
                serial = (uint64_t) 109*100000000 + m_isca->rom.loraId;
                gsmTx.payload.serialNumber[0] = (serial >> 32) & 0xFF;
                gsmTx.payload.serialNumber[1] = (serial >> 24) & 0xFF;
                gsmTx.payload.serialNumber[2] = (serial >> 16) & 0xFF;
                gsmTx.payload.serialNumber[3] = (serial >> 8) & 0xFF;
                gsmTx.payload.serialNumber[4] = (serial) & 0xFF;
                gsmTx.payload.fw[0] = 0xDD;
                gsmTx.payload.fw[1] = 0xEE;
                gsmTx.payload.hw = m_isca->rom.hwVer;
                gsmTx.payload.protocol = 0xCC;
                gsmTx.payload.counter[0] = (counter >> 8) & 0xFF;
                gsmTx.payload.counter[1] = counter & 0xFF;
                m_isca->status.GSMCount = counter;
                if(counter == 0xFFFF)
                    counter = 0;
                else
                    counter++;
                struct timeval current;
                gettimeofday(&current, NULL);
                gsmTx.payload.timestamp[0] = (current.tv_sec >> 24) & 0xFF;
                gsmTx.payload.timestamp[1] = (current.tv_sec >> 16) & 0xFF;
                gsmTx.payload.timestamp[2] = (current.tv_sec >> 8) & 0xFF;
                gsmTx.payload.timestamp[3] = (current.tv_sec) & 0xFF;
                
                state = SM_WAIT_FOR_EVENT;
                gsmTx.payload.type = 0x00;
                gsmTx.payload.loraID[0] = (m_isca->rom.loraId >> 16) & 0xff;
                gsmTx.payload.loraID[1] = (m_isca->rom.loraId >> 8) & 0xff;
                gsmTx.payload.loraID[2] = (m_isca->rom.loraId) & 0xff;
                memcpy(gsmTx.payload.imei, m_isca->rom.imei, sizeof(m_isca->rom.imei));
                gsmTx.payload.temp = m_isca->status.temperatureCelsius;
                gsmTx.payload.batteryVoltage[0] = (m_isca->status.batteryMiliVolts >> 8) & 0xff;
                gsmTx.payload.batteryVoltage[1] = (m_isca->status.batteryMiliVolts) & 0xff;
                gsmTx.payload.flags.asBit.bt = m_isca->status.flags.asBit.bleStatus;
                gsmTx.payload.flags.asBit.emergency = m_isca->status.flags.asBit.emergency;
                gsmTx.payload.flags.asBit.input = m_isca->status.flags.asBit.input;
                gsmTx.payload.flags.asBit.jammerGsm = m_isca->status.flags.asBit.jammer;
                gsmTx.payload.flags.asBit.jammerLoRa = m_isca->status.flags.asBit.jammer;
                gsmTx.payload.flags.asBit.lowBattery = m_isca->status.flags.asBit.lowBattery;
                gsmTx.payload.flags.asBit.movement = m_isca->status.flags.asBit.movement;
                gsmTx.payload.flags.asBit.output = m_isca->status.flags.asBit.output;
                gsmTx.payload.flags.asBit.statusBattery = m_isca->status.flags.asBit.statusBattery;
                gsmTx.payload.flags.asBit.stockMode = m_isca->status.flags.asBit.stockMode;
                gsmTx.payload.lastRst = 0x00;
                memcpy(&gsmTx.config.apn, m_isca->config.gsm.apn, strlen(m_isca->config.gsm.apn));
                gsmTx.config.port = m_isca->config.gsm.port;
                memcpy(&gsmTx.config.server, m_isca->config.gsm.server, strlen(m_isca->config.gsm.server));
                esp_event_post(APP_EVENT, APP_EVENT_GSM_TX_REQ, &gsmTx, sizeof(GSMTxReq_t), 0);
            }
            break;

            case SM_GSM_TX_RES:
            {
                unsigned char hexArray[ESP_MODEM_C_API_STR_MAX];  
                size_t hexArrayLen= 0, encodedLen = 0;
                mbedtls_base64_decode(hexArray, sizeof(hexArray), &hexArrayLen, (const unsigned char*)gsmTxRes.base64, gsmTxRes.size);
                
                GSMTxPayload_t *element_p = (GSMTxPayload_t*)hexArray;
                uint64_t serialNumber = 0L;
                serialNumber += (((uint64_t) element_p->serialNumber[0]) << 32);
                serialNumber += (((uint64_t) element_p->serialNumber[1]) << 24);
                serialNumber += (((uint64_t) element_p->serialNumber[2]) << 16);
                serialNumber += (((uint64_t) element_p->serialNumber[3]) << 8);
                serialNumber += (((uint64_t) element_p->serialNumber[4]));

                uint64_t imei = 0L;
                imei += (((uint64_t) element_p->imei[0]) << 48);
                imei += (((uint64_t) element_p->imei[1]) << 40);
                imei += (((uint64_t) element_p->imei[2]) << 32);
                imei += (((uint64_t) element_p->imei[3]) << 24);
                imei += (((uint64_t) element_p->imei[4]) << 16);
                imei += (((uint64_t) element_p->imei[5]) << 8);
                imei += (((uint64_t) element_p->imei[6]));
                
                uint16_t fw = (element_p->fw[0] << 8);
                fw += (element_p->fw[1]);

                uint16_t count = (element_p->counter[0] << 8);
                count += element_p->counter[1];

                uint32_t timestamp = (element_p->timestamp[0] << 24);
                timestamp += (element_p->timestamp[1] << 16);
                timestamp += (element_p->timestamp[2] << 8);
                timestamp += (element_p->timestamp[3]);

                uint32_t loraId = (element_p->loraID[0] << 16);
                loraId += (element_p->loraID[1] << 8);
                loraId += (element_p->loraID[2]);
                uint16_t battery = (element_p->batteryVoltage[0] << 8);
                battery += element_p->batteryVoltage[1];
                uint16_t flags = (element_p->flags.asArray[0] << 8);
                flags += (element_p->flags.asArray[1]);

                if(gsmTxRes.lastErr != ESP_OK)
                {
                    ESP_LOGE(TAG, "GSM error [%d] %s at state: %s", gsmTxRes.lastErr,
                             esp_err_to_name(gsmTxRes.lastErr), printState(gsmTxRes.lastState));

                    element_p->flags.asBit.online = 0;
                    element_p->loraID[2] = (gsmTxRes.lastErr)>>8;
                    element_p->loraID[1] = gsmTxRes.lastErr & 0xff;
                    element_p->loraID[0] = gsmTxRes.lastState;

                    element_p->crc = 0;
                    uint8_t crc_calc = crc8_itu((uint8_t *)hexArray, hexArrayLen);
                    element_p->crc = crc_calc;
                    memset(gsmTxRes.base64, 0, sizeof(gsmTxRes.base64));
                    mbedtls_base64_encode((unsigned char *)gsmTxRes.base64, sizeof(gsmTxRes.base64), &encodedLen, (unsigned char *)hexArray, hexArrayLen);
                    uint32_t id;
                    gsmTxRes.base64[gsmTxRes.size] = 0x00;
                    esp_err_t err = storage_gsm_save_position(gsmTxRes.base64, &id);
                    ESP_LOGW(TAG, "Store GSM_POS status [%d] %s -> [%ld] %s", err, esp_err_to_name(err), id, gsmTxRes.base64);
                }
                else
                {
                    ESP_LOGI(TAG, "GSM SENT ");
                }
                printf("[PARSE] header: %d | sn: %llu | imei: %llu | fw: %04X | hw: %d\r\n",
                            element_p->header, serialNumber, imei, fw, element_p->hw);
                
                printf("\t proto: %02X | counter: %d | timestamp: %ld | type: %d | loraId: %ld\r\n",
                            element_p->protocol, count, timestamp, element_p->type, loraId);
                printf("\t temp: %d | battery: %d | crc: %d | flags: %04X | reset: %d | #erbs: %d\r\n",
                            element_p->temp, battery, element_p->crc, flags, element_p->lastRst, element_p->n_erbs);
                            
                state = SM_WAIT_FOR_EVENT;
            }
            break;

            case SM_GSM_RX:
            {
                GSMRx_t _rx;
                if(xQueueReceive(xQueueGSMRx, &_rx, 10) == pdPASS)
                {
                    ESP_LOGW(TAG, "[GSM RX] (%d bytes): %s", _rx.size, _rx.payload);
                }
                state = SM_WAIT_FOR_EVENT;
            }
            break;
        }
    }
}
const char* getCMDString(CommandLRWDict_t command)
{
	switch(command)
	{
	case EMERGENCY:
		return "EMERGENCY";
	case BLUETOOTH:
		return "BLUETOOTH";
	case STOCK_MODE:
		return "STOCK_MODE";
	case SET_OUTPUT:
		return "SET_OUTPUT";
	case DO_RESET:
		return "DO_RESET";
	case BLE_POWER:
		return "BLE_POWER";
	case BLE_TIME:
		return "BLE_TIME";
	case ALL_TIMES:
		return "ALL_TIMES";
	case GET_STATUS:
		return "GET_STATUS";
	case LED:
		return "LED";
	case P2P_MOV_NOR:
		return "P2P_MOV_NOR";
	case P2P_MOV_EMER:
		return "P2P_MOV_EMER";
	case P2P_STP_NOR:
		return "P2P_STP_NOR";
	case P2P_STP_EMER:
		return "P2P_STP_EMER";
	case LRW_MOV_NOR:
		return "LRW_MOV_NOR";
	case LRW_MOV_EMER:
		return "LRW_MOV_EMER";
	case LRW_STP_NOR:
		return "LRW_STP_NOR";
	case LRW_STP_EMER:
		return "LRW_STP_EMER";
	default:
		return "unknow";
	}
}

void lorawanRX(LoRaLRWRx_t* _lrwRx)
{
    LoRaLRWRxParams_t* rxParams = (LoRaLRWRxParams_t*)&_lrwRx->params;
    printf("[LORA] LRW downlink received port:%d | snr: %d  | rssi: %d {", rxParams->port,rxParams->snr, rxParams->rssi);
	for (uint8_t i = 0; i < _lrwRx->payload.size; i++)
		printf(" %02X ", _lrwRx->payload.buffer[i]);
	printf("}\r\n");
    
    if(rxParams->port == LRW_CMD_PORT)
    {
        uint8_t pktCounter = 0;
        bool error = false;
        if(_lrwRx->payload.buffer[0] == CMD_LRW_HEADER)
        {
            pktCounter = 1;
            while(pktCounter < (_lrwRx->payload.size-1) && error == false)
            {
                switch(_lrwRx->payload.buffer[pktCounter])
                {
                case EMERGENCY:
                    if(_lrwRx->payload.size - (pktCounter+1) >= CMD_PARAM_SIZE_EMER)
                    {

                        if(_lrwRx->payload.buffer[pktCounter + 1])
                            enterEmergency();
                        else
                            exitEmergency();
                        pktCounter+= CMD_PARAM_SIZE_EMER;
                    }
                    else
                    {
                        printf("[LORA] Warning! There's no Emergency Param\r\n");
                        error = 1;
                    }
                    break;

                case BLUETOOTH:
                    if(_lrwRx->payload.size - (pktCounter+1) >= CMD_PARAM_SIZE_BLE)
                    {
                        if(_lrwRx->payload.buffer[pktCounter + 1])
                            turnOnBLE();
                        else
                            turnOffBLE();
                        pktCounter+= CMD_PARAM_SIZE_BLE;
                    }
                    else
                    {
                        printf("[LORA] Warning! There's no BLE Param\r\n");
                        error = 1;
                    }
                    break;

                case STOCK_MODE:
                    if(_lrwRx->payload.size - (pktCounter+1) >= CMD_PARAM_SIZE_STOCK)
                    {
                        if(_lrwRx->payload.buffer[pktCounter + 1])
                            enterStockMode();
                        pktCounter+= CMD_PARAM_SIZE_STOCK;
                    }
                    else
                    {
                        printf("[LORA] Warning! There's no BLE Param\r\n");
                        error = 1;
                    }
                    break;

                case SET_OUTPUT:
                    if(_lrwRx->payload.size - (pktCounter+1) >= CMD_PARAM_SIZE_OUTPUT)
                    {
                        if(_lrwRx->payload.buffer[pktCounter + 1])
                            turnOnOutput();
                        else
                            turnOffOutput();
                        pktCounter+= CMD_PARAM_SIZE_OUTPUT;
                    }
                    else
                    {
                        printf("[LORA] Warning! There's no OUTPUT Param\r\n");
                        error = 1;
                    }
                    break;

                case DO_RESET:
                    if(_lrwRx->payload.size - (pktCounter+1) >= CMD_PARAM_SIZE_RESET)
                    {
                        if(_lrwRx->payload.buffer[pktCounter + 1])
                            resetRequest();
                        pktCounter+= CMD_PARAM_SIZE_RESET;
                    }
                    else
                    {
                        printf("[LORA] Warning! There's no RESET Param\r\n");
                        error = 1;
                    }
                    break;

                case BLE_POWER:
                    if(_lrwRx->payload.size - (pktCounter+1) >= CMD_PARAM_SIZE_BLE_POWER)
                    {
                        if(_lrwRx->payload.buffer[pktCounter + 1] <= 31)
                            //changeBLEPower(rxDataLRW.payload[pktCounter+1]);
                            changeBLEPower();
                        else
                            printf("[LORA] Warning! Not a valid BLE Power\r\n");
                        pktCounter+= CMD_PARAM_SIZE_BLE_POWER;
                    }
                    else
                    {
                        printf("[LORA] Warning! There's no BLE Power Param\r\n");
                        error = 1;
                    }
                    break;

                case BLE_TIME:
                    if(_lrwRx->payload.size - (pktCounter+1) >= CMD_PARAM_SIZE_BLE_TIME)
                    {
                        if(_lrwRx->payload.buffer[pktCounter + 1] <= 31)
                            //changeBLETime(rxDataLRW.payload[pktCounter+1]);
                            changeBLETime();
                        else
                            printf("[LORA] Warning! Not a valid BLE Time\r\n");
                        pktCounter+= CMD_PARAM_SIZE_BLE_TIME;
                    }
                    else
                    {
                        printf("[LORA] Warning! There's no BLE Time Param\r\n");
                        error = 1;
                    }
                    break;

                case ALL_TIMES:
                    if(_lrwRx->payload.size - (pktCounter+1) >= CMD_PARAM_SIZE_ALL_TIMES)
                    {
                        uint32_t LoRaTimesArray[8] = {};
                        uint32_t *p_LoRaTimesArray = &LoRaTimesArray[0];

                        for(int i = 0; i < 20; i+=5)
                        {
                            *p_LoRaTimesArray |= _lrwRx->payload.buffer[pktCounter + i + 1] << 12;
                            *p_LoRaTimesArray |= _lrwRx->payload.buffer[pktCounter + i + 2] << 4;
                            *p_LoRaTimesArray |= (_lrwRx->payload.buffer[pktCounter + i + 3] & 0xF0) >> 4;

                            p_LoRaTimesArray++;

                            *p_LoRaTimesArray |= (_lrwRx->payload.buffer[pktCounter + i + 3] & 0x0F) << 16;
                            *p_LoRaTimesArray |= _lrwRx->payload.buffer[pktCounter + i + 4] << 8;
                            *p_LoRaTimesArray |= (_lrwRx->payload.buffer[pktCounter + i + 5]);

                            p_LoRaTimesArray++;
                        }

                        changeLoRaTimes(&LoRaTimesArray[0]);
                        pktCounter+= CMD_PARAM_SIZE_ALL_TIMES;

                    }
                    else
                    {
                        printf("[LORA] Times Param size not valid or not present! \r\n");
                        error = 1;
                    }

                    break;

                case GET_STATUS:
                    if(_lrwRx->payload.size - (pktCounter+1) >= CMD_PARAM_SIZE_GET_STATUS)
                    {
                        if(_lrwRx->payload.buffer[pktCounter + 1])
                            sendStatus();
                        pktCounter+= CMD_PARAM_SIZE_GET_STATUS;
                    }
                    else
                    {
                        printf("[LORA] Warning! There's no GET_STATUS Param\r\n");
                        error = 1;
                    }
                    break;

                case LED:
                    if(_lrwRx->payload.size - (pktCounter+1) >= CMD_PARAM_SIZE_LED)
                    {
                        if(_lrwRx->payload.buffer[pktCounter + 1])
                            enableLed();
                        else
                            disableLed();
                        pktCounter+= CMD_PARAM_SIZE_LED;
                    }
                    else
                    {
                        printf("[LORA] Warning! There's no LED Param\r\n");
                        error = 1;
                    }
                    break;

                case P2P_MOV_NOR:
                case P2P_MOV_EMER:
                case LRW_MOV_NOR:
                case LRW_MOV_EMER:
                    if(_lrwRx->payload.size - (pktCounter+1) >= TIME_CMD_PARAM_SIZE)
                    {
                        uint32_t dummy = 0;
                        dummy = (_lrwRx->payload.buffer[pktCounter + 1] & 0x0F) << 16;
                        dummy |= _lrwRx->payload.buffer[pktCounter + 2]  << 8;
                        dummy |= _lrwRx->payload.buffer[pktCounter + 3];
                        printf("[LORA] Requested Change to %s:%ld\r\n", getCMDString((CommandLRWDict_t)_lrwRx->payload.buffer[pktCounter]),dummy);
                        printf("WARNING! Feature not implemented\r\n");
                        pktCounter+= TIME_CMD_PARAM_SIZE;
                    }
                    else
                    {
                        printf("[LORA] Warning! There's no %s parameter\r\n",  getCMDString((CommandLRWDict_t)_lrwRx->payload.buffer[pktCounter]));
                        error = 1;
                    }
                    break;

                case P2P_STP_NOR:
                    if(_lrwRx->payload.size - (pktCounter+1) >= TIME_CMD_PARAM_SIZE)
                    {
                        uint32_t dummy = 0;
                        dummy = (_lrwRx->payload.buffer[pktCounter + 1] & 0x0F) << 16;
                        dummy |= _lrwRx->payload.buffer[pktCounter + 2]  << 8;
                        dummy |= _lrwRx->payload.buffer[pktCounter + 3];
                        printf("[LORA] Changed P2P SN to %ld\r\n", dummy);
                        changeTime(P2P_SN_Time, dummy);
                        pktCounter+= TIME_CMD_PARAM_SIZE;
                    }
                    else
                    {
                        printf("[LORA] Warning! There's no P2P_SN Param\r\n");
                        error = 1;
                    }
                    break;
                case P2P_STP_EMER:
                    if(_lrwRx->payload.size - (pktCounter+1) >= TIME_CMD_PARAM_SIZE)
                    {
                        uint32_t dummy = 0;
                        dummy = (_lrwRx->payload.buffer[pktCounter + 1] & 0x0F) << 16;
                        dummy |= _lrwRx->payload.buffer[pktCounter + 2]  << 8;
                        dummy |= _lrwRx->payload.buffer[pktCounter + 3];
                        printf("[LORA] Changed P2P SE to %ld\r\n", dummy);
                        changeTime(P2P_SE_Time, dummy);
                        pktCounter+= TIME_CMD_PARAM_SIZE;
                    }
                    else
                    {
                        printf("[LORA] Warning! There's no P2P_SE Param\r\n");
                        error = 1;
                    }
                    break;
                case LRW_STP_NOR:
                    if(_lrwRx->payload.size - (pktCounter+1) >= TIME_CMD_PARAM_SIZE)
                    {
                        uint32_t dummy = 0;
                        dummy = (_lrwRx->payload.buffer[pktCounter + 1] & 0x0F) << 16;
                        dummy |= _lrwRx->payload.buffer[pktCounter + 2]  << 8;
                        dummy |= _lrwRx->payload.buffer[pktCounter + 3];
                        printf("[LORA] Changed LRW SN to %ld\r\n", dummy);
                        changeTime(LRW_SN_Time, dummy);
                        pktCounter+= TIME_CMD_PARAM_SIZE;
                    }
                    else
                    {
                        printf("[LORA] Warning! There's no LRW_SN Param\r\n");
                        error = 1;
                    }
                    break;

                case LRW_STP_EMER:
                    if(_lrwRx->payload.size - (pktCounter+1) >= TIME_CMD_PARAM_SIZE)
                    {
                        uint32_t dummy = 0;
                        dummy = (_lrwRx->payload.buffer[pktCounter + 1] & 0x0F) << 16;
                        dummy |= _lrwRx->payload.buffer[pktCounter + 2]  << 8;
                        dummy |= _lrwRx->payload.buffer[pktCounter + 3];
                        printf("[LORA] Changed LRW SE to %ld\r\n", dummy);
                        changeTime(LRW_SE_Time, dummy);
                        pktCounter+= TIME_CMD_PARAM_SIZE;
                    }
                    else
                    {
                        printf("[LORA] Warning! There's no LRW_SE Param\r\n");
                        error = 1;
                    }
                    break;

                default:
                    printf("[LORA] Warning! Not a valid command. Aborting!\r\n");
                    error = 1;
                    break;

                }
            }
            if(error)
                printf("[LORA] Error Parsing data! \r\n");
        }
        else
        {
            printf("[LORA] Warning! Not a LRW command!\r\n");
        }
    }
}