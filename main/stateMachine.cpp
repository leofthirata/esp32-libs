#include "stateMachine.hpp"
#include "esp_timer.h"
#include "lora.hpp"

static Isca_t *m_config;
static bool resetRequested = false;
static const char *TAG = "STATE";
static Isca_SM_t state;
static TaskHandle_t xTaskToNotify = NULL;

static void p2pTimerCallback(void* arg);
static void lrwTimerCallback(void* arg);
esp_timer_handle_t p2pTimer;
esp_timer_handle_t lrwTimer;

void stopLoRaTimers();

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
    setMachineState(SM_ENTER_EMERGENCY);
    xTaskNotifyGive(xTaskToNotify);
    printf("[SM] WARNING! Emergency ON!\r\n");
}

void exitEmergency()
{
    setMachineState(SM_EXIT_EMERGENCY);	
    xTaskNotifyGive(xTaskToNotify);
    printf("[SM] WARNING! Emergency OFF!\r\n");
}

void enterStockMode()
{
	if(!m_config->flags.asBit.emergency)
	{
		m_config->flags.asBit.stockMode = true;
		printf("[SM] Warning! Enter Stock Mode\r\n");
		stopLoRaTimers();
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
		m_config->flags.asBit.stockMode = false;
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
	//queueSend(QUEUE_SEND_STATUS);
}

void stopLoRaTimers()
{
    if(esp_timer_is_active(p2pTimer))
	{
		uint64_t expiryTime = 0;
		esp_timer_get_expiry_time(p2pTimer, &expiryTime);

		printf("[TIMER] Stopping Active P2P Timer %llus |\r\n", expiryTime/1000000);
		esp_timer_stop(p2pTimer);
    }

    if(esp_timer_is_active(lrwTimer))
	{
		uint64_t expiryTime = 0;
		esp_timer_get_expiry_time(lrwTimer, &expiryTime);

		printf("[TIMER] Stopping Active LRW Timer %llus |\r\n", expiryTime/1000000);
		esp_timer_stop(lrwTimer);
    }
}


void updateP2PTimer(uint32_t newTime)
{
	if(esp_timer_is_active(p2pTimer))
	{
		uint64_t expiryTime = 0;
		esp_timer_get_expiry_time(p2pTimer, &expiryTime);

		printf("[TIMER] Stopping Active P2P Timer %llus |\r\n", expiryTime/1000000);
		esp_timer_stop(p2pTimer);
		printf("[TIMER] Restarting P2P Timer new timeout %lus |\r\n", newTime);
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
        uint8_t ret_val = esp_timer_start_periodic(p2pTimer, (uint64_t)newTime*1000000);
        if(ret_val != ESP_OK)
		{
			printf("[TIMER] Error starting P2P Timer new timeout\r\n");
			while(1);
		}
    }
}

void updateLRWTimer(uint32_t newTime)
{
	if(esp_timer_is_active(lrwTimer))
	{
		uint64_t expiryTime = 0;
		esp_timer_get_expiry_time(lrwTimer, &expiryTime);

		printf("[TIMER] Stopping Active LRW Timer %llus |\r\n", expiryTime/1000000);
		esp_timer_stop(lrwTimer);
		printf("[TIMER] Restarting LRW Timer new timeout %lus |\r\n", newTime);
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
        uint8_t ret_val = esp_timer_start_periodic(lrwTimer, (uint64_t)newTime*1000000);
        if(ret_val != ESP_OK)
		{
			printf("[TIMER] Error starting LRW Timer new timeout\r\n");
			while(1);
		}
    }
}

void changeLoRaTimes(uint32_t *timeArray)
{
	m_config->p2pMovNorm = *timeArray;
	m_config->p2pMovEmer = *(timeArray + 1);
	m_config->p2pStpNorm = *(timeArray + 2);
	m_config->p2pStpEmer = *(timeArray + 3);
	m_config->lrwMovNorm = *(timeArray + 4);
	m_config->lrwMovEmer = *(timeArray + 5);
	m_config->lrwStpNorm = *(timeArray + 6);
	m_config->lrwStpEmer = *(timeArray + 7);
	printf("[SM Warning! Changed LoRa Times\r\n");
	printf("\tP2P_MN: %d | P2P_ME: %d | P2P_SN: %d | P2P_SE: %d\r\n", m_config->p2pMovNorm, m_config->p2pMovEmer, m_config->p2pStpNorm, m_config->p2pStpEmer);
	printf("\tLRW_MN: %d | LRW_ME: %d | LRW_SN: %d | LRW_SE: %d\r\n", m_config->lrwMovNorm, m_config->lrwMovEmer, m_config->lrwStpNorm, m_config->lrwStpEmer);
}


void changeP2P_SN_Time(uint32_t time)
{
	printf("[SM] Warning! Change P2P SN Time\r\n");
	if(time > 0xFFFFF)
		time = 0xFFFFF;

	m_config->p2pStpNorm = time;
	updateP2PTimer(time);
}

void changeP2P_SE_Time(uint32_t time)
{
	printf("[SM] Warning! Change P2P SE Time\r\n");
	if(time > 0xFFFFF)
		time = 0xFFFFF;

	m_config->p2pStpEmer = time;
	updateP2PTimer(time);
}

void changeLRW_SN_Time(uint32_t time)
{
	printf("[SM] Warning! Change LRW SN Time\r\n");
	if(time > 0xFFFFF)
		time = 0xFFFFF;

	m_config->lrwStpNorm = time;
	updateLRWTimer(time);
}

void changeLRW_SE_Time(uint32_t time)
{
	printf("[SM] Warning! Change LRW SE Time\r\n");
	if(time > 0xFFFFF)
		time = 0xFFFFF;

	m_config->lrwStpEmer = time;
	updateLRWTimer(time);
}



static void p2pTimerCallback(void* arg)
{
    int64_t time_since_boot = esp_timer_get_time();
    printf("\r\n\r\n[TIMER] ------------ P2P TIMEOUT ------------\r\n");
    m_config->lastP2PTick = time_since_boot;
    queueP2P();
}


static void lrwTimerCallback(void* arg)
{
    int64_t time_since_boot = esp_timer_get_time();
    printf("\r\n\r\n[TIMER] ------------ LRWTIMEOUT ------------\r\n");
    m_config->lastLRWTick = time_since_boot;
    queueLRW();
}

void stateTask (void* pvParameters)
{
    xTaskToNotify = xTaskGetCurrentTaskHandle();
    m_config = (Isca_t *)pvParameters;

    const esp_timer_create_args_t p2pTimerArgs = {
            .callback = &p2pTimerCallback,
            /* name is optional, but may help identify the timer when debugging */
            .name = "p2pTimer"
    };

    const esp_timer_create_args_t lrwTimerArgs = {
            .callback = &lrwTimerCallback,
            /* name is optional, but may help identify the timer when debugging */
            .name = "lrwTimer"
    };

    ESP_ERROR_CHECK(esp_timer_create(&p2pTimerArgs, &p2pTimer));
    ESP_ERROR_CHECK(esp_timer_create(&lrwTimerArgs, &lrwTimer));
    
    m_config->p2pStpEmer = 0x1E;
    m_config->p2pStpNorm = 0x3C;
    m_config->lrwStpEmer = 0x1E;
    m_config->lrwStpNorm = 0x3C;
    
    state = SM_UPDATE_TIMERS;
    uint64_t p2pExpiryTime = 0, lrwExpiryTime = 0;
    while(1)
    {
        //sendP2P
        //Check CMD P2P
        //Send LRW
        //Check downlink
        //save NVS
        switch(state)
        {
            case SM_WAIT_FOR_EVENT:
            {
                uint32_t ulNotificationValue = ulTaskNotifyTake( pdTRUE, pdMS_TO_TICKS(10000));
                if( ulNotificationValue != 1)
                {
                    const int64_t now = esp_timer_get_time();
                    
                    //ESP_ERROR_CHECK(esp_timer_get_expiry_time(p2pTimer, &p2pExpiryTime));
                    esp_err_t ret = esp_timer_get_period(p2pTimer, &p2pExpiryTime);
                    if (ret != ESP_OK)
                    {
                        ESP_LOGE(TAG, "expiry timer P2P failed %d", ret);
                    }
                    ret = esp_timer_get_period(lrwTimer, &lrwExpiryTime);
                    if (ret != ESP_OK)
                    {
                        ESP_LOGE(TAG, "expiry timer LRW failed %d", ret);
                    }
                    ESP_LOGW(TAG, "expiry timers P2P:%lld | LRW:%lld", ((m_config->lastP2PTick + p2pExpiryTime)-now)/1000000, ((m_config->lastLRWTick +lrwExpiryTime)-now)/1000000);
                }
            }
            break;
            
            case SM_SEND_P2P:
                queueP2P();
                state = SM_WAIT_FOR_EVENT;
            break;

            case SM_SEND_LRW:
                queueLRW();
                state = SM_WAIT_FOR_EVENT;
            break;

            case SM_UPDATE_TIMERS:
                if(m_config->flags.asBit.emergency)
                {
                    stopLoRaTimers();
                    updateP2PTimer(m_config->p2pStpEmer);
                    updateLRWTimer(m_config->lrwStpEmer);
                }
                else
                {
                    stopLoRaTimers();
                    updateP2PTimer(m_config->p2pStpNorm);
                    updateLRWTimer(m_config->lrwStpNorm);

                }
                state =SM_WAIT_FOR_EVENT;
            break;

            case SM_ENTER_EMERGENCY:
                m_config->flags.asBit.emergency = true;
                state = SM_UPDATE_TIMERS;
            break;

            case SM_EXIT_EMERGENCY:
                m_config->flags.asBit.emergency = false;
                state = SM_UPDATE_TIMERS;
            break;




        }
        vTaskDelay(1000);
    }
}