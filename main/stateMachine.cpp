#include "stateMachine.hpp"
#include "esp_timer.h"
#include "lora.hpp"

static Isca_t *m_config;
static bool resetRequested = false;
static const char *TAG = "StateTask";
static Isca_SM_t state;
static TaskHandle_t xTaskToNotify = NULL;
static QueueHandle_t xQueueP2PRx, xQueueLRWRx;

static void p2pTimerCallback(void* arg);
static void lrwTimerCallback(void* arg);
esp_timer_handle_t p2pTimer;
esp_timer_handle_t lrwTimer;

static LoRaElementLRWTx_t lrwTx;
static LoRaElementLRWRx_t lrwRx;
static LoRaElementP2PRx_t p2pRx;
static LoRaElementP2P_t p2pTx, p2pTxDone;
static LoRaElementLRWTxDone_t lrwTxDone;

// APP event base declaration
ESP_EVENT_DECLARE_BASE(APP_EVENT);

#define BIT_LRW_REQ_TX  0x01
#define BIT_LRW_RX  0x02
#define BIT_P2P_REQ_TX  0x04
#define BIT_P2P_RX  0x08
#define BIT_P2P_TX_DONE 0x10
#define BIT_LRW_TX_DONE 0x20


void stopLoRaTimers();

uint8_t dallas_crc8(const uint8_t *pdata, const uint32_t size)
{

	uint8_t crcr = 0;
	for (uint32_t i = 0; i < size; ++i)
	{
		uint8_t inbyte = pdata[i];
		for (uint8_t j = 0; j < 8; ++j)
		{
			uint8_t mix = (crcr ^ inbyte) & 0x01;
			crcr >>= 1;
			if (mix)
				crcr ^= 0x8C;
			inbyte >>= 1;
		}
	}
	return crcr;
}


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
        m_config->P2PAlarm = 0;
    }

    if(esp_timer_is_active(lrwTimer))
	{
		uint64_t expiryTime = 0;
		esp_timer_get_expiry_time(lrwTimer, &expiryTime);

		printf("[TIMER] Stopping Active LRW Timer %llus |\r\n", expiryTime/1000000);
		esp_timer_stop(lrwTimer);
        m_config->LRWAlarm = 0;
    }
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
    m_config->P2PAlarm = now + (newTime*1000000);
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
    m_config->LRWAlarm = now + (newTime*1000000);
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
            m_config->p2pStpNorm = time;
            updateP2PTimer(time);
        break;
        case P2P_SE_Time:
        	m_config->p2pStpEmer = time;
            updateP2PTimer(time);
        break;
        case P2P_MN_Time:
        break;
        case P2P_ME_Time:
        break;
        case LRW_SN_Time:
        	m_config->lrwStpNorm = time;
            updateLRWTimer(time);
        break;
        case LRW_SE_Time:
        	m_config->lrwStpEmer = time;
	        updateLRWTimer(time);
        break;
        case LRW_MN_Time:
        break;
        case LRW_ME_Time:
        break;
    }

}


static void p2pTimerCallback(void* arg)
{
    int64_t now = esp_timer_get_time();
    ESP_LOGW(TAG, "----------- P2P TIMEOUT ------------\r\n");
    m_config->lastP2PTick = now;
    uint64_t p2pExpiryTime;
    esp_err_t ret = esp_timer_get_period(p2pTimer, &p2pExpiryTime);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Get P2P TIMER failed %d", ret);
    }
    m_config->P2PAlarm = now + p2pExpiryTime;
    xTaskNotify(xTaskToNotify, BIT_P2P_REQ_TX, eSetBits);
}

static void lrwTimerCallback(void* arg)
{
    int64_t now = esp_timer_get_time();
    ESP_LOGW(TAG, "------------ LRWTIMEOUT ------------\r\n");
    m_config->lastLRWTick = now;
    uint64_t lrwExpiryTime;
    esp_err_t ret = esp_timer_get_period(lrwTimer, &lrwExpiryTime);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Get LRW TIMER failed %d", ret);
    }
    m_config->LRWAlarm = now + lrwExpiryTime;
    xTaskNotify(xTaskToNotify, BIT_LRW_REQ_TX, eSetBits);
}

static void app_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == APP_EVENT && event_id == APP_EVENT_P2P_RX)
    {
        LoRaElementP2PRx_t *p2pRx_p = (LoRaElementP2PRx_t*) event_data;
        memcpy(&p2pRx, p2pRx_p, sizeof(LoRaElementP2PRx_t));
        xQueueSend(xQueueP2PRx, &p2pRx, 0);
        xTaskNotify(xTaskToNotify, BIT_P2P_RX, eSetBits);
    }

    if (event_base == APP_EVENT && event_id == APP_EVENT_LRW_RX)
    {
        LoRaElementLRWRx_t *lrwRX_p = (LoRaElementLRWRx_t*) event_data;
        memcpy(&lrwRx, lrwRX_p, sizeof(LoRaElementLRWRx_t));
        xQueueSend(xQueueLRWRx, &lrwRx, 0);
        xTaskNotify(xTaskToNotify, BIT_LRW_RX, eSetBits);
    }

    if(event_base == APP_EVENT && event_id == APP_EVENT_P2P_TX_DONE)
    {
        LoRaElementP2P_t *p2pTxDone_p = (LoRaElementP2P_t*) event_data;
        memcpy(&p2pTxDone, p2pTxDone_p, sizeof(LoRaElementP2P_t));
        xTaskNotify(xTaskToNotify, BIT_P2P_TX_DONE, eSetBits);
    }

    if(event_base == APP_EVENT && event_id == APP_EVENT_LRW_TX_DONE)
    {
        LoRaElementLRWTxDone_t *lrwTxDone_p = (LoRaElementLRWTxDone_t*) event_data;
        memcpy(&lrwTxDone, lrwTxDone_p, sizeof(LoRaElementLRWTxDone_t));
        xTaskNotify(xTaskToNotify, BIT_LRW_TX_DONE, eSetBits);
    }

    if(event_base == APP_EVENT && event_id == APP_EVENT_REQ_P2P_SEND)
    {
        uint64_t now = esp_timer_get_time();
        if(now - m_config->lastP2PTick > 10000000 )
        {
            m_config->lastP2PTick = now;
            xTaskNotify(xTaskToNotify, BIT_P2P_REQ_TX, eSetBits);
        }
    }

    if(event_base == APP_EVENT && event_id == APP_EVENT_REQ_LRW_SEND)
    {
        uint64_t now = esp_timer_get_time();
        if(now - m_config->lastLRWTick > 10000000 )
        {
            m_config->lastP2PTick = now;
            xTaskNotify(xTaskToNotify, BIT_LRW_REQ_TX, eSetBits);
        }
    }

}

void lorawanRX(LoRaElementLRWRx_t* _lrwRx);

void stateTask (void* pvParameters)
{
    xTaskToNotify = xTaskGetCurrentTaskHandle();
    
    m_config = (Isca_t *)pvParameters;
    
    xQueueP2PRx = xQueueCreate(QUEUE_P2P_RX_SIZE, sizeof(LoRaElementP2PRx_t));
    xQueueLRWRx = xQueueCreate(QUEUE_LRW_RX_SIZE, sizeof(LoRaElementLRWRx_t));

    esp_event_handler_instance_register(APP_EVENT, ESP_EVENT_ANY_ID, &app_event_handler, nullptr, nullptr);
    
    const esp_timer_create_args_t p2pTimerArgs = {
            .callback = &p2pTimerCallback,
            .name = "p2pTimer"
    };

    const esp_timer_create_args_t lrwTimerArgs = {
            .callback = &lrwTimerCallback,
            .name = "lrwTimer"
    };

    ESP_ERROR_CHECK(esp_timer_create(&p2pTimerArgs, &p2pTimer));
    ESP_ERROR_CHECK(esp_timer_create(&lrwTimerArgs, &lrwTimer));
        
    m_config->p2pStpEmer = 0x1E;
    m_config->p2pStpNorm = 0x3C;
    m_config->lrwStpEmer = 0x1E;
    m_config->lrwStpNorm = 0x3C;
    
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
                            (uxQueueSpacesAvailable(xQueueP2PRx) == QUEUE_P2P_RX_SIZE) )
                {
                    xTaskNotifyWait( pdFALSE, pdFALSE, &ulNotifiedValue, xMaxBlockTime );
                }

                int64_t now = esp_timer_get_time();

                ESP_LOGI(TAG, "flags: %02lX | Timeout P2P: %03llds | Timeout LRW: %03llds", 
                ulNotifiedValue,((m_config->P2PAlarm - now))/1000000, ((m_config->LRWAlarm - now))/1000000);

                if(uxQueueSpacesAvailable(xQueueP2PRx) != QUEUE_P2P_RX_SIZE)
                {
                    state = SM_RCV_P2P;
                } 
                else if(uxQueueSpacesAvailable(xQueueLRWRx) != QUEUE_LRW_RX_SIZE)
                {
                    state = SM_RCV_LRW;
                }
                else if( ( ulNotifiedValue & BIT_P2P_REQ_TX ) != 0 )
                {
                    state = SM_SEND_P2P;
                    ulTaskNotifyValueClear( xTaskToNotify, BIT_P2P_REQ_TX );
                    ulNotifiedValue &= ~BIT_P2P_REQ_TX;
                } 
                else if( ( ulNotifiedValue & BIT_P2P_TX_DONE ) != 0 )
                {
                    state = SM_SEND_P2P_DONE;
                    ulTaskNotifyValueClear( xTaskToNotify, BIT_P2P_TX_DONE );
                    ulNotifiedValue &= ~BIT_P2P_TX_DONE;
                }
                else if( ( ulNotifiedValue & BIT_P2P_RX ) != 0 )
                {
                    state = SM_RCV_P2P;
                    ulTaskNotifyValueClear( xTaskToNotify, BIT_P2P_RX );
                    ulNotifiedValue &= ~BIT_P2P_RX;
                } 
                else if( ( ulNotifiedValue & BIT_LRW_REQ_TX ) != 0 )
                {
                    state = SM_SEND_LRW;
                    ulTaskNotifyValueClear( xTaskToNotify, BIT_LRW_REQ_TX );
                    ulNotifiedValue &= ~BIT_LRW_REQ_TX;
                }
                else if( ( ulNotifiedValue & BIT_LRW_TX_DONE ) != 0 )
                {
                    state = SM_SEND_LRW_DONE;
                    ulTaskNotifyValueClear( xTaskToNotify, BIT_LRW_TX_DONE );
                    ulNotifiedValue &= ~BIT_LRW_TX_DONE;
                }
                else if( ( ulNotifiedValue & BIT_LRW_RX ) != 0 )
                {
                    state = SM_RCV_LRW;
                    ulTaskNotifyValueClear( xTaskToNotify, BIT_LRW_RX );
                    ulNotifiedValue &= ~BIT_LRW_RX;
                }

            }
            break;
            
            case SM_SEND_P2P:
            {
                static uint8_t counter = 0;
                PositionP2PUnion_t pos;
                memset(&pos, 0, sizeof(PositionP2PUnion_t));

                float P2PBattery = (float)(m_config->batteryMiliVolts / 20.0);

                pos.param.loraId[0] = (uint8_t) m_config->loraId & 0xFFFFFF;
                pos.param.loraId[1] = (uint8_t) (m_config->loraId >> 8) & 0xFFFF;
                pos.param.loraId[2] = (uint8_t) (m_config->loraId >> 16) & 0xFF;
                pos.param.packetType = 80;
                pos.param.flags.headingGps = 511;
                pos.param.flags.batteryVoltageInfos = 2;

                pos.param.batteryVoltage = (uint8_t)(P2PBattery < 0 ? (P2PBattery - 0.5) : (P2PBattery + 0.5));;
                pos.param.flags.accelerometerStatus = 0;
                pos.param.flags.criticalBatteryStatus = m_config->flags.asBit.lowBattery;
                pos.param.flags.powerSupplyStatus = 0;
                pos.param.flags.emergencyStatus = m_config->flags.asBit.emergency;
                pos.param.header.sequenceNumber = counter;

                if (counter++ > 63)
                    counter = 0;

                pos.array[5] = dallas_crc8((const uint8_t*) (pos.array),
                    sizeof(PositionP2P_t));
                
                memset(&p2pTx.payload.buffer, 0, sizeof(p2pTx.payload.buffer));
                memcpy(&p2pTx.payload.buffer, pos.array, sizeof(PositionP2P_t));
                p2pTx.payload.size = sizeof(PositionP2P_t);
                
                p2pTx.params.txFreq = m_config->p2pTXFreq;
                p2pTx.params.txPower = m_config->p2pTxPower;
                p2pTx.params.BW = m_config->p2pBW;
                p2pTx.params.SF = m_config->p2pSF;
                p2pTx.params.CR = m_config->p2pCR;
                p2pTx.params.rxFreq = m_config->p2pRXFreq;
                p2pTx.params.rxDelay = m_config->p2pRXDelay;
                p2pTx.params.rxTimeout = m_config->p2pRXTimeout;
               
                esp_event_post(APP_EVENT, APP_EVENT_QUEUE_P2P_SEND, (void*)&p2pTx, sizeof(LoRaElementP2P_t), 0);

                state = SM_WAIT_FOR_EVENT;
            }

            break;

            case SM_SEND_P2P_DONE:
            {
                //static LoRaElementP2P_t p2pTx, p2pTxDone;
                PositionP2PUnion_t pos;
                memcpy(&pos, p2pTxDone.payload.buffer, sizeof(PositionP2PUnion_t));
                printf("[LORA] PARSE P2P TX Counter: %02d | ", pos.param.header.sequenceNumber);
                printf("LoraID: %.2X%.2X%.2X | ", pos.param.loraId[0], pos.param.loraId[1], pos.param.loraId[2]);
                printf("Batt: %.2X | ", pos.param.batteryVoltage);
                printf("Accelerometer: %s | ", (pos.param.flags.accelerometerStatus) ? "ON" : "OFF");
                printf("criticalBatteryStatus: %s | ", (pos.param.flags.criticalBatteryStatus) ? "ON" : "OFF");
                printf("powerSupplyStatus: %s | ", (pos.param.flags.powerSupplyStatus) ? "ON" : "OFF");
                printf("Emergency: %s\r\n", (pos.param.flags.emergencyStatus) ? "ON" : "OFF");
                state = SM_WAIT_FOR_EVENT;
            }
            break;

            case SM_RCV_P2P:
            {
                LoRaElementP2PRx_t _rx;
                if(xQueueReceive(xQueueP2PRx, &_rx, 10) == pdPASS)
                {
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

                            if (idLora == m_config->loraId)
                            {
                                printf("[LORA] Message for me from %02X%02X%02X | rssi: %d | srn: %d | ",
                                                            commandReceived.param.loraIdGw[2],
                                                            commandReceived.param.loraIdGw[1],
                                                            commandReceived.param.loraIdGw[0],
                                                            _rx.params.rssi, _rx.params.snr);

                                if (commandReceived.param.loraEmergencyCommand)
                                {
                                    enterEmergency();
                                }
                                else
                                {
                                    exitEmergency();
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

            case SM_SEND_LRW:
            {                
                lrwTx.params.port = m_config->lrwPosPort;
                lrwTx.params.confirmed = m_config->lrwConfirmed;
               
                memset(&lrwTx.payload.buffer, 0, sizeof(lrwTx.payload.buffer));
                lrwTx.payload.buffer[0] = m_config->lrwProtocol;
                lrwTx.payload.buffer[1] = (m_config->loraId >> 16) & 0xFF;
                lrwTx.payload.buffer[2] = (m_config->loraId >> 8) & 0xFF;
                lrwTx.payload.buffer[3] = m_config->loraId & 0xFF;
                lrwTx.payload.buffer[4] = m_config->temperatureCelsius;
                lrwTx.payload.buffer[5] = (uint8_t)((m_config->batteryMiliVolts & 0xFF00)>>8);
                lrwTx.payload.buffer[6] = (uint8_t)(m_config->batteryMiliVolts & 0x00FF);
                lrwTx.payload.buffer[7] = m_config->flags.asArray[0];
                lrwTx.payload.buffer[8] = m_config->flags.asArray[1];
                lrwTx.payload.size = 9;               
                
                esp_event_post(APP_EVENT, APP_EVENT_QUEUE_LRW_SEND, &lrwTx, sizeof(LoRaElementLRWTx_t), 0);

                state = SM_WAIT_FOR_EVENT;
            }
            break;

            case SM_SEND_LRW_DONE:
            {
                //static LoRaElementLRWTxDone_t lrwTxDone;
                printf("[LORA] LRW TX Uplink: %ld | channel: %d | length: %d \r\n", lrwTxDone.done.upLinkCounter, lrwTxDone.done.channel, lrwTxDone.done.length);
				printf("[LORA] PARSE TX LRW Protocol Version: %02X | ", lrwTxDone.payload.buffer[0]);
				printf("LoRaID: %02X %02X %02X | ", lrwTxDone.payload.buffer[1], lrwTxDone.payload.buffer[2], lrwTxDone.payload.buffer[3]);
				printf("Temp: 0x%02X = %d | ", lrwTxDone.payload.buffer[4], lrwTxDone.payload.buffer[4]);
				printf("Battery: %d mV | ", (int)((lrwTxDone.payload.buffer[5]<<8) + lrwTxDone.payload.buffer[6]));
				printf("Flags: %02X %02X\r\n", lrwTxDone.payload.buffer[7], lrwTxDone.payload.buffer[8]);
                
                state = SM_WAIT_FOR_EVENT;
            }
            break;

            case SM_RCV_LRW:
                LoRaElementLRWRx_t _rx;
                if(xQueueReceive(xQueueLRWRx, &_rx, 10) == pdPASS)
                {
                    lorawanRX(&_rx);
                }
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
        //vTaskDelay(1000);
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

void lorawanRX(LoRaElementLRWRx_t* _lrwRx)
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