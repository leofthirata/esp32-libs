#include "stateMachine.hpp"
#include "esp_timer.h"
#include "lora.hpp"

static Isca_t *m_config;
static bool resetRequested = false;
static const char *TAG = "StateTask";
static Isca_SM_t state;
static TaskHandle_t xTaskToNotify = NULL;

static void p2pTimerCallback(void* arg);
static void lrwTimerCallback(void* arg);
esp_timer_handle_t p2pTimer;
esp_timer_handle_t lrwTimer;
static loraLRWRXParam_t lrwRX;
static loraP2PTXParam_t p2pTX;
static loraLRWTXParam_t lrwTX;
static loraP2PRXParam_t p2pRX;

/** @brief IP event base declaration */
ESP_EVENT_DECLARE_BASE(APP_EVENT);

#define BIT_LRW_TX  0x01
#define BIT_LRW_RX  0x02
#define BIT_P2P_TX  0x04
#define BIT_P2P_RX  0x08


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
    xTaskNotify(xTaskToNotify, BIT_P2P_TX, eSetBits);
}

static void lrwTimerCallback(void* arg)
{
    int64_t now = esp_timer_get_time();
    ESP_LOGW(TAG, "------------ LRWTIMEOUT ------------\r\n");
    m_config->lastLRWTick = now;
    uint64_t lrwExpiryTime;
    esp_err_t ret = esp_timer_get_period(p2pTimer, &lrwExpiryTime);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Get P2P TIMER failed %d", ret);
    }
    m_config->LRWAlarm = now + lrwExpiryTime;
    xTaskNotify(xTaskToNotify, BIT_LRW_TX, eSetBits);
}

static void app_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == APP_EVENT && event_id == APP_EVENT_P2P_RX)
    {
        // printf("P2P RCV Event\r\n");
        static loraP2PRXParam_t *p2pRX_p = (loraP2PRXParam_t*) event_data;
        memcpy(&lrwRX, p2pRX_p, sizeof(loraP2PRXParam_t));
        xTaskNotify(xTaskToNotify, BIT_P2P_RX, eSetBits);
    }
    if (event_base == APP_EVENT && event_id == APP_EVENT_LRW_RX)
    {
        // printf("LRW RCV Event\r\n");
        static loraLRWRXParam_t *lrwRX_p = (loraLRWRXParam_t*) event_data;
        memcpy(&lrwRX, lrwRX_p, sizeof(loraLRWRXParam_t));
        xTaskNotify(xTaskToNotify, BIT_LRW_RX, eSetBits);

    }
    if(event_base == APP_EVENT && event_id == APP_EVENT_REQ_P2P_SEND)
    {
        uint64_t now = esp_timer_get_time();
        if(now - m_config->lastP2PTick > 10000000 )
        {
            xTaskNotify(xTaskToNotify, BIT_P2P_TX, eSetBits);
            m_config->lastP2PTick = now;
        }
    }
    if(event_base == APP_EVENT && event_id == APP_EVENT_REQ_LRW_SEND)
    {
        uint64_t now = esp_timer_get_time();
        if(now - m_config->lastLRWTick > 10000000 )
        {
            xTaskNotify(xTaskToNotify, BIT_LRW_TX, eSetBits);
            m_config->lastP2PTick = now;
        }
    }

}

void lorawanRX();


void stateTask (void* pvParameters)
{
    xTaskToNotify = xTaskGetCurrentTaskHandle();
    m_config = (Isca_t *)pvParameters;

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
    
    state = SM_UPDATE_TIMERS;
    uint64_t p2pExpiryTime = 0, lrwExpiryTime = 0;
    uint32_t ulNotifiedValue = 0;
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 10000 );

    esp_event_handler_instance_register(APP_EVENT, ESP_EVENT_ANY_ID, &app_event_handler, nullptr, nullptr);

    while(1)
    {
        switch(state)
        {
            case SM_WAIT_FOR_EVENT:
            {             
                if(ulNotifiedValue == 0)
                {
                    xTaskNotifyWait( pdFALSE, pdFALSE, &ulNotifiedValue, xMaxBlockTime );
                }

                int64_t now = esp_timer_get_time();

                ESP_LOGI(TAG, "TASK Notify: %02lX | Timeout P2P: %03llds | Timeout LRW: %03llds", ulNotifiedValue,
                        // ((p2pExpiryTime))/1000000, ((lrwExpiryTime))/1000000);
                        ((m_config->P2PAlarm - now))/1000000, ((m_config->LRWAlarm - now))/1000000);

                if( ( ulNotifiedValue & BIT_P2P_TX ) != 0 )
                {
                    state = SM_SEND_P2P;
                    ulTaskNotifyValueClear( xTaskToNotify, BIT_P2P_TX );
                    ulNotifiedValue &= ~BIT_P2P_TX;
                    
                } 
                else if( ( ulNotifiedValue & BIT_P2P_RX ) != 0 )
                {
                    state = SM_RCV_P2P;
                    ulTaskNotifyValueClear( xTaskToNotify, BIT_P2P_RX );
                    ulNotifiedValue &= ~BIT_P2P_RX;

                } 
                else if( ( ulNotifiedValue & BIT_LRW_TX ) != 0 )
                {
                    state = SM_SEND_LRW;
                    ulTaskNotifyValueClear( xTaskToNotify, BIT_LRW_TX );
                    ulNotifiedValue &= ~BIT_LRW_TX;

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

                // atribui novo valor ao byte do crc
                pos.array[5] = dallas_crc8((const uint8_t*) (pos.array),
                    sizeof(PositionP2P_t));
                
                memset(&p2pTX, 0, sizeof(loraP2PTXParam_t));
                memcpy(&p2pTX.buffer, pos.array, sizeof(PositionP2P_t));
                p2pTX.BW = m_config->p2pBW;
                p2pTX.CR = m_config->p2pCR;
                p2pTX.freq = m_config->p2pTXFreq;
                p2pTX.SF = m_config->p2pSF;
                p2pTX.size = sizeof(PositionP2P_t);
                p2pTX.txPower = m_config->p2pTxPower;

                p2pTX.timeoutRX = m_config->p2pRXTimeout;
                p2pTX.freqRX = m_config->p2pRXFreq;
                p2pTX.delayRX = m_config->p2pRXDelay;
                esp_event_post(APP_EVENT, APP_EVENT_QUEUE_P2P_SEND, (void*)&p2pTX, sizeof(loraP2PTXParam_t), 0);

                state = SM_WAIT_FOR_EVENT;
            }

            break;

            case SM_RCV_P2P:

                //printf("[LORA] rssi:%d | size:%d\r\n",  p2pRX.rssi, p2pRX.size);
                
                if(p2pRX.size == sizeof(CommandP2P_t))
                {
                    CommandP2PUnion_t commandReceived;
                    memcpy(commandReceived.array, p2pRX.buffer, p2pRX.size);

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
                                                        p2pRX.rssi, p2pRX.snr);

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

                state = SM_WAIT_FOR_EVENT;
            break;

            case SM_SEND_LRW:
                memset(&lrwTX, 0, sizeof(lrwTX));
                lrwTX.port = m_config->lrwCmdPort;
                lrwTX.confirmed = m_config->lrwConfirmed;
                lrwTX.buffer[0] = m_config->lrwProtocol;
                lrwTX.buffer[1] = (m_config->loraId >> 16) & 0xFF;
                lrwTX.buffer[2] = (m_config->loraId >> 8) & 0xFF;
                lrwTX.buffer[3] = m_config->loraId & 0xFF;
                lrwTX.buffer[4] = m_config->temperatureCelsius;
                lrwTX.buffer[5] = (uint8_t)((m_config->batteryMiliVolts & 0xFF00)>>8);
                lrwTX.buffer[6] = (uint8_t)(m_config->batteryMiliVolts & 0x00FF);
                lrwTX.buffer[7] = m_config->flags.asArray[0];
                lrwTX.buffer[8] = m_config->flags.asArray[1];
                lrwTX.size = 9;               
                
                esp_event_post(APP_EVENT, APP_EVENT_QUEUE_LRW_SEND, &lrwTX, sizeof(lrwTX), 0);
                // queueLRW();

				//Serial.printf("lmh_send result %d\n", error);
				// printf("[LORA] PARSE TX LRW Protocol Version: %02X | ", lrwTX.buffer[0]);
				// printf("LoRaID: %02X %02X %02X | ", lrwTX.buffer[1], lrwTX.buffer[2], lrwTX.buffer[3]);
				// printf("Temp: 0x%02X = %d | ", lrwTX.buffer[4], lrwTX.buffer[4]);
				// printf("Battery: 0x%04X = %d mV | ", m_config->batteryMiliVolts, m_config->batteryMiliVolts);
				// printf("Flags: %02X %02X\r\n", lrwTX.buffer[7], lrwTX.buffer[8]);
                state = SM_WAIT_FOR_EVENT;
            break;
            
            case SM_RCV_LRW:
                printf("[LORA] port:%d | rssi:%d | size:%d\r\n", lrwRX.port, lrwRX.rssi, lrwRX.size);

                lorawanRX();
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

void lorawanRX()
{
    if(lrwRX.port == LRW_CMD_PORT)
    {
        uint8_t pktCounter = 0;
        bool error = false;
        if(lrwRX.buffer[0] == CMD_LRW_HEADER)
        {
            pktCounter = 1;
            while(pktCounter < (lrwRX.size-1) && error == false)
            {
                switch(lrwRX.buffer[pktCounter])
                {
                case EMERGENCY:
                    if(lrwRX.size - (pktCounter+1) >= CMD_PARAM_SIZE_EMER)
                    {

                        if(lrwRX.buffer[pktCounter + 1])
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
                    if(lrwRX.size - (pktCounter+1) >= CMD_PARAM_SIZE_BLE)
                    {
                        if(lrwRX.buffer[pktCounter + 1])
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
                    if(lrwRX.size - (pktCounter+1) >= CMD_PARAM_SIZE_STOCK)
                    {
                        if(lrwRX.buffer[pktCounter + 1])
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
                    if(lrwRX.size - (pktCounter+1) >= CMD_PARAM_SIZE_OUTPUT)
                    {
                        if(lrwRX.buffer[pktCounter + 1])
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
                    if(lrwRX.size - (pktCounter+1) >= CMD_PARAM_SIZE_RESET)
                    {
                        if(lrwRX.buffer[pktCounter + 1])
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
                    if(lrwRX.size - (pktCounter+1) >= CMD_PARAM_SIZE_BLE_POWER)
                    {
                        if(lrwRX.buffer[pktCounter + 1] >= 0 &&  lrwRX.buffer[pktCounter + 1] <= 31)
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
                    if(lrwRX.size - (pktCounter+1) >= CMD_PARAM_SIZE_BLE_TIME)
                    {
                        if(lrwRX.buffer[pktCounter + 1] >= 0 &&  lrwRX.buffer[pktCounter + 1] <= 31)
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
                    if(lrwRX.buffer[pktCounter + 1] >= CMD_PARAM_SIZE_ALL_TIMES)
                    {
                        uint32_t LoRaTimesArray[8] = {};
                        uint32_t *p_LoRaTimesArray = &LoRaTimesArray[0];

                        for(int i = 0; i < 20; i+=5)
                        {
                            *p_LoRaTimesArray |= lrwRX.buffer[pktCounter + i + 1] << 12;
                            *p_LoRaTimesArray |= lrwRX.buffer[pktCounter + i + 2] << 4;
                            *p_LoRaTimesArray |= (lrwRX.buffer[pktCounter + i + 3] & 0xF0) >> 4;

                            p_LoRaTimesArray++;

                            *p_LoRaTimesArray |= (lrwRX.buffer[pktCounter + i + 3] & 0x0F) << 16;
                            *p_LoRaTimesArray |= lrwRX.buffer[pktCounter + i + 4] << 8;
                            *p_LoRaTimesArray |= (lrwRX.buffer[pktCounter + i + 5]);

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
                    if(lrwRX.size - (pktCounter+1) >= CMD_PARAM_SIZE_GET_STATUS)
                    {
                        if(lrwRX.buffer[pktCounter + 1])
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
                    if(lrwRX.size - (pktCounter+1) >= CMD_PARAM_SIZE_LED)
                    {
                        if(lrwRX.buffer[pktCounter + 1])
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
                    if(lrwRX.size - (pktCounter+1) >= TIME_CMD_PARAM_SIZE)
                    {
                        uint32_t dummy = 0;
                        dummy = (lrwRX.buffer[pktCounter + 1] & 0x0F) << 16;
                        dummy |= lrwRX.buffer[pktCounter + 2]  << 8;
                        dummy |= lrwRX.buffer[pktCounter + 3];
                        printf("[LORA] Requested Change to %s:%ld\r\n", getCMDString((CommandLRWDict_t)lrwRX.buffer[pktCounter]),dummy);
                        printf("WARNING! Feature not implemented\r\n");
                        pktCounter+= TIME_CMD_PARAM_SIZE;
                    }
                    else
                    {
                        printf("[LORA] Warning! There's no %s parameter\r\n",  getCMDString((CommandLRWDict_t)lrwRX.buffer[pktCounter]));
                        error = 1;
                    }
                    break;

                case P2P_STP_NOR:
                    if(lrwRX.size - (pktCounter+1) >= TIME_CMD_PARAM_SIZE)
                    {
                        uint32_t dummy = 0;
                        dummy = (lrwRX.buffer[pktCounter + 1] & 0x0F) << 16;
                        dummy |= lrwRX.buffer[pktCounter + 2]  << 8;
                        dummy |= lrwRX.buffer[pktCounter + 3];
                        printf("[LORA] Changed P2P SN to %ld\r\n", dummy);
                        changeP2P_SN_Time(dummy);
                        pktCounter+= TIME_CMD_PARAM_SIZE;
                    }
                    else
                    {
                        printf("[LORA] Warning! There's no P2P_SN Param\r\n");
                        error = 1;
                    }
                    break;
                case P2P_STP_EMER:
                    if(lrwRX.size - (pktCounter+1) >= TIME_CMD_PARAM_SIZE)
                    {
                        uint32_t dummy = 0;
                        dummy = (lrwRX.buffer[pktCounter + 1] & 0x0F) << 16;
                        dummy |= lrwRX.buffer[pktCounter + 2]  << 8;
                        dummy |= lrwRX.buffer[pktCounter + 3];
                        printf("[LORA] Changed P2P SE to %ld\r\n", dummy);
                        changeP2P_SE_Time(dummy);
                        pktCounter+= TIME_CMD_PARAM_SIZE;
                    }
                    else
                    {
                        printf("[LORA] Warning! There's no P2P_SE Param\r\n");
                        error = 1;
                    }
                    break;
                case LRW_STP_NOR:
                    if(lrwRX.size - (pktCounter+1) >= TIME_CMD_PARAM_SIZE)
                    {
                        uint32_t dummy = 0;
                        dummy = (lrwRX.buffer[pktCounter + 1] & 0x0F) << 16;
                        dummy |= lrwRX.buffer[pktCounter + 2]  << 8;
                        dummy |= lrwRX.buffer[pktCounter + 3];
                        printf("[LORA] Changed LRW SN to %ld\r\n", dummy);
                        changeLRW_SN_Time(dummy);
                        pktCounter+= TIME_CMD_PARAM_SIZE;
                    }
                    else
                    {
                        printf("[LORA] Warning! There's no LRW_SN Param\r\n");
                        error = 1;
                    }
                    break;

                case LRW_STP_EMER:
                    if(lrwRX.size - (pktCounter+1) >= TIME_CMD_PARAM_SIZE)
                    {
                        uint32_t dummy = 0;
                        dummy = (lrwRX.buffer[pktCounter + 1] & 0x0F) << 16;
                        dummy |= lrwRX.buffer[pktCounter + 2]  << 8;
                        dummy |= lrwRX.buffer[pktCounter + 3];
                        printf("[LORA] Changed LRW SE to %ld\r\n", dummy);
                        changeLRW_SE_Time(dummy);
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