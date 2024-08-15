#include "lora.hpp"
#include <LoRaWan-Arduino.h>
#include <SPI.h>
#include "loraEvents.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "stateMachine.hpp"

hw_config hwConfig;
QueueHandle_t xQueueLoRa, xQueueSendP2P, xQueueSendLRW;

static loraEvents_t _events;
static LoRaSM_t state = LORA_SM_WAIT_FOR_SEND, state_prev = LORA_SM_WAIT_FOR_SEND;
static TaskHandle_t xTaskToNotify = NULL;

static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];			  ///< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0}; ///< Lora user application data structure.
static lorawanTXParams_t lorawanTXParams; //variable to get lrw tx params

static void lorawan_has_joined_handler(void);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void lorawan_join_failed_handler(void);
static void lorawanRX(lmh_app_data_t *app_data);

/**@brief Structure containing LoRaWan parameters, needed for lmh_init() */
static lmh_param_t lora_param_init = {LORAWAN_ADR_ON, DR_2, LORAWAN_PUBLIC_NETWORK, JOINREQ_NBTRIALS, LORAWAN_DEFAULT_TX_POWER, LORAWAN_DUTYCYCLE_OFF, &lorawanTXParams};

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init() */
static lmh_callback_t lora_callbacks = {BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
										lorawanRX, lorawan_has_joined_handler,
										lorawan_confirm_class_handler, lorawan_join_failed_handler};


static const char *TAG = "loraTask";


static LoRaLRWRx_t _lrwRx;
static LoRaP2PRx_t _p2pRx;

static LoRaQueueElement_t p2pQueueElement = {.type = QUEUE_SEND_P2P};
static LoRaQueueElement_t lrwQueueElement = {.type = QUEUE_SEND_LRW};

#define LORA_BIT_P2P_TX_DONE		0x0001
#define LORA_BIT_P2P_TX_TIMEOUT		0x0002
#define LORA_BIT_P2P_RX_DONE		0x0004
#define LORA_BIT_P2P_RX_TIMEOUT		0x0008
#define LORA_BIT_P2P_RX_ERROR		0x0010
#define LORA_BIT_LRW_TX_DONE		0x0020
#define LORA_BIT_LRW_TX_TIMEOUT		0x0040
#define LORA_BIT_LRW_RX_DONE		0x0080
#define LORA_BIT_LRW_RX_TIMEOUT		0x0100
#define LORA_BIT_LRW_RX_ERROR		0x0200

const char* printLoRaStateMachineState(LoRaSM_t _status)
{
	switch(_status)
	{
		case LORA_SM_WAIT_FOR_SEND:
			return "wait for send";
    	case LORA_SM_WAIT_FOR_TIMEOUT:
			return "waiting for timeout";
		case LORA_SM_P2P_TX:
			return "P2P TX";
		case LORA_SM_P2P_TX_DONE:
			return "P2P TX done";
		case LORA_SM_P2P_TX_TIMEOUT:
			return "P2P TX timeout";
		case LORA_SM_P2P_RX_DONE:
			return "P2P RX done";
		case LORA_SM_P2P_RX_TIMEOUT:
			return "P2P RX timeout";
		case LORA_SM_P2P_RX_ERROR:
			return "P2P RX error";
		case LORA_SM_LRW_TX:
			return "LRW TX";
		case LORA_SM_LRW_TX_DONE:
			return "LRW TX_done";
		case LORA_SM_LRW_TX_TIMEOUT:
			return "LRW TX timeout";
		case LORA_SM_LRW_STATUS_TX:
			return "LRW STATUS TX";
		case LORA_SM_LRW_RX_DONE:
			return "LRW RX done";
		case LORA_SM_LRW_RX_TIMEOUT:
			return "LRW RX timeout";
		case LORA_SM_LRW_RX_ERROR:
			return "LRW RX error";
		default:
			return "UNKNOW STATUS";
	}
}

void _eventsTXDone(bool isLRW)
{
	if(isLRW)
	{
		xTaskNotify(xTaskToNotify, LORA_BIT_LRW_TX_DONE, eSetBits);
	}
	else
	{
		xTaskNotify(xTaskToNotify, LORA_BIT_P2P_TX_DONE, eSetBits);
	}
}

void _eventsTXTimeout(bool isLRW, timeoutType_t type)
{
	if(isLRW)
	{
		xTaskNotify(xTaskToNotify, LORA_BIT_LRW_TX_TIMEOUT, eSetBits);
	}
	else
	{
		xTaskNotify(xTaskToNotify, LORA_BIT_P2P_TX_TIMEOUT, eSetBits);
	}
}

void _eventsRXDone(bool isLRW, uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
	if(isLRW)
	{
		xTaskNotify(xTaskToNotify, LORA_BIT_LRW_RX_DONE, eSetBits);
	}
	else
	{
		//copy payload to LoRaElement_t structure
		memcpy(&_p2pRx.payload.buffer, payload, size );
		_p2pRx.payload.size = size;
		_p2pRx.params.rssi = rssi;
		_p2pRx.params.snr = snr;

		//post event
		esp_event_post(APP_EVENT, APP_EVENT_P2P_RX, (void*)&_p2pRx, sizeof(LoRaP2PRx_t), 0);
		
		//notify task
		xTaskNotify(xTaskToNotify, LORA_BIT_P2P_RX_DONE, eSetBits);
	}
}

void _eventsRXTimeout(bool isLRW, timeoutType_t type)
{
	if(isLRW)
	{
		xTaskNotify(xTaskToNotify, LORA_BIT_LRW_RX_TIMEOUT, eSetBits);
	}
	else
	{
		xTaskNotify(xTaskToNotify, LORA_BIT_P2P_RX_TIMEOUT, eSetBits);
	}
}

void _eventsRXError(bool isLRW)
{
	if(isLRW)
	{
		xTaskNotify(xTaskToNotify, LORA_BIT_LRW_RX_ERROR, eSetBits);
	}
	else
	{
		xTaskNotify(xTaskToNotify, LORA_BIT_P2P_RX_ERROR, eSetBits);
	}
}

static void lorawan_join_failed_handler(void)
{
	Serial.println("OVER_THE_AIR_ACTIVATION failed!");
	Serial.println("Check your EUI's and Keys's!");
	Serial.println("Check if a Gateway is in range!");
}

static void lorawan_has_joined_handler(void)
{
#if (OVER_THE_AIR_ACTIVATION != 0)
	Serial.println("Network Joined");
#else
	//Serial.println("OVER_THE_AIR_ACTIVATION != 0");
#endif
	lmh_class_request(CLASS_A);
}

static void lorawan_confirm_class_handler(DeviceClass_t Class)
{
	Serial.printf("switch to class %c done\n", "ABC"[Class]);

	// Informs the server that switch has occurred ASAP
	m_lora_app_data.buffsize = 0;
	m_lora_app_data.port = LORAWAN_APP_PORT;
	lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG);
}

static void lorawanRX(lmh_app_data_t *app_data)
{
	uint16_t size = 0;

	if( (app_data->buffsize) > LORA_MAX_PAYLOAD )
		size = LORA_MAX_PAYLOAD;
	else
		size = app_data->buffsize;

	memcpy(_lrwRx.payload.buffer, app_data->buffer, size);
	_lrwRx.payload.size = size;
	_lrwRx.params.port= app_data->port;
	_lrwRx.params.rssi = app_data->rssi;
	_lrwRx.params.snr = app_data->snr;

	esp_event_post(APP_EVENT, APP_EVENT_LRW_RX, (void*)&_lrwRx, sizeof(LoRaLRWRx_t), 0);

	switch (app_data->port)
	{
	case 3:
		// Port 3 switches the class
		if (app_data->buffsize == 1)
		{
			switch (app_data->buffer[0])
			{
			case 0:
				lmh_class_request(CLASS_A);
				break;

			case 1:
				lmh_class_request(CLASS_B);
				break;

			case 2:
				lmh_class_request(CLASS_C);
				break;

			default:
				break;
			}
		}
		break;

	default:
		break;
	}
}

static void app_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{

    if (event_base == APP_EVENT && event_id == APP_EVENT_P2P_TX_REQ)
    {
		LoRaP2PReq_t *element_p = (LoRaP2PReq_t*) event_data;
		LoRaP2PReq_t _p2pTx;

		memcpy(&_p2pTx, element_p, sizeof(LoRaP2PReq_t));
		xQueueSend(xQueueSendP2P, &_p2pTx, 0);
		xQueueSend(xQueueLoRa, &p2pQueueElement, 0);
    }
    if (event_base == APP_EVENT && event_id == APP_EVENT_LRW_TX_REQ)
    {
		LoRaLRWTxReq_t *element_p = (LoRaLRWTxReq_t*) event_data;
		LoRaLRWTxReq_t _lrwTx;
		memcpy(&_lrwTx, element_p, sizeof(LoRaLRWTxReq_t));
		xQueueSend(xQueueSendLRW, &_lrwTx, 0);
		xQueueSend(xQueueLoRa, &lrwQueueElement, 0);
    }

}

void loraTask(void* param)
{	
	Isca_t *config = (Isca_t*) param;
	LoRaQueueElement_t loraQueueElement2Send = {.type = QUEUE_NONE};
    
	uint32_t ulNotifiedValue = 0;
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 10000 );
	LoRaP2PReq_t p2pElement;
	LoRaLRWTxReq_t lrwElement;

	xQueueLoRa = xQueueCreate( LORA_TX_QUEUE_SIZE, sizeof( LoRaQueueElement_t ) );
	xQueueSendP2P = xQueueCreate(5, sizeof(LoRaP2PReq_t));
	xQueueSendLRW = xQueueCreate(5, sizeof(LoRaLRWTxReq_t));
	
	// xTaskToNotify = xTaskGetCurrentTaskHandle();

	// Define the HW configuration between MCU and SX126x
	hwConfig.CHIP_TYPE = SX1262_CHIP;		  // Example uses an eByte E22 module with an SX1262
	hwConfig.PIN_LORA_RESET = PIN_NUM_LORA_RESET; // LORA RESET
	hwConfig.PIN_LORA_NSS = PIN_NUM_LORA_NSS;	  // LORA SPI CS
	hwConfig.PIN_LORA_SCLK = PIN_NUM_LORA_SCLK;	  // LORA SPI CLK
	hwConfig.PIN_LORA_MISO = PIN_NUM_LORA_MISO;	  // LORA SPI MISO
	hwConfig.PIN_LORA_DIO_1 = PIN_NUM_LORA_DIO_1; // LORA DIO_1
	hwConfig.PIN_LORA_BUSY = PIN_NUM_LORA_BUSY;	  // LORA SPI BUSY
	hwConfig.PIN_LORA_MOSI = PIN_NUM_LORA_MOSI;	  // LORA SPI MOSI
	hwConfig.RADIO_TXEN = PIN_NUM_RADIO_TXEN;		  // LORA ANTENNA TX ENABLE
	hwConfig.RADIO_RXEN = PIN_NUM_RADIO_RXEN;		  // LORA ANTENNA RX ENABLE
	hwConfig.USE_DIO2_ANT_SWITCH = true;	  // Example uses an CircuitRocks Alora RFM1262 which uses DIO2 pins as antenna control
	hwConfig.USE_DIO3_TCXO = false;			  // Example uses an CircuitRocks Alora RFM1262 which uses DIO3 to control oscillator voltage
	hwConfig.USE_DIO3_ANT_SWITCH = false;	  // Only Insight ISP4520 module uses DIO3 as antenna control
    
	esp_event_handler_instance_register(APP_EVENT, APP_EVENT_P2P_TX_REQ,
									 &app_event_handler, nullptr, nullptr);
	esp_event_handler_instance_register(APP_EVENT, APP_EVENT_LRW_TX_REQ,
									 &app_event_handler, nullptr, nullptr);
									  
	// Initialize LoRa chip.
	uint32_t err_code = lora_hardware_init(hwConfig);
	if (err_code != 0)
	{
		Serial.printf("lora_hardware_init failed - %ld\n", err_code);
	}

	// Setup the EUIs and Keys
	lmh_setDevEui(config->rom.deviceEUI);
	lmh_setAppEui(config->rom.appEUI);
	lmh_setAppKey(config->rom.appKey);
	lmh_setNwkSKey(config->rom.nwkSKey);
	lmh_setAppSKey(config->rom.appSKey);
	lmh_setDevAddr(config->rom.devAddr);

	// _events.TxDone = _eventsTXDone;
	// _events.TxTimeout = _eventsTXTimeout;
	// _events.RxDone = _eventsRXDone;
	// _events.RxTimeout = _eventsRXTimeout;
	// _events.RxError = _eventsRXError;

	// setLoRaEvents(&_events);

	// Initialize LoRaWan
	err_code = lmh_init(&lora_callbacks, lora_param_init, false, CLASS_A, LORAMAC_REGION_AU915);
	if (err_code != 0)
	{
		Serial.printf("lmh_init failed - %ld\n", err_code);
	}

	// Start Join procedure
	lmh_join();

	while(1)
    {
        ESP_LOGI(TAG, "%s", printLoRaStateMachineState(state));
		switch(state)
		{
			case LORA_SM_WAIT_FOR_SEND:
				
				printf("\r\n");
				if( xQueueReceive( xQueueLoRa, &( loraQueueElement2Send ), portMAX_DELAY ) == pdPASS )
				{
					if(loraQueueElement2Send.type == QUEUE_SEND_P2P)
					{
						state = LORA_SM_P2P_TX;
					}
					else if (loraQueueElement2Send.type == QUEUE_SEND_LRW)
					{
						state = LORA_SM_LRW_TX;
					}
				}
				state_prev = LORA_SM_WAIT_FOR_SEND;
				break;

			case LORA_SM_WAIT_FOR_TIMEOUT:
			{
				state = LORA_SM_WAIT_FOR_SEND;
                // if(ulNotifiedValue == 0)
				// {
				// 	xTaskNotifyWait( pdFALSE, pdFALSE, &ulNotifiedValue, xMaxBlockTime );
				// }
				ESP_LOGI(TAG, "flags: %03lX | free xQueueTxLoRa = %d/%d", ulNotifiedValue, uxQueueSpacesAvailable(xQueueLoRa),LORA_TX_QUEUE_SIZE);

				if( ( ulNotifiedValue & LORA_BIT_P2P_TX_DONE ) != 0 )
				{
					state = LORA_SM_P2P_TX_DONE;
					//ulTaskNotifyValueClear( xTaskToNotify, LORA_BIT_P2P_TX_DONE );
                    ulNotifiedValue &= ~LORA_BIT_P2P_TX_DONE;
				} 
				else if((ulNotifiedValue & LORA_BIT_P2P_TX_TIMEOUT) != 0 )
				{
					state = LORA_SM_P2P_TX_TIMEOUT;
					//ulTaskNotifyValueClear( xTaskToNotify, LORA_BIT_P2P_TX_TIMEOUT );
                    ulNotifiedValue &= ~LORA_BIT_P2P_TX_TIMEOUT;
				} 
				else if((ulNotifiedValue & LORA_BIT_P2P_RX_TIMEOUT) != 0 )
				{
					state = LORA_SM_P2P_RX_TIMEOUT;
					//ulTaskNotifyValueClear( xTaskToNotify, LORA_BIT_P2P_RX_TIMEOUT );
                    ulNotifiedValue &= ~LORA_BIT_P2P_RX_TIMEOUT;
				} 
				else if((ulNotifiedValue & LORA_BIT_P2P_RX_DONE) != 0 )
				{
					state = LORA_SM_P2P_RX_DONE;
					//ulTaskNotifyValueClear( xTaskToNotify, LORA_BIT_P2P_RX_DONE );
                    ulNotifiedValue &= ~LORA_BIT_P2P_RX_DONE;
				} 
				else if((ulNotifiedValue & LORA_BIT_P2P_RX_ERROR) != 0 )
				{
					state = LORA_SM_P2P_RX_ERROR;
					//ulTaskNotifyValueClear( xTaskToNotify, LORA_BIT_P2P_RX_ERROR );
                    ulNotifiedValue &= ~LORA_BIT_P2P_RX_ERROR;
				} 
				else if((ulNotifiedValue & LORA_BIT_LRW_TX_DONE) != 0 )
				{
					state = LORA_SM_LRW_TX_DONE;
					//ulTaskNotifyValueClear( xTaskToNotify, LORA_BIT_LRW_TX_DONE );
                    ulNotifiedValue &= ~LORA_BIT_LRW_TX_DONE;
				} 
				else if((ulNotifiedValue & LORA_BIT_LRW_TX_TIMEOUT) != 0 )
				{
					state = LORA_SM_LRW_TX_TIMEOUT;
					//ulTaskNotifyValueClear( xTaskToNotify, LORA_BIT_LRW_TX_TIMEOUT );
                    ulNotifiedValue &= ~LORA_BIT_LRW_TX_TIMEOUT;
				} 
				else if((ulNotifiedValue & LORA_BIT_LRW_RX_DONE) != 0 )
				{	
					state = LORA_SM_LRW_RX_DONE;
					//ulTaskNotifyValueClear( xTaskToNotify, LORA_BIT_LRW_RX_DONE );
                    ulNotifiedValue &= ~LORA_BIT_LRW_RX_DONE;					
				} 
				else if((ulNotifiedValue & LORA_BIT_LRW_RX_TIMEOUT) != 0 )
				{
					state = LORA_SM_LRW_RX_TIMEOUT;
					//ulTaskNotifyValueClear( xTaskToNotify, LORA_BIT_LRW_RX_TIMEOUT );
                    ulNotifiedValue &= ~LORA_BIT_LRW_RX_TIMEOUT;					
				} 
				else if((ulNotifiedValue & LORA_BIT_LRW_RX_ERROR) != 0 )
				{
					state = LORA_SM_LRW_RX_ERROR;
					//ulTaskNotifyValueClear( xTaskToNotify, LORA_BIT_LRW_RX_ERROR );
                    ulNotifiedValue &= ~LORA_BIT_LRW_RX_ERROR;					
				}

				state_prev = LORA_SM_WAIT_FOR_TIMEOUT;
				break;
			}

			case LORA_SM_P2P_TX:
			{
				if(xQueueReceive(xQueueSendP2P, &p2pElement, pdMS_TO_TICKS(10)) == pdPASS)
				{
					if(Radio.GetStatus() == RF_RX_RUNNING)
					{
						ESP_LOGI(TAG,"RX Running");
						Radio.Standby();
					
						switch(Radio.GetStatus())
						{
							case RF_RX_RUNNING:
							ESP_LOGI(TAG,"RF_RX_RUNNING");
							break;

							case RF_IDLE:
							ESP_LOGI(TAG,"RF_IDLE");
							break;
							
							case RF_TX_RUNNING:
							ESP_LOGI(TAG,"RF_TX_RUNNING");
							break;
							
							case RF_CAD:
							ESP_LOGI(TAG,"RF_CAD");
							break;

						}
					}

					MibRequestConfirm_t mibReq;
					mibReq.Type = MIB_PUBLIC_NETWORK;
					mibReq.Param.EnablePublicNetwork = false;
					LoRaMacMibSetRequestConfirm(&mibReq);
					
					// Set Radio TX configuration
					Radio.SetChannel(p2pElement.params.txFreq);
					Radio.SetTxConfig(MODEM_LORA, p2pElement.params.txPower, 0, p2pElement.params.BW,
									p2pElement.params.SF, p2pElement.params.CR,
									LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
									true, 0, 0, LORA_IQ_INVERSION_ON, p2pElement.params.txTimeout);
					
					Radio.Send(p2pElement.payload.buffer, p2pElement.payload.size);
					
					char payload[256];
					for(int i = 0; i < p2pElement.payload.size; i++)
					{
						snprintf(&payload[i*3], 256, "%02X ", p2pElement.payload.buffer[i]);
					}
					payload[p2pElement.payload.size * 3] = 0x00;
					ESP_LOGI(TAG, "P2P SENT: %s", payload);

					state = LORA_SM_WAIT_FOR_TIMEOUT;
				}
				else
				{
					state = LORA_SM_WAIT_FOR_SEND;
				}

				state_prev = LORA_SM_P2P_TX;
				break;
			}

			case LORA_SM_P2P_TX_DONE:
				{
					esp_event_post(APP_EVENT, APP_EVENT_P2P_TX_RES, (void*)&p2pElement, sizeof(LoRaP2PReq_t), 0);
					
					if(p2pElement.params.rxDelay)
						vTaskDelay(pdMS_TO_TICKS(p2pElement.params.rxDelay));

					Radio.SetChannel(p2pElement.params.rxFreq);
					Radio.SetRxConfig(MODEM_LORA, p2pElement.params.BW, p2pElement.params.SF,
					p2pElement.params.CR, 0, LORA_PREAMBLE_LENGTH,
					LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0, true, 0, 0,
					LORA_IQ_INVERSION_ON, true);
					Radio.Rx(p2pElement.params.rxTimeout);

					state_prev = LORA_SM_P2P_TX_DONE;
					state = LORA_SM_WAIT_FOR_TIMEOUT;
				}
				break;


			case LORA_SM_P2P_RX_DONE:
			{
				// esp_event_post(APP_EVENT, APP_EVENT_P2P_RX, (void*)&_p2pRX, sizeof(loraP2PRXParam_t), 0);
				
				state_prev = LORA_SM_P2P_RX_DONE;
				state = LORA_SM_WAIT_FOR_TIMEOUT;
				break;
			}


			case LORA_SM_LRW_TX:
			{
				if(xQueueReceive(xQueueSendLRW, &lrwElement, pdMS_TO_TICKS(10)) == pdPASS)
				{
					MibRequestConfirm_t mibReq;
					mibReq.Type = MIB_PUBLIC_NETWORK;
					mibReq.Param.EnablePublicNetwork = true;
					LoRaMacMibSetRequestConfirm(&mibReq);
					
					if (lmh_join_status_get() != LMH_SET)
					{
						//Not joined, try again later
						ESP_LOGE(TAG, "Did not join network, skip sending frame");
						return;
					}
					
					memset(m_lora_app_data_buffer, 0, sizeof(m_lora_app_data_buffer));
					
					memcpy(m_lora_app_data_buffer, lrwElement.payload.buffer, lrwElement.payload.size);
					m_lora_app_data.buffsize = lrwElement.payload.size;
					m_lora_app_data.port = lrwElement.params.port;

					lmh_error_status error = lmh_send(&m_lora_app_data, (lmh_confirm) lrwElement.params.confirmed);
					
					if (error == LMH_SUCCESS)
					{	
						char payload[256];
						for(int i = 0; i < m_lora_app_data.buffsize; i++)
						{
							snprintf(&payload[i*3], 256, "%02X ", m_lora_app_data.buffer[i]);
						}
						payload[m_lora_app_data.buffsize * 3] = 0x00;
						ESP_LOGI(TAG, "LRW SENT: %s", payload);
					}

					memset(m_lora_app_data_buffer, 0, sizeof(m_lora_app_data_buffer));
				
					state = LORA_SM_WAIT_FOR_TIMEOUT;
				}
				else
				{
					state = LORA_SM_WAIT_FOR_SEND;
				}
				state_prev = LORA_SM_LRW_TX;
				break;
			}

			case LORA_SM_LRW_TX_DONE:
			{
				LoRaLRWTxRes_t elementTxDone;

				memcpy(&elementTxDone.payload, &lrwElement.payload, sizeof(lrwElement.payload));
				memcpy(&elementTxDone.params, &lrwElement.params, sizeof(lrwElement.params));
				elementTxDone.done.channel = lorawanTXParams.channel;
				elementTxDone.done.length = lorawanTXParams.PktLen;
				elementTxDone.done.upLinkCounter = lorawanTXParams.UpLinkCounter;

				esp_event_post(APP_EVENT, APP_EVENT_LRW_TX_RES, (void*)&elementTxDone, sizeof(LoRaLRWTxRes_t), 0);

				ESP_LOGI(TAG, "UpLinkCounter: %ld | Channel: %d | Lenght: %d", 
					lorawanTXParams.UpLinkCounter, lorawanTXParams.channel, lorawanTXParams.PktLen);
				
				state_prev = LORA_SM_LRW_TX_DONE;
				state = LORA_SM_WAIT_FOR_TIMEOUT;
			}
				break;
			
			case LORA_SM_P2P_RX_TIMEOUT:
				state_prev = LORA_SM_P2P_RX_TIMEOUT;
				state = LORA_SM_WAIT_FOR_SEND;

			break;

			case LORA_SM_LRW_RX_TIMEOUT:
			{
				static uint8_t count = 0;
				if(count >= 1)
				{
					count = 0;
					state = LORA_SM_WAIT_FOR_SEND;
				} 
				else
				{
					count++;
					state = LORA_SM_WAIT_FOR_TIMEOUT;
				}
			}
				state_prev = state;
				break;
			case LORA_SM_P2P_TX_TIMEOUT:
			case LORA_SM_P2P_RX_ERROR:
			case LORA_SM_LRW_TX_TIMEOUT:
			case LORA_SM_LRW_STATUS_TX:
			case LORA_SM_LRW_RX_DONE:
			case LORA_SM_LRW_RX_ERROR:

				state_prev = state;
				state = LORA_SM_WAIT_FOR_SEND;
				break;

		}
    }
}