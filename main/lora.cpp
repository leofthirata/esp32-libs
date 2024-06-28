#include "lora.hpp"
#include <LoRaWan-Arduino.h>
#include <SPI.h>
#include "loraEvents.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "stateMachine.hpp"

hw_config hwConfig;
QueueHandle_t xQueueLoRa;

loraEvents_t p2p, lrw;
static LoRa_SM_t state = LORA_SM_WAIT_FOR_SEND, state_prev = LORA_SM_WAIT_FOR_SEND;
static TaskHandle_t xTaskToNotify = NULL;

static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];			  ///< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0}; ///< Lora user application data structure.
static lorawanTXParams_t lorawanTXParams;

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


static const char *TAG = "LoRaTask";

static const LoRa_Queue_t p2pElement = QUEUE_SEND_P2P;
static const LoRa_Queue_t lrwElement = QUEUE_SEND_LRW;

static loraLRWRXParam_t _lrwRX;
static loraLRWTXParam_t _lrwTX;
static loraP2PTXParam_t _p2pTX;
static loraP2PRXParam_t _p2pRX;

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

const char* printLoRaStateMachineState(LoRa_SM_t _status)
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


void p2pTXDone()
{
	xTaskNotify(xTaskToNotify, LORA_BIT_P2P_TX_DONE, eSetBits);
}

void p2pTXTimeout(timeoutType_t type)
{
	xTaskNotify(xTaskToNotify, LORA_BIT_P2P_TX_TIMEOUT, eSetBits);
}

void p2pRXDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
	printf("	[Pre notify] RX DONE\r\n");
	memset(&_p2pRX, 0, sizeof(loraP2PRXParam_t));

	memcpy(&_p2pRX.buffer, payload, size );
	_p2pRX.rssi = rssi;
	_p2pRX.size = size;
	_p2pRX.snr = snr;
	esp_event_post(APP_EVENT, APP_EVENT_P2P_RX, (void*)&_p2pRX, sizeof(loraP2PRXParam_t), 0);
	xTaskNotify(xTaskToNotify, LORA_BIT_P2P_RX_DONE, eSetBits);
}

void p2pRXTimeout(timeoutType_t type)
{
	printf("	[PRE NOTIFY] RX TIMEOUT\r\n");
	xTaskNotify(xTaskToNotify, LORA_BIT_P2P_RX_TIMEOUT, eSetBits);
}

void p2pRXError()
{
	xTaskNotify(xTaskToNotify, LORA_BIT_P2P_RX_ERROR, eSetBits);
}

void lrwTXDone()
{
	xTaskNotify(xTaskToNotify, LORA_BIT_LRW_TX_DONE, eSetBits);
}

void lrwTXTimeout(timeoutType_t type)
{
	xTaskNotify(xTaskToNotify, LORA_BIT_LRW_TX_TIMEOUT, eSetBits);
}

void lrwRXDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
	xTaskNotify(xTaskToNotify, LORA_BIT_LRW_RX_DONE, eSetBits);
}

void lrwRXTimeout(timeoutType_t type)
{
	xTaskNotify(xTaskToNotify, LORA_BIT_LRW_RX_TIMEOUT, eSetBits);
}

void lrwRXError()
{
	xTaskNotify(xTaskToNotify, LORA_BIT_LRW_RX_ERROR, eSetBits);
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
	Serial.println("OVER_THE_AIR_ACTIVATION != 0");
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
	Serial.printf("[LORA] LRW downlink received port:%d, size:%d, rssi:%d, snr:%d\n",
				  app_data->port, app_data->buffsize, app_data->rssi, app_data->snr);
	for (uint8_t i = 0; i < app_data->buffsize; i++)
		printf(" %02X ", *(app_data->buffer + i));
	printf("}\r\n");

	uint16_t size = 0;

	if( (app_data->buffsize) > LORA_MAX_PAYLOAD )
		size = LORA_MAX_PAYLOAD;
	else
		size = app_data->buffsize;

	memcpy(_lrwRX.buffer, app_data->buffer, size);
	
	_lrwRX.size = size;
	_lrwRX.port = app_data->port;
	_lrwRX.rssi = app_data->rssi;
	_lrwRX.snr = app_data->snr;


	esp_event_post(APP_EVENT, APP_EVENT_LRW_RX, (void*)&_lrwRX, sizeof(loraLRWRXParam_t), 0);

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

	case LRW_CMD_PORT:
	{


	}
		break;

	default:
		break;
	}
}

static void app_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    // Isca_t *m_config = (Isca_t *) event_data;
    if (event_base == APP_EVENT && event_id == APP_EVENT_QUEUE_P2P_SEND)
    {
        loraP2PTXParam_t *p2pTX_p = (loraP2PTXParam_t*) event_data;

		memset(&_p2pTX, 0, sizeof(loraP2PTXParam_t));
		memcpy(&_p2pTX, p2pTX_p, sizeof(loraP2PTXParam_t));
		xQueueSend(xQueueLoRa, &p2pElement, 0);
    }
    if (event_base == APP_EVENT && event_id == APP_EVENT_QUEUE_LRW_SEND)
    {
		loraLRWTXParam_t *lrwTX_p = (loraLRWTXParam_t*) event_data;

		memset(&_lrwTX, 0, sizeof(loraLRWTXParam_t));
		memcpy(&_lrwTX, lrwTX_p, sizeof(loraLRWTXParam_t));
		xQueueSend(xQueueLoRa, &lrwElement, 0);
    }

}

void loraTask(void* param)
{
	
	Isca_t *config = (Isca_t*) param;
	LoRa_Queue_t typeSendLoRa = QUEUE_NONE;
    
	xQueueLoRa = xQueueCreate( 10, sizeof( LoRa_Queue_t ) );
	xTaskToNotify = xTaskGetCurrentTaskHandle();
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
    
	esp_event_handler_instance_register(APP_EVENT, ESP_EVENT_ANY_ID, &app_event_handler, nullptr, nullptr);

	// Initialize LoRa chip.
	uint32_t err_code = lora_hardware_init(hwConfig);
	if (err_code != 0)
	{
		Serial.printf("lora_hardware_init failed - %ld\n", err_code);
	}

	// Setup the EUIs and Keys
	lmh_setDevEui(config->nodeDeviceEUI);
	lmh_setAppEui(config->nodeAppEUI);
	lmh_setAppKey(config->nodeAppKey);
	lmh_setNwkSKey(config->nodeNwsKey);
	lmh_setAppSKey(config->nodeAppsKey);
	lmh_setDevAddr(config->nodeDevAddr);

	p2p.TxDone = p2pTXDone;
	p2p.TxTimeout = p2pTXTimeout;
	p2p.RxDone = p2pRXDone;
	p2p.RxTimeout = p2pRXTimeout;
	p2p.RxError = p2pRXError;

	setP2PEvents(&p2p);

	lrw.TxDone = lrwTXDone;
	lrw.TxTimeout = lrwTXTimeout;
	lrw.RxDone = lrwRXDone;
	lrw.RxTimeout = lrwRXTimeout;
	lrw.RxError = lrwRXError;

	setLRWEvents(&lrw);

	// Initialize LoRaWan
	err_code = lmh_init(&lora_callbacks, lora_param_init, false, CLASS_A, LORAMAC_REGION_AU915);
	if (err_code != 0)
	{
		Serial.printf("lmh_init failed - %ld\n", err_code);
	}

	// Start Join procedure
	lmh_join();
	uint32_t ulNotifiedValue = 0;
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 10000 );
    while(1)
    {
        ESP_LOGI(TAG, "%s", printLoRaStateMachineState(state));
		switch(state)
		{
			case LORA_SM_WAIT_FOR_SEND:
				
				printf("\r\n");
				if( xQueueReceive( xQueueLoRa, &( typeSendLoRa ), portMAX_DELAY ) == pdPASS )
				{
					if(typeSendLoRa == QUEUE_SEND_P2P)
					{
						state = LORA_SM_P2P_TX;
					}
					else if (typeSendLoRa == QUEUE_SEND_LRW)
					{
						state = LORA_SM_LRW_TX;
					}
				}
				state_prev = LORA_SM_WAIT_FOR_SEND;
				break;

			case LORA_SM_WAIT_FOR_TIMEOUT:
			{
				state = LORA_SM_WAIT_FOR_SEND;
                if(ulNotifiedValue == 0)
				{
					xTaskNotifyWait( pdFALSE, pdFALSE, &ulNotifiedValue, xMaxBlockTime );
				}
				ESP_LOGW(TAG, "Task Notify: %03lX", ulNotifiedValue);

				if( ( ulNotifiedValue & LORA_BIT_P2P_TX_DONE ) != 0 )
				{
					state = LORA_SM_P2P_TX_DONE;
					ulTaskNotifyValueClear( xTaskToNotify, LORA_BIT_P2P_TX_DONE );
                    ulNotifiedValue &= ~LORA_BIT_P2P_TX_DONE;
				} 
				else if((ulNotifiedValue & LORA_BIT_P2P_TX_TIMEOUT) != 0 )
				{
					state = LORA_SM_P2P_TX_TIMEOUT;
					ulTaskNotifyValueClear( xTaskToNotify, LORA_BIT_P2P_TX_TIMEOUT );
                    ulNotifiedValue &= ~LORA_BIT_P2P_TX_TIMEOUT;
				} 
				else if((ulNotifiedValue & LORA_BIT_P2P_RX_TIMEOUT) != 0 )
				{
					state = LORA_SM_P2P_RX_TIMEOUT;
					ulTaskNotifyValueClear( xTaskToNotify, LORA_BIT_P2P_RX_TIMEOUT );
                    ulNotifiedValue &= ~LORA_BIT_P2P_RX_TIMEOUT;
				} 
				else if((ulNotifiedValue & LORA_BIT_P2P_RX_DONE) != 0 )
				{
					state = LORA_SM_P2P_RX_DONE;
					ulTaskNotifyValueClear( xTaskToNotify, LORA_BIT_P2P_RX_DONE );
                    ulNotifiedValue &= ~LORA_BIT_P2P_RX_DONE;
				} 
				else if((ulNotifiedValue & LORA_BIT_P2P_RX_ERROR) != 0 )
				{
					state = LORA_SM_P2P_RX_ERROR;
					ulTaskNotifyValueClear( xTaskToNotify, LORA_BIT_P2P_RX_ERROR );
                    ulNotifiedValue &= ~LORA_BIT_P2P_RX_ERROR;
				} 
				else if((ulNotifiedValue & LORA_BIT_LRW_TX_DONE) != 0 )
				{
					state = LORA_SM_LRW_TX_DONE;
					ulTaskNotifyValueClear( xTaskToNotify, LORA_BIT_LRW_TX_DONE );
                    ulNotifiedValue &= ~LORA_BIT_LRW_TX_DONE;
				} 
				else if((ulNotifiedValue & LORA_BIT_LRW_TX_TIMEOUT) != 0 )
				{
					state = LORA_SM_LRW_TX_TIMEOUT;
					ulTaskNotifyValueClear( xTaskToNotify, LORA_BIT_LRW_TX_TIMEOUT );
                    ulNotifiedValue &= ~LORA_BIT_LRW_TX_TIMEOUT;
				} 
				else if((ulNotifiedValue & LORA_BIT_LRW_RX_DONE) != 0 )
				{	
					state = LORA_SM_LRW_RX_DONE;
					ulTaskNotifyValueClear( xTaskToNotify, LORA_BIT_LRW_RX_DONE );
                    ulNotifiedValue &= ~LORA_BIT_LRW_RX_DONE;					
				} 
				else if((ulNotifiedValue & LORA_BIT_LRW_RX_TIMEOUT) != 0 )
				{
					state = LORA_SM_LRW_RX_TIMEOUT;
					ulTaskNotifyValueClear( xTaskToNotify, LORA_BIT_LRW_RX_TIMEOUT );
                    ulNotifiedValue &= ~LORA_BIT_LRW_RX_TIMEOUT;					
				} 
				else if((ulNotifiedValue & LORA_BIT_LRW_RX_ERROR) != 0 )
				{
					state = LORA_SM_LRW_RX_ERROR;
					ulTaskNotifyValueClear( xTaskToNotify, LORA_BIT_LRW_RX_ERROR );
                    ulNotifiedValue &= ~LORA_BIT_LRW_RX_ERROR;					
				}

				state_prev = LORA_SM_WAIT_FOR_TIMEOUT;
				break;
			}

			case LORA_SM_P2P_TX:
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
				Radio.SetChannel(_p2pTX.freq);
				Radio.SetTxConfig(MODEM_LORA, _p2pTX.txPower, 0, _p2pTX.BW,
								_p2pTX.SF, _p2pTX.CR,
								LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
								true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);
				
				Radio.Send(_p2pTX.buffer, _p2pTX.size);
				char payload[256] = {'0'};
				for(int i = 0; i < _p2pTX.size; i++)
				{
					sprintf(payload + strlen(payload), "%02X", _p2pTX.buffer[i]);
				}

				ESP_LOGW(TAG, "P2P SENT: %s", payload);

				state_prev = LORA_SM_P2P_TX;
				state = LORA_SM_WAIT_FOR_TIMEOUT;
				break;
			}

			case LORA_SM_P2P_TX_DONE:
				
				if(_p2pTX.delayRX)
					vTaskDelay(pdMS_TO_TICKS(_p2pTX.delayRX));

				Radio.SetChannel(_p2pTX.freqRX);
				Radio.SetRxConfig(MODEM_LORA, _p2pTX.BW, _p2pTX.SF,
				 _p2pTX.CR, 0, LORA_PREAMBLE_LENGTH,
				LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0, true, 0, 0,
				LORA_IQ_INVERSION_ON, true);
				Radio.Rx(_p2pTX.timeoutRX);

				state_prev = LORA_SM_P2P_TX_DONE;
				state = LORA_SM_WAIT_FOR_TIMEOUT;
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
				m_lora_app_data.port = _lrwTX.port;
				memcpy(m_lora_app_data_buffer, _lrwTX.buffer, _lrwTX.size);
				m_lora_app_data.buffsize = _lrwTX.size;

				lmh_error_status error = lmh_send(&m_lora_app_data, (lmh_confirm) _lrwTX.confirmed);
				
				if (error == LMH_SUCCESS)
				{	
					char payload[256] = {'0'};
					for(int i = 0; i < m_lora_app_data.buffsize; i++)
					{
						sprintf(payload + strlen(payload), "%02X", m_lora_app_data.buffer[i]);
					}

					ESP_LOGW(TAG, "LRW SENT: %s", payload);
				}

				memset(m_lora_app_data_buffer, 0, sizeof(m_lora_app_data_buffer));
			
				state_prev = LORA_SM_LRW_TX;
				state = LORA_SM_WAIT_FOR_TIMEOUT;
				break;
			}

			case LORA_SM_LRW_TX_DONE:
				
				ESP_LOGW(TAG, "UpLinkCounter: %ld | Channel: %d | Lenght: %d", 
					lorawanTXParams.UpLinkCounter, lorawanTXParams.channel, lorawanTXParams.PktLen);
				
				state_prev = LORA_SM_LRW_TX_DONE;
				state = LORA_SM_WAIT_FOR_TIMEOUT;
				break;
			
			case LORA_SM_P2P_RX_TIMEOUT:
				state_prev = LORA_SM_P2P_RX_TIMEOUT;
				state = LORA_SM_WAIT_FOR_SEND;

			break;

			case LORA_SM_LRW_RX_TIMEOUT:
			{
				static uint8_t count = 0;
				if(count == 1)
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