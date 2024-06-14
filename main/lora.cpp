#include "lora.hpp"
#include <LoRaWan-Arduino.h>
#include <SPI.h>
#include "loraEvents.h"
#include "freertos/queue.h"
#include "freertos/task.h"

uint8_t nodeDeviceEUI[8] = {0x00, 0x95, 0x64, 0x1F, 0xDA, 0x91, 0x19, 0x0B};
uint8_t nodeAppEUI[8] = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x02, 0x01, 0xE1};
uint8_t nodeAppKey[16] = {0x07, 0xC0, 0x82, 0x0C, 0x30, 0xB9, 0x08, 0x70, 0x0C, 0x0F, 0x70, 0x06, 0x00, 0xB0, 0xBE, 0x09};
uint32_t nodeDevAddr = 0x260116F8;
uint8_t nodeNwsKey[16] = {0x7E, 0xAC, 0xE2, 0x55, 0xB8, 0xA5, 0xE2, 0x69, 0x91, 0x51, 0x96, 0x06, 0x47, 0x56, 0x9D, 0x23};
uint8_t nodeAppsKey[16] = {0xFB, 0xAC, 0xB6, 0x47, 0xF3, 0x58, 0x45, 0xC7, 0x50, 0x7D, 0xBF, 0x16, 0x8B, 0xA8, 0xC1, 0x7C};

hw_config hwConfig;
QueueHandle_t xQueueLoRa;

loraEvents_t p2p, lrw;
static LoRa_SM_t state = LORA_SM_WAIT_FOR_EVENT, state_prev = LORA_SM_WAIT_FOR_EVENT;
static RX_Packet_t rxDataP2P;
static TaskHandle_t xTaskToNotify = NULL;

static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];			  ///< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0}; ///< Lora user application data structure.


static void lorawan_has_joined_handler(void);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void lorawan_join_failed_handler(void);
static void lorawanTX(void);
static void lorawanRX(lmh_app_data_t *app_data);

/**@brief Structure containing LoRaWan parameters, needed for lmh_init() */
static lmh_param_t lora_param_init = {LORAWAN_ADR_ON, DR_2, LORAWAN_PUBLIC_NETWORK, JOINREQ_NBTRIALS, LORAWAN_DEFAULT_TX_POWER, LORAWAN_DUTYCYCLE_OFF};

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init() */
static lmh_callback_t lora_callbacks = {BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
										lorawanRX, lorawan_has_joined_handler,
										lorawan_confirm_class_handler, lorawan_join_failed_handler};


static const char *TAG = "LoRa";


void queueLRW()
{
	LoRa_Queue_t lrwElement = QUEUE_SEND_LRW;
	xQueueSend(xQueueLoRa, &lrwElement, 0);
}

void queueP2P()
{
	LoRa_Queue_t p2pElement = QUEUE_SEND_P2P;
	xQueueSend(xQueueLoRa, &p2pElement, 0);
}

void setLoRaMachineState(LoRa_SM_t new_state)
{
	state = new_state;
}

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

const char* printLoRaStateMachineState(LoRa_SM_t _status)
{
	switch(_status)
	{
		case LORA_SM_WAIT_FOR_QUEUE:
			return "LORA_SM_WAIT_FOR_QUEUE";
    	case LORA_SM_WAIT_FOR_EVENT:
			return "LORA_SM_WAIT_FOR_EVENT";
		case LORA_SM_P2P_TX:
			return "LORA_SM_P2P_TX";
		case LORA_SM_P2P_TX_DONE:
			return "LORA_SM_P2P_TX_DONE";
		case LORA_SM_P2P_TX_TIMEOUT:
			return "LORA_SM_P2P_TX_TIMEOUT";
		case LORA_SM_P2P_RX_DONE:
			return "LORA_SM_P2P_RX_DONE";
		case LORA_SM_P2P_RX_TIMEOUT:
			return "LORA_SM_P2P_RX_TIMEOUT";
		case LORA_SM_P2P_RX_ERROR:
			return "LORA_SM_P2P_RX_ERROR";
		case LORA_SM_LRW_TX:
			return "LORA_SM_LRW_TX";
		case LORA_SM_LRW_TX_TIMEOUT:
			return "LORA_SM_LRW_TX_TIMEOUT";
		case LORA_SM_LRW_TX_STATUS:
			return "LORA_SM_LRW_TX_STATUS";
		case LORA_SM_LRW_RX_DONE:
			return "LORA_SM_LRW_RX_DONE";
		case LORA_SM_LRW_RX_TIMEOUT:
			return "LORA_SM_LRW_RX_TIMEOUT";
		case LORA_SM_LRW_RX_ERROR:
			return "LORA_SM_LRW_RX_ERROR";
		default:
			return "UNKNOW STATUS";
	}
}

void p2pTX(Isca_t *config)
{
	if(Radio.GetStatus() == RF_RX_RUNNING)
	{
		Serial.println("RX Running");
		Radio.Standby();
	
		switch(Radio.GetStatus())
		{
			case RF_RX_RUNNING:
			Serial.println("RF_RX_RUNNING");
			break;

			case RF_IDLE:
			Serial.println("RF_IDLE");
			break;
			
			case RF_TX_RUNNING:
			Serial.println("RF_TX_RUNNING");
			break;
			
			case RF_CAD:
			Serial.println("RF_CAD");
			break;

		}
	}

	MibRequestConfirm_t mibReq;
	mibReq.Type = MIB_PUBLIC_NETWORK;
	mibReq.Param.EnablePublicNetwork = false;
	LoRaMacMibSetRequestConfirm(&mibReq);

	static uint8_t counter = 0;
	PositionP2PUnion_t pos;
	memset(&pos, 0, sizeof(PositionP2PUnion_t));

    float P2PBattery = (float)(config->batteryMiliVolts / 20.0);

	pos.param.loraId[0] = (uint8_t) config->loraId & 0xFFFFFF;
	pos.param.loraId[1] = (uint8_t) (config->loraId >> 8) & 0xFFFF;
	pos.param.loraId[2] = (uint8_t) (config->loraId >> 16) & 0xFF;
	pos.param.packetType = 80;
	pos.param.flags.headingGps = 511;
	pos.param.flags.batteryVoltageInfos = 2;

	pos.param.batteryVoltage = (uint8_t)(P2PBattery < 0 ? (P2PBattery - 0.5) : (P2PBattery + 0.5));;
	pos.param.flags.accelerometerStatus = 0;
	pos.param.flags.criticalBatteryStatus = config->flags.asBit.lowBattery;
	pos.param.flags.powerSupplyStatus = 0;
	pos.param.flags.emergencyStatus = config->flags.asBit.emergency;
	pos.param.header.sequenceNumber = counter;

	if (counter++ > 63)
		counter = 0;

	// atribui novo valor ao byte do crc
	pos.array[5] = dallas_crc8((const uint8_t*) (pos.array),
			sizeof(PositionP2P_t));
	
	// Set Radio TX configuration
	Radio.SetChannel(LORA_P2P_FREQUENCY);
	Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
					LORA_SPREADING_FACTOR, LORA_CODINGRATE,
					LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
					true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);

	Radio.Send(pos.array, sizeof(PositionP2P_t));

    printf("[LORA] PARSE P2P TX Counter: %d | ", pos.param.header.sequenceNumber);
    printf("LoraID: %.2X%.2X%.2X | ", pos.param.loraId[0], pos.param.loraId[1], pos.param.loraId[2]);
    printf("Batt: %.2X | ", pos.param.batteryVoltage);
    printf("Accelerometer: %s | ", (pos.param.flags.accelerometerStatus) ? "ON" : "OFF");
    printf("criticalBatteryStatus: %s | ", (pos.param.flags.criticalBatteryStatus) ? "ON" : "OFF");
    printf("powerSupplyStatus: %s | ", (pos.param.flags.powerSupplyStatus) ? "ON" : "OFF");
    printf("Emergency: %s\r\n", (pos.param.flags.emergencyStatus) ? "ON" : "OFF");
}

void p2pRX(Isca_t *config)
{
	printf("[LORA] P2P Received | rssi:%d | snr:%d | payload[%d]:", rxDataP2P.rssi, rxDataP2P.snr, rxDataP2P.size);
	for (int i = 0; i < rxDataP2P.size; i++)
		printf(" %02X ", rxDataP2P.payload[i]);
	printf("\r\n");

	if(rxDataP2P.size == sizeof(CommandP2P_t))
	{
		CommandP2PUnion_t commandReceived;
		memcpy(commandReceived.array, rxDataP2P.payload, rxDataP2P.size);

		uint8_t lCrc = commandReceived.param.crc8;
		commandReceived.param.crc8 = 0;
		uint8_t crcValidation = dallas_crc8(commandReceived.array, sizeof(CommandP2P_t));

		if(crcValidation == lCrc)
		{
			uint32_t idLora = 0;
			idLora += commandReceived.param.loraIdReceiveCommand[0];
			idLora += (commandReceived.param.loraIdReceiveCommand[1] << 8);
			idLora += (commandReceived.param.loraIdReceiveCommand[2] << 16);

			if (idLora == config->loraId)
			{

				printf("[LORA] Message for me from %02X%02X%02X | rssi: %d | srn: %d | ",
											  commandReceived.param.loraIdGw[2],
											  commandReceived.param.loraIdGw[1],
											  commandReceived.param.loraIdGw[0],
											  rxDataP2P.rssi, rxDataP2P.snr);

				if (commandReceived.param.loraEmergencyCommand)
				{
					//enterEmergency();
				}
				else
				{
					//exitEmergency();
				}

			}
			else
			{
				printf("[LORA] No P2P_RX for me =(\r\n");
			}
		}
	}

}
void p2pTXDone()
{
	setLoRaMachineState(LORA_SM_P2P_TX_DONE);
	xTaskNotifyGive(xTaskToNotify);
}

void p2pTXTimeout(timeoutType_t type)
{
	setLoRaMachineState(LORA_SM_P2P_TX_TIMEOUT);
	xTaskNotifyGive(xTaskToNotify);
}

void p2pRXDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
	rxDataP2P.rssi = rssi;
	rxDataP2P.size = size;
	rxDataP2P.snr = snr;
	
	memcpy(rxDataP2P.payload, payload, size);
	setLoRaMachineState(LORA_SM_P2P_RX_DONE);
	xTaskNotifyGive(xTaskToNotify);
}

void p2pRXTimeout(timeoutType_t type)
{
	setLoRaMachineState(LORA_SM_P2P_RX_TIMEOUT);
	xTaskNotifyGive(xTaskToNotify);
}

void p2pRXError()
{
	setLoRaMachineState(LORA_SM_P2P_RX_ERROR);
	xTaskNotifyGive(xTaskToNotify);
}

void lrwTXDone()
{
}

void lrwTXTimeout(timeoutType_t type)
{
	setLoRaMachineState(LORA_SM_LRW_TX_TIMEOUT);
	xTaskNotifyGive(xTaskToNotify);
}

void lrwRXDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
	setLoRaMachineState(LORA_SM_LRW_RX_DONE);
	xTaskNotifyGive(xTaskToNotify);
}

void lrwRXTimeout(timeoutType_t type)
{
	setLoRaMachineState(LORA_SM_LRW_RX_TIMEOUT);
	xTaskNotifyGive(xTaskToNotify);
}

void lrwRXError()
{
	setLoRaMachineState(LORA_SM_LRW_TX_TIMEOUT);
	xTaskNotifyGive(xTaskToNotify);
}

/** @brief LoRa function for handling OTAA join failed */
static void lorawan_join_failed_handler(void)
{
	Serial.println("OVER_THE_AIR_ACTIVATION failed!");
	Serial.println("Check your EUI's and Keys's!");
	Serial.println("Check if a Gateway is in range!");
}

/** @brief LoRa function for handling HasJoined event. */
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

static void lorawanTX(void)
{
	MibRequestConfirm_t mibReq;
	mibReq.Type = MIB_PUBLIC_NETWORK;
	mibReq.Param.EnablePublicNetwork = true;
	LoRaMacMibSetRequestConfirm(&mibReq);
	
	if (lmh_join_status_get() != LMH_SET)
	{
		//Not joined, try again later
		Serial.println("Did not join network, skip sending frame");
		return;
	}

	uint32_t i = 0;
	m_lora_app_data.port = LORAWAN_APP_PORT;
	m_lora_app_data.buffer[i++] = 'H';
	m_lora_app_data.buffer[i++] = 'e';
	m_lora_app_data.buffer[i++] = 'l';
	m_lora_app_data.buffer[i++] = 'l';
	m_lora_app_data.buffer[i++] = 'o';
	m_lora_app_data.buffer[i++] = ' ';
	m_lora_app_data.buffer[i++] = 'w';
	m_lora_app_data.buffer[i++] = 'o';
	m_lora_app_data.buffer[i++] = 'r';
	m_lora_app_data.buffer[i++] = 'l';
	m_lora_app_data.buffer[i++] = 'd';
	m_lora_app_data.buffer[i++] = '!';
	m_lora_app_data.buffsize = i;

	lmh_error_status error = lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG);
	if (error == LMH_SUCCESS)
	{
	}
	Serial.printf("lmh_send result %d\n", error);
}

/**@brief Function for handling LoRaWan received data from Gateway
 *
 * @param[app_data] app_data  Pointer to rx data
 */
static void lorawanRX(lmh_app_data_t *app_data)
{
	Serial.printf("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d\n",
				  app_data->port, app_data->buffsize, app_data->rssi, app_data->snr);

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

	case LORAWAN_APP_PORT:
		// YOUR_JOB: Take action on received data
		break;

	default:
		break;
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
    
    // Initialize LoRa chip.
	uint32_t err_code = lora_hardware_init(hwConfig);
	if (err_code != 0)
	{
		Serial.printf("lora_hardware_init failed - %ld\n", err_code);
	}

	// Setup the EUIs and Keys
	lmh_setDevEui(nodeDeviceEUI);
	lmh_setAppEui(nodeAppEUI);
	lmh_setAppKey(nodeAppKey);
	lmh_setNwkSKey(nodeNwsKey);
	lmh_setAppSKey(nodeAppsKey);
	lmh_setDevAddr(nodeDevAddr);

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

    while(1)
    {
        ESP_LOGI(TAG, "%s", printLoRaStateMachineState(state));
		switch(state)
		{
			case LORA_SM_WAIT_FOR_QUEUE:
				
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
				state_prev = LORA_SM_WAIT_FOR_QUEUE;
				break;

			case LORA_SM_WAIT_FOR_EVENT:
			{
				uint32_t ulNotificationValue = ulTaskNotifyTake( pdTRUE, pdMS_TO_TICKS(10000));
				if( ulNotificationValue != 1)
				{
					/* The call to ulTaskNotifyTake() timed out. */
					state = LORA_SM_WAIT_FOR_QUEUE;
				}
				state_prev = LORA_SM_WAIT_FOR_EVENT;
				break;
			}

			case LORA_SM_P2P_TX:
				
				p2pTX(config);
				
				state_prev = LORA_SM_P2P_TX;
				state = LORA_SM_WAIT_FOR_EVENT;
				break;

			case LORA_SM_P2P_TX_DONE:
				
				Radio.SetChannel(LORA_P2P_CMD_FREQUENCY);
				Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
				LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
				LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0, true, 0, 0,
				LORA_IQ_INVERSION_ON, true);
				Radio.Rx(RX_TIMEOUT_VALUE);

				state_prev = LORA_SM_P2P_TX_DONE;
				state = LORA_SM_WAIT_FOR_EVENT;
				break;

			case LORA_SM_P2P_TX_TIMEOUT:

				state_prev = LORA_SM_P2P_TX_TIMEOUT;
				state = LORA_SM_WAIT_FOR_QUEUE;
				break;

			case LORA_SM_P2P_RX_DONE:
				
				p2pRX(config);
				
				state_prev = LORA_SM_P2P_RX_DONE;
				state = LORA_SM_WAIT_FOR_EVENT;
				break;

			case LORA_SM_P2P_RX_TIMEOUT:

				state_prev = LORA_SM_P2P_RX_TIMEOUT;
				state = LORA_SM_WAIT_FOR_QUEUE;
				break;

			case LORA_SM_P2P_RX_ERROR:

				state_prev = LORA_SM_P2P_RX_ERROR;
				state = LORA_SM_WAIT_FOR_QUEUE;
				break;

			case LORA_SM_LRW_TX:

				lorawanTX();
			
				state_prev = LORA_SM_LRW_TX;
				state = LORA_SM_WAIT_FOR_EVENT;
				break;

			case LORA_SM_LRW_TX_TIMEOUT:

				state_prev = LORA_SM_LRW_TX_TIMEOUT;
				state = LORA_SM_WAIT_FOR_QUEUE;
				break;

			case LORA_SM_LRW_TX_STATUS:

				state_prev = LORA_SM_LRW_TX_STATUS;
				state = LORA_SM_WAIT_FOR_QUEUE; //to do: change accordingly
				break;

			case LORA_SM_LRW_RX_DONE:

				state_prev = LORA_SM_LRW_RX_DONE;
				state = LORA_SM_WAIT_FOR_QUEUE;
				break;
		
			case LORA_SM_LRW_RX_TIMEOUT:

				state_prev = LORA_SM_LRW_RX_TIMEOUT;
				state = LORA_SM_WAIT_FOR_QUEUE;
				break;

			case LORA_SM_LRW_RX_ERROR:

				state_prev = LORA_SM_LRW_RX_ERROR;
				state = LORA_SM_WAIT_FOR_QUEUE;
				break;

		}
    }
}