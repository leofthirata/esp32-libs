#include "lora.hpp"
#include <LoRaWan-Arduino.h>
#include <SPI.h>
#include "loraEvents.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "stateMachine.hpp"

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
static lorawanTXParams_t lorawanTXParams;

static void lorawan_has_joined_handler(void);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void lorawan_join_failed_handler(void);
static void lorawanTX(Isca_t *m_config);
static void lorawanRX(lmh_app_data_t *app_data);

/**@brief Structure containing LoRaWan parameters, needed for lmh_init() */
static lmh_param_t lora_param_init = {LORAWAN_ADR_ON, DR_2, LORAWAN_PUBLIC_NETWORK, JOINREQ_NBTRIALS, LORAWAN_DEFAULT_TX_POWER, LORAWAN_DUTYCYCLE_OFF, &lorawanTXParams};

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init() */
static lmh_callback_t lora_callbacks = {BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
										lorawanRX, lorawan_has_joined_handler,
										lorawan_confirm_class_handler, lorawan_join_failed_handler};


static const char *TAG = "LoRa";

static const LoRa_Queue_t p2pElement = QUEUE_SEND_P2P;
static const LoRa_Queue_t lrwElement = QUEUE_SEND_LRW;
void queueLRW()
{
	xQueueSend(xQueueLoRa, &lrwElement, 0);
}

void queueP2P()
{
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
	ESP_LOGW("LoRa", "UpLinkCounter: %ld | Channel: %d | Lenght: %d", 
		lorawanTXParams.UpLinkCounter, lorawanTXParams.channel, lorawanTXParams.PktLen);
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

static void lorawanTX(Isca_t *m_config)
{
	static uint32_t count = 0;

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
	memset(m_lora_app_data_buffer, 0, sizeof(m_lora_app_data_buffer));
	m_lora_app_data.port = LRW_POS_PORT;
	m_lora_app_data.buffer[0] = m_config->lrwProtocol;
	m_lora_app_data.buffer[1] = (m_config->loraId >> 16) & 0xFF;
	m_lora_app_data.buffer[2] = (m_config->loraId >> 8) & 0xFF;
	m_lora_app_data.buffer[3] = m_config->loraId & 0xFF;
	m_lora_app_data.buffer[4] = m_config->temperatureCelsius;
	m_lora_app_data.buffer[5] = (uint8_t)((m_config->batteryMiliVolts & 0xFF00)>>8);
	m_lora_app_data.buffer[6] = (uint8_t)(m_config->batteryMiliVolts & 0x00FF);
	m_lora_app_data.buffer[7] = m_config->flags.asArray[0];
	m_lora_app_data.buffer[8] = m_config->flags.asArray[1];
	m_lora_app_data.buffsize = 9;

	lmh_error_status error = lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG);
	if (error == LMH_SUCCESS)
	{	
		char payload[256] = {'0'};
		for(int i = 0; i < m_lora_app_data.buffsize; i++)
		{
			sprintf(payload + strlen(payload), "%02X", m_lora_app_data.buffer[i]);
		}

		ESP_LOGW(TAG, "LRW SENT: %s", payload);
	}
	//Serial.printf("lmh_send result %d\n", error);
	printf("[LORA] PARSE TX LRW Protocol Version: %02X | ", m_lora_app_data.buffer[0]);
	printf("LoRaID: %02X %02X %02X | ", m_lora_app_data.buffer[1], m_lora_app_data.buffer[2], m_lora_app_data.buffer[3]);
	printf("Temp: 0x%02X = %d | ", m_lora_app_data.buffer[4], m_lora_app_data.buffer[4]);
	printf("Battery: 0x%04X = %d mV | ", m_config->batteryMiliVolts, m_config->batteryMiliVolts);
	printf("Flags: %02X %02X\r\n", m_lora_app_data.buffer[7], m_lora_app_data.buffer[8]);

	memset(m_lora_app_data_buffer, 0, sizeof(m_lora_app_data_buffer));
}

/**@brief Function for handling LoRaWan received data from Gateway
 *
 * @param[app_data] app_data  Pointer to rx data
 */
static void lorawanRX(lmh_app_data_t *app_data)
{
	Serial.printf("[LORA] LRW downlink received port:%d, size:%d, rssi:%d, snr:%d\n",
				  app_data->port, app_data->buffsize, app_data->rssi, app_data->snr);
	for (uint8_t i = 0; i < app_data->buffsize; i++)
		printf(" %02X ", *(app_data->buffer + i));
	printf("}\r\n");
	
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

		uint8_t pktCounter = 0;
		bool error = false;
		if(*(app_data->buffer) == CMD_LRW_HEADER)
		{
			pktCounter = 1;
			while(pktCounter < (app_data->buffsize-1) && error == false)
			{
				switch(*(app_data->buffer + pktCounter))
				{
				case EMERGENCY:
					if(app_data->buffsize - (pktCounter+1) >= CMD_PARAM_SIZE_EMER)
					{

						if(*(app_data->buffer + pktCounter+1))
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
					if(app_data->buffsize - (pktCounter+1) >= CMD_PARAM_SIZE_BLE)
					{
						if(*(app_data->buffer + pktCounter+1))
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
					if(app_data->buffsize - (pktCounter+1) >= CMD_PARAM_SIZE_STOCK)
					{
						if(*(app_data->buffer + pktCounter+1))
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
					if(app_data->buffsize - (pktCounter+1) >= CMD_PARAM_SIZE_OUTPUT)
					{
						if(*(app_data->buffer + pktCounter+1))
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
					if(app_data->buffsize - (pktCounter+1) >= CMD_PARAM_SIZE_RESET)
					{
						if(*(app_data->buffer + pktCounter+1))
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
					if(app_data->buffsize - (pktCounter+1) >= CMD_PARAM_SIZE_BLE_POWER)
					{
						if(*(app_data->buffer + pktCounter+1) >= 0 &&  *(app_data->buffer + pktCounter+1) <= 31)
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
					if(app_data->buffsize - (pktCounter+1) >= CMD_PARAM_SIZE_BLE_TIME)
					{
						if(*(app_data->buffer + pktCounter+1) >= 0 &&  *(app_data->buffer + pktCounter+1) <= 31)
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
					if(app_data->buffsize - (pktCounter+1) >= CMD_PARAM_SIZE_ALL_TIMES)
					{
						uint32_t LoRaTimesArray[8] = {};
						uint32_t *p_LoRaTimesArray = &LoRaTimesArray[0];

						for(int i = 0; i < 20; i+=5)
						{
							*p_LoRaTimesArray |= *(app_data->buffer + pktCounter + i + 1) << 12;
							*p_LoRaTimesArray |= *(app_data->buffer + pktCounter + i + 2) << 4;
							*p_LoRaTimesArray |= (*(app_data->buffer + pktCounter + i + 3) & 0xF0) >> 4;

							p_LoRaTimesArray++;

							*p_LoRaTimesArray |= (*(app_data->buffer + pktCounter + i + 3) & 0x0F) << 16;
							*p_LoRaTimesArray |= *(app_data->buffer + pktCounter + i + 4) << 8;
							*p_LoRaTimesArray |= (*(app_data->buffer + pktCounter + i + 5));

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
					if(app_data->buffsize - (pktCounter+1) >= CMD_PARAM_SIZE_GET_STATUS)
					{
						if(*(app_data->buffer + pktCounter + 1))
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
					if(app_data->buffsize - (pktCounter+1) >= CMD_PARAM_SIZE_LED)
					{
						if(*(app_data->buffer + pktCounter + 1))
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
					if(app_data->buffsize - (pktCounter+1) >= TIME_CMD_PARAM_SIZE)
					{
						uint32_t dummy = 0;
						dummy = (*(app_data->buffer + pktCounter + 1) & 0x0F) << 16;
						dummy |= *(app_data->buffer + pktCounter + 2)  << 8;
						dummy |= *(app_data->buffer + pktCounter + 3);
						printf("[LORA] Requested Change to %s:%ld\r\n", getCMDString((CommandLRWDict_t)*(app_data->buffer + pktCounter)),dummy);
						printf("WARNING! Feature not implemented\r\n");
						pktCounter+= TIME_CMD_PARAM_SIZE;
					}
					else
					{
						printf("[LORA] Warning! There's no %s parameter\r\n",  getCMDString((CommandLRWDict_t)*(app_data->buffer + pktCounter)));
						error = 1;
					}
					break;

				case P2P_STP_NOR:
					if(app_data->buffsize - (pktCounter+1) >= TIME_CMD_PARAM_SIZE)
					{
						uint32_t dummy = 0;
						dummy = (*(app_data->buffer + pktCounter + 1) & 0x0F) << 16;
						dummy |= *(app_data->buffer + pktCounter + 2)  << 8;
						dummy |= *(app_data->buffer + pktCounter + 3);
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
					if(app_data->buffsize - (pktCounter+1) >= TIME_CMD_PARAM_SIZE)
					{
						uint32_t dummy = 0;
						dummy = (*(app_data->buffer + pktCounter + 1) & 0x0F) << 16;
						dummy |= *(app_data->buffer + pktCounter + 2)  << 8;
						dummy |= *(app_data->buffer + pktCounter + 3);
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
					if(app_data->buffsize - (pktCounter+1) >= TIME_CMD_PARAM_SIZE)
					{
						uint32_t dummy = 0;
						dummy = (*(app_data->buffer + pktCounter + 1)) << 16;
						dummy |= *(app_data->buffer + pktCounter + 2)  << 8;
						dummy |= *(app_data->buffer + pktCounter + 3);
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
					if(app_data->buffsize - (pktCounter+1) >= TIME_CMD_PARAM_SIZE)
					{
						uint32_t dummy = 0;
						dummy = (*(app_data->buffer + pktCounter + 1)) << 16;
						dummy |= *(app_data->buffer + pktCounter + 2)  << 8;
						dummy |= *(app_data->buffer + pktCounter + 3);
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

				lorawanTX(config);
			
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