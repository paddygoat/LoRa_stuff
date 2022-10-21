#include <Arduino.h>
#include <SX126x-Arduino.h>
#include <SPI.h>

hw_config hwConfig;

#ifdef ESP32
// ESP32 - SX126x pin configuration
int PIN_LORA_RESET = 4;  // LORA RESET
int PIN_LORA_DIO_1 = 21; // LORA DIO_1
int PIN_LORA_BUSY = 35;  // LORA SPI BUSY
int PIN_LORA_NSS = 5;	// LORA SPI CS
int PIN_LORA_SCLK = 18;  // LORA SPI CLK
int PIN_LORA_MISO = 19;  // LORA SPI MISO
int PIN_LORA_MOSI = 23;  // LORA SPI MOSI
int RADIO_TXEN = -1;	 // LORA ANTENNA TX ENABLE
int RADIO_RXEN = 27;	 // LORA ANTENNA RX ENABLE
#endif

// Function declarations
void OnTxDone(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnTxTimeout(void);
void OnRxTimeout(void);
void OnRxError(void);
void OnCadDone(bool cadResult);

// Check if the board has an LED port defined
#ifdef ESP32
#define LED_BUILTIN 17
#endif

// Define LoRa parameters
#define RF_FREQUENCY 433500000  // Hz
#define TX_OUTPUT_POWER 2		// dBm  was 22.
#define LORA_BANDWIDTH 2		// [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 12 // [SF7..SF12]
#define LORA_CODINGRATE 1		// [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0   // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 3000
#define TX_TIMEOUT_VALUE 3000

#define BUFFER_SIZE 64 // Define the payload size here

static RadioEvents_t RadioEvents;
static uint16_t BufferSize = BUFFER_SIZE;
static uint8_t RcvBuffer[BUFFER_SIZE];
static uint8_t TxdBuffer[BUFFER_SIZE];
static bool isMaster = true;
const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";

time_t timeToSend;
time_t cadTime;

uint8_t pingCnt = 0;
uint8_t pongCnt = 0;

void setup()
{
  // Define the HW configuration between MCU and SX126x
  hwConfig.CHIP_TYPE = SX1262_CHIP;     // Example uses an eByte E22 module with an SX1262
  hwConfig.PIN_LORA_RESET = PIN_LORA_RESET; // LORA RESET
  hwConfig.PIN_LORA_NSS = PIN_LORA_NSS;  // LORA SPI CS
  hwConfig.PIN_LORA_SCLK = PIN_LORA_SCLK;   // LORA SPI CLK
  hwConfig.PIN_LORA_MISO = PIN_LORA_MISO;   // LORA SPI MISO
  hwConfig.PIN_LORA_DIO_1 = PIN_LORA_DIO_1; // LORA DIO_1
  hwConfig.PIN_LORA_BUSY = PIN_LORA_BUSY;   // LORA SPI BUSY
  hwConfig.PIN_LORA_MOSI = PIN_LORA_MOSI;   // LORA SPI MOSI
  hwConfig.RADIO_TXEN = RADIO_TXEN;     // LORA ANTENNA TX ENABLE
  hwConfig.RADIO_RXEN = RADIO_RXEN;     // LORA ANTENNA RX ENABLE
  hwConfig.USE_DIO2_ANT_SWITCH = false;   // Was true. Example uses an CircuitRocks Alora RFM1262 which uses DIO2 pins as antenna control
  hwConfig.USE_DIO3_TCXO = true;        // Example uses an CircuitRocks Alora RFM1262 which uses DIO3 to control oscillator voltage
  hwConfig.USE_DIO3_ANT_SWITCH = false;  // Only Insight ISP4520 module uses DIO3 as antenna control

  // Initialize Serial for debug output
  Serial.begin(115200);

	pinMode(LED_BUILTIN, OUTPUT);
  flash_LED();

	Serial.println("=====================================");
	Serial.println("SX126x PingPong test");
	Serial.println("=====================================");
  Serial.print("MOSI: ");Serial.println(MOSI);
  Serial.print("MISO: ");Serial.println(MISO);
  Serial.print("RESET: ");Serial.println(PIN_LORA_RESET);
  Serial.print("DIO_1: ");Serial.println(PIN_LORA_DIO_1);
  Serial.print("BUSY: ");Serial.println(PIN_LORA_BUSY);
  Serial.print("SCLK: ");Serial.println(PIN_LORA_SCLK);
  Serial.print("SS: ");Serial.println(SS);

#ifdef ESP32
	Serial.println("MCU Espressif ESP32");
#endif

	uint8_t deviceId[8];

	BoardGetUniqueId(deviceId);
	Serial.printf("BoardId: %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X\n",
				  deviceId[7],
				  deviceId[6],
				  deviceId[5],
				  deviceId[4],
				  deviceId[3],
				  deviceId[2],
				  deviceId[1],
				  deviceId[0]);

  flash_LED();
	// Initialize the LoRa chip
	Serial.println("Starting lora_hardware_init ....");
  Serial.println("TEST_01");
  
	lora_hardware_init(hwConfig);
  flash_LED();
  Serial.println("hwConfig initialised.");
  Serial.println("TEST_02");
  
	// Initialize the Radio callbacks
	RadioEvents.TxDone = OnTxDone;
	RadioEvents.RxDone = OnRxDone;
	RadioEvents.TxTimeout = OnTxTimeout;
	RadioEvents.RxTimeout = OnRxTimeout;
	RadioEvents.RxError = OnRxError;
	RadioEvents.CadDone = OnCadDone;

  Serial.println("TEST_03");
  flash_LED();
	// Initialize the Radio
	Radio.Init(&RadioEvents);
  flash_LED();
  Serial.println("Radio initialised.");
  Serial.println("TEST_04");
  
  Radio.Standby();
  
  // Required SyncWord = 0x2414.
  uint8_t my_syncword_0 = 0x14;
  uint8_t my_syncword_1 = 0x24;
  uint16_t readSyncWord = 0;

  SX126xReadRegisters(REG_LR_SYNCWORD, (uint8_t *)&readSyncWord, 0);
  Serial.print("My SyncWord 0: ");Serial.print("0x");Serial.println(readSyncWord,HEX);
  readSyncWord = 0;
  SX126xReadRegisters(REG_LR_SYNCWORD, (uint8_t *)&readSyncWord, 1);
  Serial.print("My SyncWord 1: ");Serial.print("0x");Serial.println(readSyncWord,HEX);
  readSyncWord = 0;
  SX126xReadRegisters(REG_LR_SYNCWORD, (uint8_t *)&readSyncWord, 2);
  Serial.print("My SyncWord 2: ");Serial.print("0x");Serial.println(readSyncWord,HEX);
  Serial.println();

  SX126xWriteRegister(REG_LR_SYNCWORD, my_syncword_0);
  SX126xWriteRegister(REG_LR_SYNCWORD +1, my_syncword_1);

  readSyncWord = 0;
  SX126xReadRegisters(REG_LR_SYNCWORD, (uint8_t *)&readSyncWord, 0);
  Serial.print("My SyncWord 0: ");Serial.print("0x");Serial.println(readSyncWord,HEX);
  readSyncWord = 0;
  SX126xReadRegisters(REG_LR_SYNCWORD, (uint8_t *)&readSyncWord, 1);
  Serial.print("My SyncWord 1: ");Serial.print("0x");Serial.println(readSyncWord,HEX);
  readSyncWord = 0;
  SX126xReadRegisters(REG_LR_SYNCWORD, (uint8_t *)&readSyncWord, 2);
  Serial.print("My SyncWord 2: ");Serial.print("0x");Serial.println(readSyncWord,HEX);
  
	// Set Radio channel
	Radio.SetChannel(RF_FREQUENCY);
  flash_LED();
  Serial.println("Frequency set.");
  Serial.println("TEST_05");

	// Set Radio TX configuration
	Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
					  LORA_SPREADING_FACTOR, LORA_CODINGRATE,
					  LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
					  true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);

	// Set Radio RX configuration
	Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
					  LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
					  LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
					  0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

	// Start LoRa
  Serial.println("TEST_06");
	Serial.println("Starting Radio.Rx .....");
	Radio.Rx(RX_TIMEOUT_VALUE);
  Serial.println("TEST_07");
  flash_LED();
  Serial.println("Radio.Rx started.");
  Serial.println("Waiting for LoRa transmission ....");
  pinMode(27, OUTPUT); // LORA ANTENNA RX ENABLE
  // digitalWrite(27, HIGH); // LORA ANTENNA RX ENABLE
  digitalWrite(27, LOW); // LORA ANTENNA RX DIS-ABLE
  Serial.println("TEST_08");
  // LOG_LIB("BRD", "SyncWord = %04X", readSyncWord);
	timeToSend = millis();
}

void loop()
{
	// We are on FreeRTOS, give other tasks a chance to run
	delay(100);
	yield();
}

/**@brief Function to be executed on Radio Tx Done event
 */
void OnTxDone(void)
{
	Serial.println("OnTxDone");
	Radio.Rx(RX_TIMEOUT_VALUE);
}

/**@brief Function to be executed on Radio Rx Done event
 */
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
	Serial.println("OnRxDone");
	delay(10);
	BufferSize = size;
	memcpy(RcvBuffer, payload, BufferSize);

	Serial.printf("RssiValue=%d dBm, SnrValue=%d\n", rssi, snr);

	for (int idx = 0; idx < size; idx++)
	{
		Serial.printf("%02X ", RcvBuffer[idx]);
	}
	Serial.println("");

	digitalWrite(LED_BUILTIN, HIGH);

	if (isMaster == true)
	{
		if (BufferSize > 0)
		{
			if (strncmp((const char *)RcvBuffer, (const char *)PongMsg, 4) == 0)
			{
				Serial.println("Received a PONG in OnRxDone as Master");

				// Wait 500ms before sending the next package
				delay(500);

				// Check if our channel is available for sending
				Radio.Standby();
				Radio.SetCadParams(LORA_CAD_08_SYMBOL, LORA_SPREADING_FACTOR + 13, 10, LORA_CAD_ONLY, 0);
				cadTime = millis();
				Radio.StartCad();
				// Sending next Ping will be started when the channel is free
			}
			else if (strncmp((const char *)RcvBuffer, (const char *)PingMsg, 4) == 0)
			{ // A master already exists then become a slave
				Serial.println("Received a PING in OnRxDone as Master");
				isMaster = false;
				Radio.Rx(RX_TIMEOUT_VALUE);
			}
			else // valid reception but neither a PING or a PONG message
			{	// Set device as master and start again
				isMaster = true;
				Radio.Rx(RX_TIMEOUT_VALUE);
			}
		}
	}
	else
	{
		if (BufferSize > 0)
		{
			if (strncmp((const char *)RcvBuffer, (const char *)PingMsg, 4) == 0)
			{
				Serial.println("Received a PING in OnRxDone as Slave");

				// Check if our channel is available for sending
				Radio.Standby();
				Radio.SetCadParams(LORA_CAD_08_SYMBOL, LORA_SPREADING_FACTOR + 13, 10, LORA_CAD_ONLY, 0);
				cadTime = millis();
				Radio.StartCad();
				// Sending Pong will be started when the channel is free
			}
			else // valid reception but not a PING as expected
			{	// Set device as master and start again
				Serial.println("Received something in OnRxDone as Slave");
				isMaster = true;
				Radio.Rx(RX_TIMEOUT_VALUE);
			}
		}
	}
}

/**@brief Function to be executed on Radio Tx Timeout event
 */
void OnTxTimeout(void)
{
	// Radio.Sleep();
	Serial.println("OnTxTimeout");
	digitalWrite(LED_BUILTIN, LOW);

	Radio.Rx(RX_TIMEOUT_VALUE);
}

/**@brief Function to be executed on Radio Rx Timeout event
 */
void OnRxTimeout(void)
{
	Serial.println("");
  Serial.println("OnRxTimeout");

	digitalWrite(LED_BUILTIN, LOW);

	if (isMaster == true)
	{
		// Wait 500ms before sending the next package
		delay(500);

		// Check if our channel is available for sending
		Radio.Standby();
		Radio.SetCadParams(LORA_CAD_08_SYMBOL, LORA_SPREADING_FACTOR + 13, 10, LORA_CAD_ONLY, 0);
		cadTime = millis();
		Radio.StartCad();
		// Sending the ping will be started when the channel is free
	}
	else
	{
		// No Ping received within timeout, switch to Master
		isMaster = true;
		// Check if our channel is available for sending
		Radio.Standby();
		Radio.SetCadParams(LORA_CAD_08_SYMBOL, LORA_SPREADING_FACTOR + 13, 10, LORA_CAD_ONLY, 0);
		cadTime = millis();
		Radio.StartCad();
		// Sending the ping will be started when the channel is free
	}
}

/**@brief Function to be executed on Radio Rx Error event
 */
void OnRxError(void)
{
	Serial.println("OnRxError");
	digitalWrite(LED_BUILTIN, LOW);

	if (isMaster == true)
	{
		// Wait 500ms before sending the next package
		delay(500);

		// Check if our channel is available for sending
		Radio.Standby();
		Radio.SetCadParams(LORA_CAD_08_SYMBOL, LORA_SPREADING_FACTOR + 13, 10, LORA_CAD_ONLY, 0);
		cadTime = millis();
		Radio.StartCad();
		// Sending the ping will be started when the channel is free
	}
	else
	{
		Radio.Rx(RX_TIMEOUT_VALUE);
	}
}

/**@brief Function to be executed on Radio Rx Error event
 */
void OnCadDone(bool cadResult)
{
	time_t duration = millis() - cadTime;
  flash_LED();
	if (cadResult)
	{
		Serial.printf("CAD returned channel busy after %ldms\n", duration);
		Radio.Rx(RX_TIMEOUT_VALUE);
	}
	else
	{
		Serial.printf("CAD returned channel free after %ldms\n", duration);
		if (isMaster)
		{
			Serial.println("Sending a PING in OnCadDone as Master");
			// Send the next PING frame
			TxdBuffer[0] = 'P';
			TxdBuffer[1] = 'I';
			TxdBuffer[2] = 'N';
			TxdBuffer[3] = 'G';
		}
		else
		{
			Serial.println("Sending a PONG in OnCadDone as Slave");
			// Send the reply to the PONG string
			TxdBuffer[0] = 'P';
			TxdBuffer[1] = 'O';
			TxdBuffer[2] = 'N';
			TxdBuffer[3] = 'G';
		}
		// We fill the buffer with numbers for the payload
		for (int i = 4; i < BufferSize; i++)
		{
			TxdBuffer[i] = i - 4;
		}

		Radio.Send(TxdBuffer, BufferSize);
	}
}
void flash_LED()
{
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
}
