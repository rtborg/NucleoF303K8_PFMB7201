#include "rs485_modbus_rtu.h"

static UART_HandleTypeDef huart1;										// USART1 handle
CRC_HandleTypeDef hcrc;													// CRC handle

/*
 * UART1 Inettupt based-transmit buffer
 * */
#define UART1_TX_BUFFER_SIZE 64
static volatile uint8_t uart1TxHead = 0;
static volatile uint8_t uart1TxTail = 0;
static volatile uint8_t uart1TxBuffer[UART1_TX_BUFFER_SIZE];
static volatile uint8_t uart1TxBufferRemaining;

/*
 * Modbus buffer - 8 bytes, populated by USART1 IRQ. Not to be used by main
 * */
#define MODBUS_COMMAND_LENGTH	8
static volatile uint8_t modbus_buffer_head = 0;
static volatile uint8_t modbus_buffer_tail = 0;
static volatile uint8_t modbus_rx_buffer[MODBUS_COMMAND_LENGTH];
static volatile uint modbus_buffer_count = 0;

static uint32_t modbus_device_address = 0x0;								// Device address; updated once in USART1_RS485_Init()
static const uint8_t modbus_function_code = 0x04;							// Function code

#define COMMAND_BUFFER_SIZE	8												// Maximum modbus buffer size
static volatile ModbusCommand commands[COMMAND_BUFFER_SIZE];				// Declaration of modbus command buffer
static volatile uint8_t mc_head = 0;
static volatile uint8_t mc_tail = 0;
static volatile uint8_t mc_count = 0;



/****************************************************************************************************************/
/**
 * @brief USART1 Initialization Function (for USART1 with Modbus and CRC functions)
 * @param device_address
 * @retval None
 */
/****************************************************************************************************************/
void USART1_RS485_Init(uint32_t device_address) {
	modbus_device_address = device_address;

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_2;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 16, 16) != HAL_OK) {
		Error_Handler();
	}

	huart1.Instance->CR2 |= USART_CR2_RTOEN;						// Enable receiver timeout for Modbus
	huart1.Instance->RTOR |= 0x50;									// Timeout: 40 bits. 1 bit @ 9600bps = 1/9600 = 104.17us. 40 bits = 4.17ms, approx. 3.5 chars * 11 bits each

	uart1TxHead = 0;												// Initialize UART buffer variables
	uart1TxTail = 0;
	uart1TxBufferRemaining = sizeof(uart1TxBuffer);

	// Set interrupt priority & enable interrupts
	HAL_NVIC_SetPriority(USART1_IRQn, 5, 5);						// Set interrupt priority
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RTO);						// Enable Receive Timeout interrupt
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);					// Enable Receive interrupt
	HAL_NVIC_EnableIRQ(USART1_IRQn);

	MX_CRC_Init();													// Initialize CRC calculation unit
}


/****************************************************************************************************************/
/**
 * @brief USART1 interrupt service routine
 */
/****************************************************************************************************************/
void USART1_IRQHandler(void) {

	if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {											// Handle RX interrupt
		huart1.Instance->RTOR |= 0x50;															// Test only: set RTOR again after each reception
		modbus_rx_buffer[modbus_buffer_head++] = USART1->RDR;									// Place char in modbus command buffer (8 bytes only)
		if (modbus_buffer_head == MODBUS_COMMAND_LENGTH) modbus_buffer_head = 0;				// Wrap-around buffer head
		modbus_buffer_count++;																	// Increase modbus buffer count
	}

	if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)) {					// Clear overrun flag
		__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_ORE);
	}

	if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE)) {					// Handle transmit interrupt
		if(sizeof(uart1TxBuffer) > uart1TxBufferRemaining) {			// If the number of free spaces in the buffer is less than the size of the buffer that means there's still characters to be sent
			USART1->TDR = uart1TxBuffer[uart1TxTail++];					// Place char in the TX buffer. This also clears the interrupt flag
			if(sizeof(uart1TxBuffer) <= uart1TxTail)					// Wrap around tail if needed
			{
				uart1TxTail = 0;
			}
			uart1TxBufferRemaining++;			 						// Increase number of remaining characters
		} else {														// If remaining chars == buffer size, there's nothing to transmit
			USART1->CR1 &= ~USART_CR1_TXEIE;							// Disable TXE interrupt
			__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_TXE);				// Clear flag
		}
	}


	if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RTOF)) {					// The Receive Timeout interrupt happens when an idle tie of more than 40 bits (3.5 modbus 11 bit chars)
		__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RTOF);					// Clear receive timeout interrupt flag

		if (modbus_buffer_head == 0 && modbus_buffer_count == 8) {		// Check modbus command buffer. If head == 0 and count == 8, an 8-byte command has been received and head has wrapped around
			modbus_buffer_count = 0;									// Zero modbus command buffer count

			if (modbus_rx_buffer[0] == modbus_device_address) {			// Check if modbuss address matches
				if (modbus_rx_buffer[1] == modbus_function_code) {		// Check if function code matches

					commands[mc_head].address = modbus_rx_buffer[0];	// Copy received data into modbus command buffer
					commands[mc_head].function_code = modbus_rx_buffer[1];
					commands[mc_head].data[0] = modbus_rx_buffer[2];
					commands[mc_head].data[1] = modbus_rx_buffer[3];
					commands[mc_head].data[2] = modbus_rx_buffer[4];
					commands[mc_head].data[3] = modbus_rx_buffer[5];
					commands[mc_head].crc[0] = modbus_rx_buffer[6];
					commands[mc_head].crc[1] = modbus_rx_buffer[7];

					mc_head++;											// Increase and wrap-around buffer head and count variables
					if (mc_head == COMMAND_BUFFER_SIZE) mc_head = 0;
					mc_count++;
					if (mc_count == COMMAND_BUFFER_SIZE) mc_count = 0;
				} else {
					// @TODO: send exception for wrong function code
				}
			}
		} else {														// If counter is not 8 and head is not wrapped around, clear the buffer as a non-8-byte message has been received
			modbus_buffer_head = 0;										// and wait for another modbus message
			modbus_buffer_count = 0;
		}
	}
}

/****************************************************************************************************************/
/**
 * @brief USART1 putchar function
 * @param ch
 */
 /****************************************************************************************************************/
void USART1_putchar(uint8_t ch) {
	while (0 == uart1TxBufferRemaining) continue;						// Wait until there's a free space in the transmit buffer

	if (0 == (USART1->CR1 & USART_CR1_TXEIE)) {							// If TXE interrupt is disabled, directly put char in USART TDR
		USART1->TDR = ch;
	} else {															// If TXE interrupt is enabled, there's transmission going on
		USART1->CR1 &= ~USART_CR1_TXEIE;								// Disable TXE interrupt temporarily
		uart1TxBuffer[uart1TxHead++] = ch;								// Place data in buffer
		if(sizeof(uart1TxBuffer) <= uart1TxHead)						// Wrap around buffer head
		{
			uart1TxHead = 0;
		}
		uart1TxBufferRemaining--;										// Decrease number of free spaces in buffer
	}

	USART1->CR1 |= USART_CR1_TXEIE;										// Enable TXE interrupt
}

/****************************************************************************************************************/
/**
 * @brief USART1 interrupt-driven putstring function
 * @param s
 * @param size
 */
/****************************************************************************************************************/
void USART1_putstring(uint8_t *s, uint8_t size) {

	for (int i = 0; i < size; i++) {
		USART1_putchar(s[i]);
	}
}

/****************************************************************************************************************/
/**
 * @brief Check if there's commands in the modbus buffer
 * @return Number of commands awaiting to be read from the buffer
 */
/****************************************************************************************************************/
uint8_t modbus_command_available(void) {
	return mc_count;
}

/****************************************************************************************************************/
/**
 * @brief Retrieve next modbus command from buffer
 * @return the modbus command. If none is available, a modbus command with address set to 0 is returned
 */
/****************************************************************************************************************/
ModbusCommand get_modbus_command(void) {
	ModbusCommand m_command;
	m_command.address = 0x00;

	if (mc_count > 0) {
		m_command.address = commands[mc_tail].address;
		m_command.function_code = commands[mc_tail].function_code;
		m_command.data[0] = commands[mc_tail].data[0];
		m_command.data[1] = commands[mc_tail].data[1];
		m_command.data[2] = commands[mc_tail].data[2];
		m_command.data[3] = commands[mc_tail].data[3];
		m_command.crc[0] = commands[mc_tail].crc[0];
		m_command.crc[1] = commands[mc_tail].crc[1];

		mc_tail++;
		mc_count--;
		if (mc_tail == COMMAND_BUFFER_SIZE) mc_tail = 0;
		return m_command;
	} else {
		return m_command;
	}
}

/****************************************************************************************************************/
/**
 * @brief Check CRC of a ModbusCommand
 * @param mc
 * @return 0 on success
 */
/****************************************************************************************************************/
uint8_t modbus_command_check_crc(ModbusCommand mc) {
	// Copy CRC into a 32-bit int
	uint32_t message_crc =  ((uint32_t) mc.crc[1] << 8UL) | (mc.crc[0]);

	// Copy address field, function code and data field into an array
	uint8_t temp[6];
	temp[0] = mc.address;
	temp[1] = mc.function_code;
	temp[2] = mc.data[0];
	temp[3] = mc.data[1];
	temp[4] = mc.data[2];
	temp[5] = mc.data[3];

	uint32_t calculated_crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)&temp, 6);

	if (calculated_crc == message_crc) {
		return 0;
	} else {
		return 1;
	}
}

/****************************************************************************************************************/
/**
 * @brief Generate a CRC for the given message
 * @param message
 * @param message_len
 * @return 16-bit CRC
 */
/****************************************************************************************************************/
uint16_t modbus_generate_crc(uint8_t *message, uint8_t message_len) {
	uint32_t temp = HAL_CRC_Calculate(&hcrc, (uint32_t*)message, message_len);

	return (uint16_t) temp;
}

/****************************************************************************************************************/
/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
/****************************************************************************************************************/
void MX_CRC_Init(void)
{
	hcrc.Instance = CRC;
	hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;			// Disable default polynomial
	hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;				// Disable initial value
	hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_BYTE;		// Required?
	hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;	// Required?
	hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;						// Input data format?
	hcrc.Init.GeneratingPolynomial = 0x8005UL;								// Modbus 16-bit polynomial
	hcrc.Init.CRCLength = CRC_POLYLENGTH_16B;								// 16-bit CRC
	hcrc.Init.InitValue = 0xffff;											// Initial CRC register value

	__HAL_RCC_CRC_CLK_ENABLE();												// Enable clock?

	if (HAL_CRC_Init(&hcrc) != HAL_OK)
	{
		Error_Handler();
	}
}



