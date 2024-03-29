#include "main.h"
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "rs485_modbus_rtu.h"

#define MEASURE	0x00010001
#define VREFINT_CAL_ADDR ((uint16_t*)((uint32_t)0x1FFFF7BA))			// VREFINT_CAL value. See datasheet for converting ADC to absolute voltage

// Peripheral handles as generated by Cube
UART_HandleTypeDef huart2;
IWDG_HandleTypeDef hiwdg;
ADC_HandleTypeDef hadc2;
OPAMP_HandleTypeDef hopamp2;
DMA_HandleTypeDef hdma_adc2;

// ADC Data structure, also accessed by ADC callback
typedef struct ADC_Data {
	volatile uint32_t adc_value_channel_3;
	volatile uint32_t adc_vrefint_data;
	volatile bool adc_conversion_complete;
	uint16_t vrefint_cal;
	double vdd;
}ADC_Data;

// Functions generated by Cube
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_ADC2_Init(void);
static void MX_OPAMP2_Init(void);

// User functions
uint32_t get_modbus_address();											// Function to get modbus device address from reading 5-bit dip switch
void process_modbus_command(ModbusCommand mc, float step_per_liter, float zero_value);		// Process the received modbus command and respond
float get_adc_value(void);												// Return the ADC value on channel 3
bool self_calibration(float *spl, float *zo);							// Perform sensor calibration. See function description below
int16_t get_flow(float step_per_liter, float zero_value);
void HAL_IncTick(void);													// The function is defined as weak in stm32f3xx_hal.c and is redefined in main in order to use the sys tick interrupt (ocurring each ms)

// User variables
static uint8_t uart_buffer[64] = {0};									// Buffer used for printing strings to USART
static uint32_t device_modbus_address = 0;								// Device modbus address is self-populated by get_modbus_address()
static volatile uint32_t adc_value = 0;									// The ADC value is the average of 16 measurements
static volatile uint32_t vrefint_value = 0;								// Internal VREFINT value; connected to channel 18, also averaged
static volatile bool adc_conversion_complete = false;					// This flag needs to be polled when an ADC conversion is started
uint32_t adc_buffer[48];												// Buffer used for ADC2 DMA interrupt
ADC_Data adc_data;														// ADC_Data structure used for passing data between main and the ADC interrupt callback
static float adc_step_per_liter = 0;
static float zero_offset = 0;

int main(void) {

	// Initialize peripherals
	HAL_Init();
	SystemClock_Config();
	MX_DMA_Init();
	MX_ADC2_Init();
	MX_OPAMP2_Init();
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	HAL_OPAMP_Start(&hopamp2);
	device_modbus_address = get_modbus_address();
	USART1_RS485_Init(device_modbus_address);
	MX_IWDG_Init();

	adc_data.adc_conversion_complete = false;
	adc_data.adc_value_channel_3 = 0;
	adc_data.adc_vrefint_data = 0;
	adc_data.vrefint_cal = *VREFINT_CAL_ADDR;
	adc_data.vdd = 0.0;

	if ( self_calibration(&adc_step_per_liter, &zero_offset) == false ) {
		// @TODO: Take action if calibration fails
	}

	while (1) {

		if (modbus_command_available()) {																// Check if a command has been received
			ModbusCommand mc = get_modbus_command();													// Read modbus command
			if (mc.address ==  device_modbus_address & (modbus_command_check_crc(mc) == 0)) {			// Check command validity
				process_modbus_command(mc, adc_step_per_liter, zero_offset);							// Parse command and take action
			}
		}

		HAL_IWDG_Refresh(&hiwdg);

	}
}

/****************************************************************************************************************/
/**
 * @brief System Clock Configuration
 * @retval None
 */
/****************************************************************************************************************/
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1
			| RCC_PERIPHCLK_ADC12;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}


/****************************************************************************************************************/
/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
/****************************************************************************************************************/
static void MX_USART2_UART_Init(void) {

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
}

/****************************************************************************************************************/
/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
/****************************************************************************************************************/
static void MX_IWDG_Init(void)
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 1250;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }

}

/****************************************************************************************************************/
/**
 * @brief GPIO Initialization Function. LD3 is on-board LED; Debug_Pin is PB4 and is used for various debug
 * purposes
 * @param None
 * @retval None
 */
/****************************************************************************************************************/
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();


	/*Configure GPIO pins : DIP_Bit_0_Pin DIP_Bit_1_Pin DIP_Bit_2_Pin DIP_Bit_3_Pin
                           DIP_Bit_4_Pin */
	GPIO_InitStruct.Pin = DIP_Bit_0_Pin|DIP_Bit_1_Pin|DIP_Bit_2_Pin|DIP_Bit_3_Pin
			|DIP_Bit_4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LD3_Pin Debug_Pin_Pin */
	GPIO_InitStruct.Pin = LD3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 3;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;

  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }


  /* USER CODE BEGIN ADC2_Init 2 */

  // Calibration as per https://www.youtube.com/watch?v=qqGsy06mris
  ADC2->CR &= ~ADC_CR_ADEN;			// Disable ADC
  ADC2->CR |= ADC_CR_ADCAL;			// Start calibration
  while ( (ADC2->CR & ADC_CR_ADCAL) != 0);
  uint32_t calfact = ADC2->CALFACT;
  /* USER CODE END ADC2_Init 2 */


}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief OPAMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP2_Init(void)
{

  /* USER CODE BEGIN OPAMP2_Init 0 */

  /* USER CODE END OPAMP2_Init 0 */

  /* USER CODE BEGIN OPAMP2_Init 1 */

  /* USER CODE END OPAMP2_Init 1 */
	  hopamp2.Instance = OPAMP2;
	  hopamp2.Init.Mode = OPAMP_FOLLOWER_MODE;
	  hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO2;
	  hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
	  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_USER;
	  hopamp2.Init.TrimmingValueP = 0;
	  hopamp2.Init.TrimmingValueN = 0;
	  if (HAL_OPAMP_Init(&hopamp2) != HAL_OK)
	  {
	    Error_Handler();
	  }
//	  if (HAL_OPAMP_SelfCalibrate(&hopamp2) != HAL_OK)
//	  {
//	    Error_Handler();
//	  }
  /* USER CODE BEGIN OPAMP2_Init 2 */

  /* USER CODE END OPAMP2_Init 2 */

}

/**
 * ADC Conversion via interrupt callback function. Path of execution:
 * The ADC interrupt handler in stm32f3xx_it.c calls the handler HAL_ADC_IRQHandler(&hadc2) in the
 * driver files. The HAL_ADC_ConvCpltCallback() function is the called after conversion is complete.
 *
 * Need to perform 16 consecutive conversions and average the result
 *
 * @param hadc
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	adc_data.adc_conversion_complete = true;
}


/****************************************************************************************************************/
/**
 * @brief Get device modbus address from reading 5-bit dip switch connected to PA0, PA1, PA3, PA4, PA5
 * @note The function is called immediately after boot ONCE. If change in address is needed, set up the address
 * and reset the circuit
 * @return Modbus address
 */
/****************************************************************************************************************/
uint32_t get_modbus_address() {
	/**
	 * Pin mapping
	 * DIP SWITCH BIT			MCU PIN
	 * 0						PA0
	 * 1						PA1
	 * 2						PA3
	 * 3						PA4
	 * 4						PA5
	 */

	uint32_t porta_idr = GPIOA->IDR;					// Copy input data register
	porta_idr &= 0x0000003BUL;							// Mask all other bits
	uint32_t mask = porta_idr >> 1;
	mask &= 0x1c;
	porta_idr &= 0x00000003;
	porta_idr |= mask;

	return porta_idr;
}

/****************************************************************************************************************/
/**
 * @brief Process modbus command
 * @note Description of commands under function code 0x04
 * Register 0x01 - flow measurement; 1 register
 * Register 0x02 - serial number; 2 registers
 * Register 0x03 - soft reset; response OK in ASCII
 * @param mc The command for processing
 */
/****************************************************************************************************************/
void process_modbus_command(ModbusCommand mc, float step_per_liter, float zero_value) {

	// Get modbus start register and number of registers from data field
	uint32_t data_field = (uint32_t)(mc.data[0] << 24UL) | (uint32_t)(mc.data[1] << 16UL) | (uint32_t)(mc.data[2] << 8UL) | (uint32_t)(mc.data[3]);
	uint8_t response[9] = {};												// Response array
	response[0] = mc.address;												// Copy device address
	response[1] = mc.function_code;											// Copy function code

	if (mc.function_code == 0x04) {											// Function code 0x04 - Read Holding registers
		// Address 0x01; 1 register of 16 bits (int16_t)
		if (data_field == 0x00010001) {										// Starting address 0001, quantity of input registers: 0001
			int16_t temp = 0xffff;
			response[2] = 2;												// 2 bytes (1 register) in payload
			temp = get_flow(step_per_liter, zero_value);					// Get measurement
			response[3] = (uint8_t) (temp >> 8);							// Copy measurement in buffer
			response[4] = (uint8_t) (temp & 0xff);
			uint16_t crc = modbus_generate_crc(response, 5);				// Generate CRC
			response[5] = (uint8_t) (crc & 0xff);							// Copy CRC in buffer
			response[6] = (uint8_t) (crc >> 8);
			USART1_putstring(response, 7);									// Send measurement to USART1 (rs485)
		}

//		// SFM4100 Serial number; 0x02; 2 registers
//		if (data_field == 0x00020002) {
//			uint32_t sfm4100_serial_number = 0;
//			sfm_err = sfm4100_read_serial_number(&sfm4100_serial_number);	// Read sensor serial number
//
//			response[2] = 4;												// 4 bytes in payload
//			response[3] = (uint8_t) (sfm4100_serial_number >> 24);
//			response[4] = (uint8_t) (sfm4100_serial_number >> 16);
//			response[5] = (uint8_t) (sfm4100_serial_number >> 8);
//			response[6] = (uint8_t) (sfm4100_serial_number);
//			uint16_t crc = modbus_generate_crc(response, 7);				// Generate CRC
//			response[7] = (uint8_t) (crc & 0xff);
//			response[8] = (uint8_t) (crc >> 8);
//			USART1_putstring(response, 9);									// Send serial no. to USART1 (rs485)
//
//			if (sfm_err) sfm4100_soft_reset();								// If an error is returned when reading sensor, issue soft reset
//		}
//
//		// Address 0x03; 1 register
//		if (data_field == 0x00030001) {
//			sfm4100_soft_reset();											// Issue soft reset
//			response[2] = 2;												// 2 bytes (1 register) in payload
//			response[3] = 0x4f;												// ASCII 'O'
//			response[4] = 0x4b;												// ASCII 'K'
//			uint16_t crc = modbus_generate_crc(response, 5);				// Generate CRC
//			response[5] = (uint8_t) (crc & 0xff);							// Copy CRC in buffer
//			response[6] = (uint8_t) (crc >> 8);
//			USART1_putstring(response, 7);									// Send response to USART1 (rs485)
//		}
//
//		// Address 0x04; 1 register; report slave ID
//		if (data_field == 0x00040001) {
//			response[2] = 2;												// 2 bytes (1 register) in payload
//			response[3] = (uint8_t) (device_modbus_address >> 8);			// Copy measurement in buffer
//			response[4] = (uint8_t) (device_modbus_address & 0xff);
//			uint16_t crc = modbus_generate_crc(response, 5);				// Generate CRC
//			response[5] = (uint8_t) (crc & 0xff);							// Copy CRC in buffer
//			response[6] = (uint8_t) (crc >> 8);
//			USART1_putstring(response, 7);									// Send response to USART1 (rs485)
//		}
	}

}

/****************************************************************************************************************/
/**
 * The function triggers an ADC conversion, waits 1ms and polls the data ready flag of the ADC_Data structure.
 * The ADC scans channel 3 (connected to OPAMP2 output) and Vrefint, which is done to get the Vdd.
 * After that the function calculates the true value on ADC channel 3 using Vdd and returns it
 */
/****************************************************************************************************************/
float get_adc_value() {
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc_buffer, 48);					// Start a new conversion
	HAL_Delay(1);															// Wait a predetermined amount of time
	if (adc_data.adc_conversion_complete != true) {							// Poll flag in adc_data
		return -1.0;
	}

	adc_data.adc_conversion_complete = false;								// Reset flag
	adc_data.adc_value_channel_3 = 0;										// Clear ADC channel 3 data
	adc_data.adc_vrefint_data = 0;											// Clear ADC Vrefint data
	uint32_t channel4_adc = 0;
	uint32_t *tempbuf = adc_buffer;											// Get a pointer to the ADC buffer

	for (int x = 0; x < 16; x++) {											// Iterate over the ADC buffer and copy the data
		adc_data.adc_value_channel_3 += *tempbuf++;
		adc_data.adc_vrefint_data += *tempbuf++;
		channel4_adc += *tempbuf++;
	}

	channel4_adc = channel4_adc >> 4;
	adc_data.adc_value_channel_3 = adc_data.adc_value_channel_3 >> 4;		// Divide values by 16
	adc_data.adc_vrefint_data = adc_data.adc_vrefint_data >> 4;
	adc_data.vdd = 3.3 * adc_data.vrefint_cal / adc_data.adc_vrefint_data;	// Get current Vdd value
	float adc_result = adc_data.vdd * adc_data.adc_value_channel_3 / 4095; 	// Convert raw ADC data from channel 3
	float temp = channel4_adc * adc_data.vdd / 4095;
	return adc_result;
}


/****************************************************************************************************************/
/**
 * The function caluclates the range and step per liter of the ADC.
 * Preliminaries: The sensor needs to be connected and the flow to be zero
 */
/****************************************************************************************************************/
bool self_calibration(float *spl, float *zo)
{
	*zo = get_adc_value();												// Get current ADC data
	if ( (*zo == -1) || (*zo < 0.5) ) {											// If t == -1, an ADC timeout has occurred; if less than 500, input voltage is not correct - it should be 0.65V
		return false;
	}

	*spl = ((adc_data.vdd - *zo) / 200);
	return true;
}


int16_t get_flow(float step_per_liter, float zero_value) {
	float flow = -1;
	// Get current ADC reading
	float adc_reading = get_adc_value();

	if (adc_reading == -1) {
		return -1;
	}

	flow = (adc_reading - zero_value) / step_per_liter;
	flow = round(flow);
	return (int16_t) flow;
}

/****************************************************************************************************************/
/**
 * The function is called each time a sys tick interrupt occurs. See void SysTick_Handler(void) in stm32f3xx_it.c
 */
/****************************************************************************************************************/
void HAL_IncTick(void)
{
	uwTick += uwTickFreq;
	// User code begin
	// User code end
	if (uwTick % 256 == 0) {
		LD3_GPIO_Port->ODR ^= LD3_Pin;
	}
}


/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	// @TODO Blink a LED if error handler called
	__ASM volatile("BKPT #01");
	while (1) {

	}
	/* USER CODE END Error_Handler_Debug */
}
