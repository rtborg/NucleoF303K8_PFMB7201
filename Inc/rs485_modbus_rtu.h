#ifndef INC_RS485_MODBUS_RTU_H_
#define INC_RS485_MODBUS_RTU_H_

#include "main.h"
#include <string.h>


//Modbus command structure definition and buffer
typedef struct ModbusCommand {
	uint8_t		address;
	uint8_t		function_code;
	uint8_t		data[4];
	uint8_t		crc[2];
}ModbusCommand;

// USART1 Modbus API
void USART1_RS485_Init(uint32_t device_address);
void USART1_IRQHandler(void);
void USART1_putchar(uint8_t ch);
void USART1_putstring(uint8_t *s, uint8_t size);
uint8_t modbus_command_available(void);
ModbusCommand get_modbus_command(void);
uint8_t modbus_command_check_crc(ModbusCommand mc);
uint16_t modbus_generate_crc(uint8_t *message, uint8_t message_len);
void MX_CRC_Init(void);

#endif /* INC_RS485_MODBUS_RTU_H_ */
