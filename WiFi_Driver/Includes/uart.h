/*
 * uart.h
 *
 *  Created on: 2016/01/07
 *      Author: masuda
 */

#ifndef INCLUDES_UART_H_
#define INCLUDES_UART_H_

#include	<MK12D5.h>

#ifdef __cplusplus
extern "C" {
#endif

#define	UART_NUM		3
//#define	UART_RX_BUFSIZE	128
//#define	UART_TX_BUFSIZE	128
#define	UART_RX_TIMEOUT	10

enum{
	eUART_IDLE = 0,		// IDLE status
	eUART_RX,			// Receive status
	eUART_TX,			// Transmit status
	eUART_TIMEOUT,		// Timeout status
	eUART_RX_BUF_FULL,	// Rx buffer overflow
	eUART_ERROR			// UART H/W Error
};

enum{
	Error_None = 0,
	Error_Hw,
	Error_Rx_Buf_Full,
};

typedef struct __UART_ERROR_INFO{
	uint32_t	errType;
	struct{
		uint8_t	s1;
		uint8_t	s2;
	} reg;
} UART_ERROR_INFO;

typedef bool	(*RECEIVE_COMPLETED_HANDLER)(char* buffer, uint32_t length);
typedef	void	(*UART_ERROR_INTERRUPT_HANDLER)(uint8_t status);

void Uart_Config(uint32_t index, uint32_t baudrate);
void Uart_Set_Baudrate(uint32_t index, uint32_t baudrate);
void Uart_Register_Receive_Completed_Handler(uint32_t index, RECEIVE_COMPLETED_HANDLER user_handler);
void Uart_Register_Err_Interrupt_Handler(uint32_t index, UART_ERROR_INTERRUPT_HANDLER user_handler);
bool Uart_Send_Data_Async(uint32_t index, uint8_t* buf, uint32_t length);
bool Uart_Send_Data_Sync(uint32_t index, uint8_t* buf, uint32_t length);
void Uart_Set_Receive_Buffer(uint32_t index, uint8_t* buf, uint32_t length);
uint32_t Uart_Receive_Data_Sync(uint32_t index, uint8_t* buf, uint32_t length);
uint32_t Uart_Receive_Data_Count(uint32_t index);
//bool Uart_Receive_Completed(uint32_t index);
//bool Uart_Transmit_Completed(uint32_t index);
uint32_t Uart_Get_Status(uint32_t index);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDES_UART_H_ */
