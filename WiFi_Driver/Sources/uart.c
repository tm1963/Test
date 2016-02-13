/*
 * uart.c
 *
 *  Created on: 2016/01/06
 *      Author: masuda
 */

#include	<stdio.h>
#include	<string.h>
#include	<stdbool.h>
#include	"uart.h"
#include	"timer.h"

static UART_Type*	uart_base[] = UART_BASE_PTRS;	// UART Register Address Array
static RECEIVE_COMPLETED_HANDLER	_uart_receive_completed[] = {NULL, NULL, NULL};
//static UART_INTERRUPT_HANDLER	_uart_rx_tx_interrupt[] = {NULL, NULL, NULL};	// UART Rx Tx Interrupt Function Pointer Array
static UART_ERROR_INTERRUPT_HANDLER	_uart_err_interrupt[] = {NULL, NULL, NULL};		// UART Err Interrupt Function Pointer Array
static uint8_t	status[UART_NUM];
static uint32_t	rx_ptr[] = {0, 0, 0};
static uint32_t	tx_ptr[] = {0, 0, 0};
static uint32_t	rx_buffer_length[] = {0, 0, 0};
static uint32_t	tx_buffer_length[] = {0, 0, 0};
static uint8_t*	rx_buffer[UART_NUM];
static uint8_t*	tx_buffer[UART_NUM];
static uint8_t	rx_fifo_depth[UART_NUM];
static uint8_t	tx_fifo_depth[UART_NUM];
static uint8_t	fifo_depth_table[] = {
		1, 4, 8, 16, 32, 64, 128, 0
};
static uint32_t	timeout_func_id[UART_NUM];
static bool	timeout_func_installed[] = {false, false, false};

///////////////////////////////////////////////////////////////////////////////
// Function Declaration
///////////////////////////////////////////////////////////////////////////////
static void Uart_State_Machine(uint32_t index);
static void Uart_Timeout_Callback(uint32_t id, void* param);
static void Uart_Start_Timeout_Timer(uint32_t index, uint32_t timeout);
static void Uart_Stop_Timeout_Timer(uint32_t index);

///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
//		Uart_Config
// 機能：
//		UARTからデータを送受信できるようにレジスタを設定する。
// 引数：
//		index : UARTインデックス
//		baudrate : ボーレート
//
///////////////////////////////////////////////////////////////////////////////
void Uart_Config(uint32_t index, uint32_t baudrate)
{
	UART_Type*	uart = uart_base[index];

	NVIC_EnableIRQ(UART0_RX_TX_IRQn + 2 * index);
	NVIC_EnableIRQ(UART0_ERR_IRQn + 2 * index);

	SIM_SCGC4 |= (SIM_SCGC4_UART0_MASK << index);

	UART_PFIFO_REG(uart) = UART_PFIFO_RXFE(1) | UART_PFIFO_TXFE(1);			// Enable Rx and Tx FIFO
	UART_CFIFO_REG(uart) = UART_CFIFO_TXFLUSH(1) | UART_CFIFO_RXFLUSH(1);	// Flush Tx and Rx FIFO
	UART_TWFIFO_REG(uart) = UART_TWFIFO_TXWATER(0);
	UART_RWFIFO_REG(uart) = UART_RWFIFO_RXWATER(1);

	rx_fifo_depth[index] = fifo_depth_table[((UART_PFIFO_REG(uart) & UART_PFIFO_RXFIFOSIZE_MASK) >> UART_PFIFO_RXFIFOSIZE_SHIFT)];
	tx_fifo_depth[index] = fifo_depth_table[((UART_PFIFO_REG(uart) & UART_PFIFO_TXFIFOSIZE_MASK) >> UART_PFIFO_TXFIFOSIZE_SHIFT)];

	UART_C1_REG(uart) = UART_C1_PT(0)       |	// Parity Type : No meaning when PE = 0
						UART_C1_PE(0)       |	// Parity Enable : Parity Disable
						UART_C1_ILT(0)      |	// Idle Line Type Select : Idle character bit count starts after start bit.
						UART_C1_WAKE(0)     |	// Receiver Wakeup Method Select : Idle line wakeup.
						UART_C1_M(0)        |	// 9-bit or 8-bit Mode Select : 8bit data
						UART_C1_RSRC(0)     |	// Receiver Source Select : No meaning when Loop Mode Select = 0
						UART_C1_UARTSWAI(0) |	// UART Stops in Wait Mode : UART clock continues to run in Wait mode.
						UART_C1_LOOPS(0);		// Loop Mode Select : Normal Operation

	UART_C2_REG(uart) = UART_C2_SBK(0)  |	// Send Break : Normal transmitter operation.
						UART_C2_RWU(0)  |	// Receiver Wakeup Control : Disable
						UART_C2_RE(1)   |	// Receiver Enable : Enable
						UART_C2_TE(1)   |	// Transmitter Enable : Enable
						UART_C2_ILIE(0) |	// Idle Line Interrupt Enable : Disable
						UART_C2_RIE(1)  |	// Receiver Full Interrupt Enable : Enable
						UART_C2_TCIE(0) |	// Transmission Complete Interrupt Enable : Disable(Enable when transmitting)
						UART_C2_TIE(0);		// Transmitter Interrupt Enable : Disable(Enable when transmitting)

	UART_C3_REG(uart) = UART_C3_PEIE(0)  |	// Parity Error Interrupt : Disable
						UART_C3_FEIE(1)  |	// Framing Error Interrupt : Enable
						UART_C3_NEIE(1)  |	// Noise Error Interrupt : Enable
						UART_C3_ORIE(1)  |	// Overrun Error Interrupt : Enable
						UART_C3_TXINV(0) |	// Transmit Data Inversion : Not Inverted
						UART_C3_TXDIR(0) |	// Transmitter Pin Data Direction in Single-Wire mode : No meaning in Two-Wire mode
						UART_C3_T8(0)    |	// Transmit Bit 8 : No meaning in Two-Wire mode
						UART_C3_R8(0);		// Received Bit 8 : No meaning in Two-Wire mode

	UART_C4_REG(uart) = UART_C4_BRFA(0)  |	// Baud Rate Fine Adjust : Calculate later
						UART_C4_M10(0)   |	// 10-bit Mode select : No meaning in 8bit transfer
						UART_C4_MAEN2(0) |	// Match Address Mode Enable 2 : Disable
						UART_C4_MAEN1(0);	// Match Address Mode Enable 1 : Disable

	UART_MODEM_REG(uart) = UART_MODEM_TXCTSE(0)   |	// Transmitter clear-to-send disable
						   UART_MODEM_TXRTSE(0)   |	// Transmitter request-to-send disable
						   UART_MODEM_TXRTSPOL(0) |	// Transmitter request-to-send polarity
						   UART_MODEM_RXRTSE(0);	// Receiver request-to-send disable

	Uart_Set_Baudrate(index, baudrate);

	status[index] = eUART_IDLE;
	rx_ptr[index] = 0;
	tx_ptr[index] = 0;
	rx_buffer_length[index] = 0;
	tx_buffer_length[index] = 0;
}

///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
//		Uart_Set_Baudrate
// 機能：
//		UARTのボーレートを設定する。
// 引数：
//		index : UARTインデックス
//		baudrate : ボーレート
//
///////////////////////////////////////////////////////////////////////////////
void Uart_Set_Baudrate(uint32_t index, uint32_t baudrate)
{
	UART_Type*	uart = uart_base[index];
	uint16_t	sbr, brfa;

	sbr = (uint16_t)(SystemCoreClock / (baudrate * 16));
	brfa = (uint16_t)(SystemCoreClock * 2 / baudrate - ((uint32_t)sbr * 32));

	UART_BDL_REG(uart) = (uint8_t)(sbr & UART_BDL_SBR_MASK);
	UART_BDH_REG(uart) = UART_BDH_SBR((uint8_t)((sbr & 0x1f00) >> 8)) |
						 UART_BDH_RXEDGIE(0) |
						 UART_BDH_LBKDIE(0);

	UART_C4_REG(uart) &= ~UART_C4_BRFA_MASK;
	UART_C4_REG(uart) |= (uint8_t)brfa;
}

///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
//		Uart_Register_Receive_Completed_Handler
// 機能：
//		受信完了を判定するロジックをもった関数を登録する。
// 引数：
//		index : UARTインデックス
//		user_handler : 受信完了判定関数のポインタ
//
///////////////////////////////////////////////////////////////////////////////
void Uart_Register_Receive_Completed_Handler(uint32_t index, RECEIVE_COMPLETED_HANDLER user_handler)
{
	_uart_receive_completed[index] = user_handler;
}

///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
//		Uart_Register_Err_Interrupt_Handler
// 機能：
//		受信完了を判定するロジックをもった関数を登録する。
// 引数：
//		index : UARTインデックス
//		user_handler : エラー処理関数のポインタ
//
///////////////////////////////////////////////////////////////////////////////
void Uart_Register_Err_Interrupt_Handler(uint32_t index, UART_ERROR_INTERRUPT_HANDLER user_handler)
{
	_uart_err_interrupt[index] = user_handler;
}

///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
//		Uart_Send_Data_Async
// 機能：
//		指定されたバッファからデータを非同期に送信する。
// 引数：
//		index : UARTインデックス
//		buf : 送信バッファポインタ
//		length : 送信データのサイズ
// 戻り値
//		結果 - true - 成功、false - 失敗
//
///////////////////////////////////////////////////////////////////////////////
bool Uart_Send_Data_Async(uint32_t index, uint8_t* buf, uint32_t length)
{
	UART_Type*	uart = uart_base[index];

	if(status[index] != eUART_IDLE && status[index] != eUART_TIMEOUT){
		return false;
	}

	tx_buffer[index] = buf;
	tx_buffer_length[index] = length;
	tx_ptr[index] = 1;
	status[index] = eUART_TX;
	UART_D_REG(uart) = buf[0];
	UART_C2_REG(uart) |= UART_C2_TCIE(1);

	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
//		Uart_Send_Data_Sync
// 機能：
//		指定されたバッファからデータを同期的に送信する。
// 引数：
//		index : UARTインデックス
//		buf : 送信バッファポインタ
//		length : 送信データのサイズ
// 戻り値
//		結果 - true - 成功、false - 失敗
//
///////////////////////////////////////////////////////////////////////////////
bool Uart_Send_Data_Sync(uint32_t index, uint8_t* buf, uint32_t length)
{
	UART_Type*	uart = uart_base[index];
	uint32_t	i;

	for(i = 0; i < length; i++){
		UART_D_REG(uart) = buf[i];
		while(!(UART_S1_REG(uart) & UART_S1_TC_MASK)){
			;
		}
	}

	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
//		Uart_Set_Receive_Buffer
// 機能：
//		受信用バッファを設定する。
// 引数：
//		index : UARTインデックス
//		buf : 受信バッファポインタ
//		length : 受信バッファのサイズ
//
///////////////////////////////////////////////////////////////////////////////
void Uart_Set_Receive_Buffer(uint32_t index, uint8_t* buf, uint32_t length)
{
	rx_buffer[index] = buf;
	rx_buffer_length[index] = length;
	rx_ptr[index] = 0;
}

///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
//		Uart_Receive_Data_Sync
// 機能：
//		UARTからデータを同期的に受信する。
// 引数：
//		index : UARTインデックス
//		buf : 受信バッファポインタ
//		length : 受信バッファのサイズ
// 戻り値:
//		受信データバイト数
//
///////////////////////////////////////////////////////////////////////////////
uint32_t Uart_Receive_Data_Sync(uint32_t index, uint8_t* buf, uint32_t length)
{
	UART_Type*	uart = uart_base[index];
	uint32_t	i;
	uint32_t	tick;
	bool		timeout = false;

	for(i = 0; i < length && timeout == false; i++){
		tick = Timer_Get_Tick();
		while(!(UART_S1_REG(uart) & UART_S1_RDRF_MASK)){
			if(Timer_Get_Tick() - tick > 5){
				timeout = true;
				break;
			}
		}
		if(!timeout){
			buf[i] = UART_D_REG(uart);
		}
	}

	buf[i] = '\0';

	return i;
}

///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
//		Uart_Receive_Data_Count
// 機能：
//		受信したデータ数を返す。
// 引数：
//		index : UARTのインデックス
// 戻り値:
//		受信データバイト数
//
///////////////////////////////////////////////////////////////////////////////
uint32_t Uart_Receive_Data_Count(uint32_t index)
{
	return rx_ptr[index];
}

///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
//		Uart_Receive_Completed
// 機能：
//		受信完了を判定する。
// 引数：
//		index : UARTのインデックス
// 戻り値:
//		true : 完了、false : 未完了
//
///////////////////////////////////////////////////////////////////////////////
//bool Uart_Receive_Completed(uint32_t index)
//{
//	return (status[index] == eUART_IDLE || status[index] == eUART_TIMEOUT);
//}

///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
//		Uart_Transmit_Completed
// 機能：
//		送信完了を判定する。
// 引数：
//		index : UARTのインデックス
// 戻り値:
//		true : 完了、false : 未完了
//
///////////////////////////////////////////////////////////////////////////////
//bool Uart_Transmit_Completed(uint32_t index)
//{
//	return (status[index] != eUART_TX);
//}

///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
//		Uart_Get_Status
// 機能：
//		UARTの状態を返す。
// 引数：
//		index : UARTのインデックス
//
///////////////////////////////////////////////////////////////////////////////
uint32_t Uart_Get_Status(uint32_t index)
{
	return status[index];
}

///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
//		Uart_Timeout_Callback
// 機能：
//		UARTが受信途中で無応答になったとき、状態をeUART_TIMEOUTに遷移する。
// 引数：
//		id : タイマに登録したコールバック関数のインデックス
//		param : UARTのインデックス
//
///////////////////////////////////////////////////////////////////////////////
void Uart_Timeout_Callback(uint32_t id, void* param)
{
	uint32_t	index = (uint32_t)param;
	status[index] = eUART_TIMEOUT;
	timeout_func_installed[index] = false;
}

///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
//		Uart_Start_Timeout_Timer
// 機能：
//		UARTが一定時間無応答になったとき、タイマ割込みからUart_Timeout_Callback呼ばれるように設定する。
// 引数：
//		index : UARTのインデックス
//		timeout : タイムアウトの検出時間(単位msec)
//
///////////////////////////////////////////////////////////////////////////////
void Uart_Start_Timeout_Timer(uint32_t index, uint32_t timeout)
{
	// Register timeout detect function
	timeout_func_id[index] = Timer_Register_Callback(false, UART_RX_TIMEOUT, (void*)index, Uart_Timeout_Callback);
	if(0 <= timeout_func_id[index] && 7 >= timeout_func_id[index]){
		timeout_func_installed[index] = true;
	}
}

///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
//		Uart_Stop_Timeout_Timer
// 機能：
//		タイマ割込みからUart_Timeout_Callbackの登録を外す。
// 引数：
//		index : UARTのインデックス
//
///////////////////////////////////////////////////////////////////////////////
void Uart_Stop_Timeout_Timer(uint32_t index)
{
	if(timeout_func_installed[index]){
		// Unregister timeout detect function
		Timer_Unregister_Callback(timeout_func_id[index]);
		timeout_func_installed[index] = false;
	}
}
// UART Tx Rx Interrupt Handlers
///////////////////////////////////////////////////////////////////////////////

void UART0_RX_TX_IRQHandler(void)
{
	Uart_State_Machine(0);
}

void UART1_RX_TX_IRQHandler(void)
{
	Uart_State_Machine(1);
}

void UART2_RX_TX_IRQHandler(void)
{
	Uart_State_Machine(2);
}

///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
//		Uart_State_Machine
// 機能：
//		状態遷移に基づいて、UARTの送受信を制御する。割込み処理から呼ばれる。
// 引数：
//		index : 0から2までのインデックス
//
///////////////////////////////////////////////////////////////////////////////
void Uart_State_Machine(uint32_t index)
{
	UART_Type*	uart = uart_base[index];
	uint8_t		s1;
	uint32_t	rx_len, tx_len;
	uint32_t	*rxptr, *txptr;
	uint8_t		*rx_buf, *tx_buf;
	uint8_t		dummy;

	rx_len = rx_buffer_length[index];
	tx_len = tx_buffer_length[index];
	rxptr  = &rx_ptr[index];
	txptr  = &tx_ptr[index];
	rx_buf = rx_buffer[index];
	tx_buf = tx_buffer[index];

	// Read UART Status Register S1
	s1 = UART_S1_REG(uart);

	//
	// State Machine Main Body for UART Communication
	//
	switch(status[index]){
	// IDLE or TIMEOUT State
	case eUART_IDLE:
	case eUART_TIMEOUT:
		// Check Rx Interrupt Flag
		if(s1 & UART_S1_RDRF_MASK){
			// Check if Rx buffer is set up
			if(NULL == rx_buf || 0 == rx_len){
				// Remove received data
				dummy = UART_D_REG(uart);

				status[index] = eUART_ERROR;

				// Start timeout timer
				Uart_Start_Timeout_Timer(index, UART_RX_TIMEOUT);
				break;
			}

			// Read data from Rx Buffer
			rx_buf[0] = UART_D_REG(uart);
			*rxptr = 1;
			// Check if Rx Continued
			if(*rxptr < rx_len - 1){
				// Null terminated
				rx_buf[1] = '\0';
				// Change status into Rx State
				status[index] = eUART_RX;
			}
			else{
				*rxptr = rx_len - 1;
				status[index] = eUART_RX_BUF_FULL;
			}
			// Start timeout timer
			Uart_Start_Timeout_Timer(index, UART_RX_TIMEOUT);

		}

		break;
	// Rx State
	case eUART_RX:
		// Stop timeout timer
		Uart_Stop_Timeout_Timer(index);
		if(s1 & UART_S1_RDRF_MASK){
			if(*rxptr < rx_len - 1){
				// Get Data from Data Buffer
				rx_buf[*rxptr] = UART_D_REG(uart);
				++*rxptr;
				// Null terminated
				rx_buf[*rxptr] = '\0';

				// User handler isn't registered
				if(!_uart_receive_completed[index]){
					// Receive until buffer full
					if(*rxptr >= rx_len - 1){
						status[index] = eUART_IDLE;
					}
				}
				// User Handler is registered
				else{
					// Receive Data Completed
					if(_uart_receive_completed[index](rx_buf, *rxptr)){
						status[index] = eUART_IDLE;
					}
				}
			}
			else{
				status[index] = eUART_RX_BUF_FULL;
			}
		}

		// Start timeout timer if status is not IDLE
		if(status[index] != eUART_IDLE){
			Uart_Start_Timeout_Timer(index, UART_RX_TIMEOUT);
		}
		break;
	// Tx State
	case eUART_TX:
		// Check Tx interrupt flag
		if(s1 & UART_S1_TDRE_MASK){
			if(*txptr >= tx_len){
				status[index] = eUART_IDLE;
				// Disable transmit complete interrupt
				UART_C2_REG(uart) &= ~UART_S1_TC_MASK;
			}
			else{
				UART_D_REG(uart) = tx_buf[*txptr];
				++*txptr;
			}
		}
		break;
	case eUART_ERROR:
	case eUART_RX_BUF_FULL:
		// Stop timeout timer
		Uart_Stop_Timeout_Timer(index);
		if(s1 & UART_S1_RDRF_MASK){
			// Remove received data
			dummy = UART_D_REG(uart);
		}
		// Start timeout timer
		Uart_Start_Timeout_Timer(index, UART_RX_TIMEOUT);
		break;
	// Unknown State
	default:
		status[index] = eUART_IDLE;
		break;
	}

	// Clear interrupt flag in NVIC
	NVIC_ClearPendingIRQ(UART0_RX_TX_IRQn + index * 2);
}

///////////////////////////////////////////////////////////////////////////////
// UART Tx Rx Interrupt Handlers
///////////////////////////////////////////////////////////////////////////////

void UART0_ERR_IRQHandler(void)
{
	uint8_t	s1, s2;
	uint32_t	index = 0;
	uint8_t		dummy;

	// Stop timeout timer
	Uart_Stop_Timeout_Timer(index);

	// Read status registers
	s1 = UART0_S1;
	s2 = UART0_S2;

	// Execute custom handler if registered
	if(_uart_err_interrupt[0]){
//		_uart_err_interrupt[0]();
	}

	// Status ERROR
	status[0] = eUART_ERROR;

	if(s1 & UART_S1_RDRF_MASK){
		// Remove received data
		dummy = UART0_D;
	}

	// Start timeout timer
	Uart_Start_Timeout_Timer(index, UART_RX_TIMEOUT);

	// Clear UART error IRQ
	NVIC_ClearPendingIRQ(UART0_ERR_IRQn);
}

void UART1_ERR_IRQHandler(void)
{
	uint8_t	s1, s2;

	if(_uart_err_interrupt[1]){
//		_uart_err_interrupt[1]();
	}

	s1 = UART1_S1;
	s2 = UART1_S2;

	NVIC_ClearPendingIRQ(UART1_ERR_IRQn);

	status[1] = eUART_ERROR;
}

void UART2_ERR_IRQHandler(void)
{
	uint8_t	s1, s2;

	if(_uart_err_interrupt[2]){
//		_uart_err_interrupt[2]();
	}

	s1 = UART2_S1;
	s2 = UART2_S2;

	NVIC_ClearPendingIRQ(UART2_ERR_IRQn);

	status[2] = eUART_ERROR;
}
