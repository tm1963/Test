/*
 * port.c
 *
 *  Created on: 2016/01/03
 *      Author: masuda
 */
#include	"gw.h"

static bool	led_callback_installed = false;
static uint32_t	timer_id;

static void LED_Blink_Callback(uint32_t id, void* param);

///////////////////////////////////////////////////////////////////////////////
//
// �֐����F
// 		Port_Init
// �@�\�F
// 		PORTA����PORTE�܂ł𐻕i�̎d�l�ɍ��킹�ď���������B
// �����F
// 		�Ȃ�
//
///////////////////////////////////////////////////////////////////////////////
void Port_Init(void)
{
	SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK |
			      SIM_SCGC5_PORTB_MASK |
				  SIM_SCGC5_PORTC_MASK |
				  SIM_SCGC5_PORTD_MASK |
				  SIM_SCGC5_PORTE_MASK);

	// PortA Initialized
	PORTA_PCR0 = ((PORTA_PCR0 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(7));	// ALT7 JTAG_TCLK
	PORTA_PCR1 = ((PORTA_PCR1 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(7));	// ALT7 JTAG_TDI
	PORTA_PCR2 = ((PORTA_PCR2 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(7));	// ALT7 JTAG_TDO
	PORTA_PCR3 = ((PORTA_PCR3 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(7));	// ALT7 JTAG_TMS
	PORTA_PCR4 = ((PORTA_PCR4 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0));	// Disabled
	PORTA_PCR5 = ((PORTA_PCR5 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0));	// Disabled
	PORTA_PCR12 = PORT_PCR_MUX(0);	// Disabled
	PORTA_PCR13 = PORT_PCR_MUX(0);	// Disabled
	PORTA_PCR18 = PORT_PCR_MUX(0);	// Disabled
	PORTA_PCR19 = PORT_PCR_MUX(0);	// Disabled

	// PortB Initialized
	PORTB_PCR0 = PORT_PCR_MUX(0);	// Disabled
	PORTB_PCR1 = PORT_PCR_MUX(0);	// Disabled
	PORTB_PCR2 = PORT_PCR_MUX(3);	// ALT3 UART0_RTS
	PORTB_PCR3 = PORT_PCR_MUX(3);	// ALT3 UART0_CTS
	PORTB_PCR16 = PORT_PCR_MUX(3);	// ALT3 UART0_Rx
	PORTB_PCR17 = PORT_PCR_MUX(3);	// ALT3 UART0_Tx
	PORTB_PCR18 = PORT_PCR_MUX(0);	// Disabled
	PORTB_PCR19 = PORT_PCR_MUX(0);	// Disabled

	// PortC Initialized
	PORTC_PCR0 = PORT_PCR_MUX(0);	// Disabled
	PORTC_PCR1 = PORT_PCR_MUX(3);	// ALT3 UART1_RTS
	PORTC_PCR2 = PORT_PCR_MUX(3);	// ALT3 UART1_CTS
	PORTC_PCR3 = PORT_PCR_MUX(3);	// ALT3 UART1_Rx
	PORTC_PCR4 = PORT_PCR_MUX(3);	// ALT3 UART1_Tx
	PORTC_PCR5 = PORT_PCR_MUX(1);	// ALT1 WAKE_HW
	PORTC_PCR6 = PORT_PCR_MUX(1);	// ALT1 WAKE_SW
	PORTC_PCR7 = PORT_PCR_MUX(1);	// ALT1 WAKE_CMD
	PORTC_PCR8 = PORT_PCR_MUX(0);	// Disabled
	PORTC_PCR9 = PORT_PCR_MUX(0);	// Disabled
	PORTC_PCR10 = PORT_PCR_MUX(0);	// Disabled
	PORTC_PCR11 = PORT_PCR_MUX(0);	// Disabled
	GPIOC_PDDR = (BLE_WAKE_HW_MASK | BLE_WAKE_SW_MASK | BLE_CMD_MASK);

	// PortD Initialized
	PORTD_PCR0 = PORT_PCR_MUX(1);	// ALT1 S1_STB
	PORTD_PCR1 = PORT_PCR_MUX(1);	// ALT1 S1_RDY
	PORTD_PCR2 = PORT_PCR_MUX(2);	// ALT2 SPI0_SOUT
	PORTD_PCR3 = PORT_PCR_MUX(2);	// ALT2 SPI0_SIN
	PORTD_PCR4 = PORT_PCR_MUX(1);	// ALT1 S1F_WPn
	PORTD_PCR5 = PORT_PCR_MUX(1);	// ALT1 WS
	PORTD_PCR6 = PORT_PCR_MUX(1);	// ALT1 MLDP_EV
	PORTD_PCR7 = PORT_PCR_MUX(1);	// ALT1 CON_STS
	GPIOD_PDDR = (VOICE_S1_RDY_MASK | FLASH_S1F_WPn_MASK);

	// PortE Initialized
	PORTE_PCR0 = PORT_PCR_MUX(1);	// ALT1 POWER_ONn
	PORTE_PCR1 = PORT_PCR_MUX(1);	// ALT1 BUZZER_ON
	PORTE_PCR16 = PORT_PCR_MUX(2);	// ALT2 SPI0_PCS
	PORTE_PCR17 = PORT_PCR_MUX(2);	// ALT2 SPI0_SCK
	PORTE_PCR18 = PORT_PCR_MUX(1);	// ALT1 GLED_ON
	PORTE_PCR19 = PORT_PCR_MUX(1);	// ALT1 RLED_ON
	GPIOE_PDDR = (BUZZER_PORT_MASK | GREEN_LED_PORT_MASK | RED_LED_PORT_MASK);
}

///////////////////////////////////////////////////////////////////////////////
//
// �֐����F
// 		Buzzer_On
// �@�\�F
// 		Buzzer��On/Off�𐧌䂷��B
// �����F
// 		bEnabled - true : Buzzer On�Afalse : Buzzer off
//
///////////////////////////////////////////////////////////////////////////////
void Buzzer_On(bool bEnabled)
{
	if(bEnabled){
		GPIOE_PSOR = BUZZER_PORT_MASK;
	}
	else{
		GPIOE_PCOR = BUZZER_PORT_MASK;
	}
}

///////////////////////////////////////////////////////////////////////////////
//
// �֐����F
// 		Toggle_Buzzer
// �@�\�F
// 		Buzzer��On/Off�𔽓]������B
// �����F
// 		�Ȃ�
//
///////////////////////////////////////////////////////////////////////////////
void Toggle_Buzzer(void)
{
	GPIOE_PTOR = BUZZER_PORT_MASK;
}

///////////////////////////////////////////////////////////////////////////////
//
// �֐����F
// 		Red_LED_On
// �@�\�F
// 		Red LED��On/Off�𐧌䂷��B
// �����F
// 		bEnabled - true : LED On�Afalse : LED off
//
///////////////////////////////////////////////////////////////////////////////
void Red_LED_On(bool bEnabled)
{
	if(bEnabled){
		GPIOE_PSOR = RED_LED_PORT_MASK;
	}
	else{
		GPIOE_PCOR = RED_LED_PORT_MASK;
	}
}

///////////////////////////////////////////////////////////////////////////////
//
// �֐����F
// 		Toggle_Red_LED
// �@�\�F
// 		Red LED��On/Off�𔽓]������B
// �����F
// 		�Ȃ�
//
///////////////////////////////////////////////////////////////////////////////
void Toggle_Red_LED(void)
{
	GPIOE_PTOR = RED_LED_PORT_MASK;
}

///////////////////////////////////////////////////////////////////////////////
//
// �֐����F
// 		Green_LED_On
// �@�\�F
// 		Green LED��On/Off�𐧌䂷��B
// �����F
// 		bEnabled - true : LED On�Afalse : LED off
//
///////////////////////////////////////////////////////////////////////////////
void Green_LED_On(bool bEnabled)
{
	if(bEnabled){
		GPIOE_PSOR = GREEN_LED_PORT_MASK;
	}
	else{
		GPIOE_PCOR = GREEN_LED_PORT_MASK;
	}
}

///////////////////////////////////////////////////////////////////////////////
//
// �֐����F
// 		Toggle_Green_LED
// �@�\�F
// 		Green LED��On/Off�𔽓]������B
// �����F
// 		�Ȃ�
//
///////////////////////////////////////////////////////////////////////////////
void Toggle_Green_LED(void)
{
	GPIOE_PTOR = GREEN_LED_PORT_MASK;
}

///////////////////////////////////////////////////////////////////////////////
//
// �֐����F
// 		LED_Blink_Callback
// �@�\�F
// 		LED��_�ł𐧌䂷��R�[���o�b�N�֐��ŁA�^�C�}�����݂��R�[�������B
// �����F
// 		id - �o�^���Ɏ擾����ID
//		param - �I������LED
//
///////////////////////////////////////////////////////////////////////////////
void LED_Blink_Callback(uint32_t id, void* param)
{
	uint32_t	led = (uint32_t)param;

	switch(led){
	case LED_Red:
		GPIOE_PTOR = RED_LED_PORT_MASK;
		break;
	case LED_Green:
		GPIOE_PTOR = GREEN_LED_PORT_MASK;
		break;
	case LED_Both:
		GPIOE_PTOR = (RED_LED_PORT_MASK | GREEN_LED_PORT_MASK);
		break;
	}
}

///////////////////////////////////////////////////////////////////////////////
//
// �֐����F
// 		Blink_LED
// �@�\�F
// 		LED��_�ł�����B
// �����F
// 		start - true : �J�n�Afalse : ��~
//		led - �_�ł�����LED
//		msec - On��Off�̊Ԋumsec�P��
// �߂�l
//		true : �����Afalse : ���s
//
///////////////////////////////////////////////////////////////////////////////
bool Blink_LED(bool start, LED_SEL led, uint32_t msec)
{
	uint32_t	id;

	if(!start && led_callback_installed){
		Timer_Unregister_Callback(timer_id);
		led_callback_installed = false;
		return true;
	}

	id = Timer_Register_Callback(true, msec, (void*)led, LED_Blink_Callback);
	if(id > 7){
		return false;
	}

	led_callback_installed = true;
	timer_id = id;

	return true;
}
