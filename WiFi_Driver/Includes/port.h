/*
 * port.h
 *
 *  Created on: 2016/01/03
 *      Author: masuda
 */

#ifndef INCLUDES_PORT_H_
#define INCLUDES_PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#define	BLE_WAKE_HW_SHIFT		5
#define	BLE_WAKE_SW_SHIFT		6
#define	BLE_CMD_SHIFT			7

#define	VOICE_S1_STB_SHIFT		0
#define	VOICE_S1_RDY_SHIFT		1
#define	FLASH_S1F_WPn_SHIFT		4
#define	BLE_WS_SHIFT			5
#define	BLE_MLDP_EV_SHIFT		6
#define	BLE_CON_STS_SHIFT		7

#define	BUZZER_PORT_SHIFT		1
#define	GREEN_LED_PORT_SHIFT	18
#define	RED_LED_PORT_SHIFT		19

#define	BLE_WAKE_HW_MASK		(1 << BLE_WAKE_HW_SHIFT)
#define	BLE_WAKE_SW_MASK		(1 << BLE_WAKE_SW_SHIFT)
#define	BLE_CMD_MASK			(1 << BLE_CMD_SHIFT)

#define	VOICE_S1_STB_MASK		(1 << VOICE_S1_STB_SHIFT)
#define	VOICE_S1_RDY_MASK		(1 << VOICE_S1_RDY_SHIFT)
#define	FLASH_S1F_WPn_MASK		(1 << FLASH_S1F_WPn_SHIFT)
#define	BLE_WS_MASK				(1 << BLE_WS_SHIFT)
#define	BLE_MLDP_EV_MASK		(1 << BLE_MLDP_EV_SHIFT)
#define	BLE_CON_STS_MASK		(1 << BLE_CON_STS_SHIFT)

#define	BUZZER_PORT_MASK		(1 << BUZZER_PORT_SHIFT)
#define	GREEN_LED_PORT_MASK		(1 << GREEN_LED_PORT_SHIFT)
#define	RED_LED_PORT_MASK		(1 << RED_LED_PORT_SHIFT)

typedef enum{
	LED_Red = 1,
	LED_Green,
	LED_Both
} LED_SEL;

void Port_Init(void);
void Buzzer_On(bool bEnabled);
void Toggle_Buzzer(void);
void Red_LED_On(bool bEnabled);
void Toggle_Red_LED(void);
void Green_LED_On(bool bEnabled);
void Toggle_Green_LED(void);
bool Blink_LED(bool start, LED_SEL led, uint32_t msec);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDES_PORT_H_ */
