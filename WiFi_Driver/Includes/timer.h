/*
 * timer.h
 *
 *  Created on: 2016/01/04
 *      Author: masuda
 */

#ifndef INCLUDES_TIMER_H_
#define INCLUDES_TIMER_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef	void	(*TIMER_CALLBACK)(uint32_t id, void* param);
typedef struct	_TIMER_CALLBACK_INFO{
	bool		active;
	bool		periodic;
	uint32_t	counter;
	uint32_t	set_val;
	void*		param;
	TIMER_CALLBACK	callback;
} TIMER_CALLBACK_INFO;

#define	LDVAL_1_USEC		(SystemCoreClock / 1000000 - 1)
#define	LDVAL_1_MSEC		(SystemCoreClock / 1000 - 1)
#define	LDVAL_1_SEC			(SystemCoreClock - 1)
#define	LDVAL_SEC(sec)		((uint32_t)(sec  * (float)SystemCoreClock) - 1)
#define	LDVAL_MSEC(msec)	((uint32_t)(msec * (float)SystemCoreClock / 1000.0) - 1)
#define	LDVAL_USEC(usec)	((uint32_t)(usec * (float)SystemCoreClock / 1000000.0) - 1)

void Timer_Init(void);
void Timer_Wait(uint32_t msec);
uint32_t Timer_Get_Tick(void);
uint32_t Timer_Register_Callback(bool periodic, uint32_t set_val, void* param, TIMER_CALLBACK user_function);
void Timer_Unregister_Callback(uint32_t id);

#ifdef _cplusplus
}
#endif

#endif /* INCLUDES_TIMER_H_ */
