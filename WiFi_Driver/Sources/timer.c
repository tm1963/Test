/*
 * timer.c
 *
 *  Created on: 2016/01/04
 *      Author: masuda
 */

#include	"gw.h"

static uint32_t	g_tick_count = 0;
static TIMER_CALLBACK_INFO	g_callback_table[] = {
		{false, false, 0, 0, NULL},
		{false, false, 0, 0, NULL},
		{false, false, 0, 0, NULL},
		{false, false, 0, 0, NULL},
		{false, false, 0, 0, NULL},
		{false, false, 0, 0, NULL},
		{false, false, 0, 0, NULL},
		{false, false, 0, 0, NULL}
};

///////////////////////////////////////////////////////////////////////////////
//
// �֐����F
// 		Timer_Init
// �@�\�F
// 		Periodic Interrupt Timer��������
// �����F
// 		�Ȃ�
//
///////////////////////////////////////////////////////////////////////////////
void Timer_Init(void)
{
	// NVIC Initialized
	NVIC_EnableIRQ(PIT0_IRQn);
	NVIC_EnableIRQ(PIT1_IRQn);

	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
	// 1msec
	PIT_LDVAL0 = LDVAL_MSEC(1);

	// PIT0 Initialized
	PIT_MCR = PIT_MCR_FRZ(1) | PIT_MCR_MDIS(0);
	PIT_TCTRL0 = PIT_TCTRL_TEN(1) | PIT_TCTRL_TIE(1) | PIT_TCTRL_CHN(0);
//	PIT_TCTRL1 = PIT_TCTRL_TEN(1) | PIT_TCTRL_TIE(0) | PIT_TCTRL_CHN(0);
//	PIT_TCTRL2 = PIT_TCTRL_TEN(1) | PIT_TCTRL_TIE(0) | PIT_TCTRL_CHN(0);
//	PIT_TCTRL3 = PIT_TCTRL_TEN(1) | PIT_TCTRL_TIE(0) | PIT_TCTRL_CHN(0);
}

///////////////////////////////////////////////////////////////////////////////
//
// �֐����F
// 		Timer_Wait
// �@�\�F
// 		PIT2���g���āA�w�肳�ꂽ����(msec�P��)�����v���O�����̎��s���~����B
// �����F
// 		msec : ��~����(msec�P��)
//
///////////////////////////////////////////////////////////////////////////////
void Timer_Wait(uint32_t msec)
{
	PIT_LDVAL2 = LDVAL_MSEC(msec);
	PIT_TCTRL2 = PIT_TCTRL_TEN(1);
	while((PIT_TFLG2 & PIT_TFLG_TIF_MASK) == 0){
		;
	}
	PIT_TFLG2 &= PIT_TFLG_TIF_MASK;
	PIT_TCTRL2 = 0;
}

///////////////////////////////////////////////////////////////////////////////
//
// �֐����F
// 		Timer_Get_Tick
// �@�\�F
// 		PIT3���g���āA1mse���ƂɃJ�E���g�A�b�v�����Tick�l��Ԃ��B
// �����F
// 		�Ȃ�
//
///////////////////////////////////////////////////////////////////////////////
uint32_t Timer_Get_Tick(void)
{
	return g_tick_count;
}

///////////////////////////////////////////////////////////////////////////////
//
// �֐����F
// 		Timer_Register_Callback
// �@�\�F
// 		�ݒ肵�����Ԍ�Ɏ��s�����֐���o�^����B8�܂œo�^�\�B
// �����F
// 		periodic : false - 1��̂ݎ��s�Atrue - �������s
//		set_val : ����(msec�P��)
//		param : �֐��ɓn���p�����[�^�̃|�C���^
//		user_function : ���s�����֐�
// �߂�l�F
//		0����7�܂ł�ID(�o�^���ꂽ�e�[�u���ɂ�����C���f�b�N�X)
///////////////////////////////////////////////////////////////////////////////
uint32_t Timer_Register_Callback(bool periodic, uint32_t set_val, void* param, TIMER_CALLBACK user_function)
{
	uint32_t	i;

	for(i = 0; i < sizeof(g_callback_table) / sizeof(TIMER_CALLBACK_INFO); i++){
		// ���o�^�̃G���g���[�����o
		if(!g_callback_table[i].active){
			g_callback_table[i].active   = true;
			g_callback_table[i].periodic = periodic;
			g_callback_table[i].counter  = 0;
			g_callback_table[i].set_val  = set_val;
			g_callback_table[i].param    = param;
			g_callback_table[i].callback = user_function;

			// ID�����^�[��
			return i;
		}
	}

	// �o�^�ł���
	return (-1);
}

///////////////////////////////////////////////////////////////////////////////
//
// �֐����F
// 		Timer_Unregister_Callback
// �@�\�F
// 		�o�^�����������s�֐����폜����B
// �����F
// 		id : �o�^���ɕԂ��ꂽID
//
///////////////////////////////////////////////////////////////////////////////
void Timer_Unregister_Callback(uint32_t id)
{
	if(id < sizeof(g_callback_table) / sizeof(TIMER_CALLBACK_INFO)){
		g_callback_table[id].active = false;
	}
}
///////////////////////////////////////////////////////////////////////////////
// IRQ Handlers for PIT's
///////////////////////////////////////////////////////////////////////////////

//
// PIT0 is set for a periodic timer in this sample program.
//
void PIT0_IRQHandler(void)
{
	uint32_t	i;

	// Clear Timer Flag Register
	PIT_TFLG(0) = 0x1;
	// Clear Pending IRQ bit of NVIC
	NVIC_ClearPendingIRQ(PIT0_IRQn);

	for(i = 0; i < sizeof(g_callback_table) / sizeof(TIMER_CALLBACK_INFO); i++){
		if(!g_callback_table[i].active){
			continue;
		}

		// Increment 1msec counter
		g_callback_table[i].counter++;
		if(g_callback_table[i].counter >= g_callback_table[i].set_val){
			// Execute user callback function
			g_callback_table[i].callback(i, g_callback_table[i].param);
			// Clear 1msec counter for this
			g_callback_table[i].counter = 0;

			// Periodic or one time execution?
			if(!g_callback_table[i].periodic){
				// Deactivate
				g_callback_table[i].active = false;
			}

		}
	}

	// Increment tick counter
	g_tick_count++;
}

//
// PIT1 is set for an alarm timer in this sample program.
//
//void PIT1_IRQHandler(void)
//{
//
//}
//
//void PIT2_IRQHandler(void)
//{
//
//}
//
//void PIT3_IRQHandler(void)
//{
//
//}
