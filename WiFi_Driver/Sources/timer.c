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
// 関数名：
// 		Timer_Init
// 機能：
// 		Periodic Interrupt Timerを初期化
// 引数：
// 		なし
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
// 関数名：
// 		Timer_Wait
// 機能：
// 		PIT2を使って、指定された時間(msec単位)だけプログラムの実行を停止する。
// 引数：
// 		msec : 停止時間(msec単位)
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
// 関数名：
// 		Timer_Get_Tick
// 機能：
// 		PIT3を使って、1mseごとにカウントアップされるTick値を返す。
// 引数：
// 		なし
//
///////////////////////////////////////////////////////////////////////////////
uint32_t Timer_Get_Tick(void)
{
	return g_tick_count;
}

///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
// 		Timer_Register_Callback
// 機能：
// 		設定した時間後に実行される関数を登録する。8個まで登録可能。
// 引数：
// 		periodic : false - 1回のみ実行、true - 周期実行
//		set_val : 時間(msec単位)
//		param : 関数に渡すパラメータのポインタ
//		user_function : 実行される関数
// 戻り値：
//		0から7までのID(登録されたテーブルにおけるインデックス)
///////////////////////////////////////////////////////////////////////////////
uint32_t Timer_Register_Callback(bool periodic, uint32_t set_val, void* param, TIMER_CALLBACK user_function)
{
	uint32_t	i;

	for(i = 0; i < sizeof(g_callback_table) / sizeof(TIMER_CALLBACK_INFO); i++){
		// 未登録のエントリーを検出
		if(!g_callback_table[i].active){
			g_callback_table[i].active   = true;
			g_callback_table[i].periodic = periodic;
			g_callback_table[i].counter  = 0;
			g_callback_table[i].set_val  = set_val;
			g_callback_table[i].param    = param;
			g_callback_table[i].callback = user_function;

			// IDをリターン
			return i;
		}
	}

	// 登録できず
	return (-1);
}

///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
// 		Timer_Unregister_Callback
// 機能：
// 		登録した周期実行関数を削除する。
// 引数：
// 		id : 登録時に返されたID
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
