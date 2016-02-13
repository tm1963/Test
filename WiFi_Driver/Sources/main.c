/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//#include "MK12D5.h"
#include	"gw.h"

static int i = 0;

void Initialize(void);
void Red_LED_Brink(uint32_t id);
bool Test_AT_Command(void);

int main(void)
{
	uint32_t	id;
	bool		echo;
	uint8_t		mode;
	AT_CWLIF_OUT	cwlif_out;
	AT_CWDHCP_IN	cwdhcp_in;
	AT_CWDHCPS_IN_OUT	cwdhcps;

    /* Write your code here */
	Initialize();

	// Register Red_LED_Brink function to PIT with 500 msec period
//	id = Timer_Register_Callback(true, 500, Red_LED_Brink);

	// Make sound of buzzer for 5 seconds
//	Buzzer_On(true);
//	Timer_Wait(5000);
//	Buzzer_On(false);

	// AT command
	if(Test_AT_Command()){
		Green_LED_On(true);
	}
	else{
		Red_LED_On(true);
	}

    /* This for loop should be replaced. By default this loop allows a single stepping. */
    for (;;) {
        i++;
    }
    /* Never leave main */
    return 0;
}
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
void Initialize(void)
{
	SystemCoreClockUpdate();

	Port_Init();
	Timer_Init();
	Uart_Config(0, 115200);
	AT_Prepare();
}

void Red_LED_Brink(uint32_t id)
{
	static bool	enabled = false;

	Red_LED_On(enabled);
	enabled ^= 0x1;
}

char	buf1[128];
char	buf2[128];
char	buf3[128];
char	buf4[128];

bool Test_AT_Command(void)
{
	uint32_t		i;
	uint32_t		mode;
	uint32_t		power;
	AT_GMR_OUT		gmr_out;
	AT_CWJAP_IN		cwjap_in;
	AT_CWJAP_OUT	cwjap_out;
	AT_CWLAP_OUT	cwlap_out;
	AT_CWSAP_IN_OUT	cwsap_inout;
	AT_CIPMAC_IN_OUT	cipmac;
	AT_CIPADDR_IN_OUT	cipaddr;

	// AT command
	if(!AT_Exe(NULL, 0, NULL, 0)){
		return false;
	}

	// AT command Reset
	if(!AT_RST_Exe(NULL, 0, NULL, 0)){
		return false;
	}

	gmr_out.at_ver = buf1;
	gmr_out.at_ver_len = sizeof(buf1);
	gmr_out.sdk_ver = buf2;
	gmr_out.sdk_ver_len = sizeof(buf2);
	gmr_out.compile_time = buf3;
	gmr_out.compile_time_len = sizeof(buf3);
	// Version info
	if(!AT_GMR_Exe(NULL, 0, &gmr_out, sizeof(gmr_out))){
		return false;
	}

	// Sleep mode
	if(!AT_SLEEP_Query(NULL, 0, &mode, sizeof(mode))){
		return false;
	}

	// Set sleep mode
	mode = 0;
	if(!AT_SLEEP_Set(&mode, sizeof(mode), NULL, 0)){
		return false;
	}

	// Set RF power
	power = 50;
	if(!AT_RFPOWER_Set(&power, sizeof(power), NULL, 0)){
		return false;
	}

	// Query RF VDD
	if(!AT_RFVDD_Query(NULL, 0, &power, sizeof(power))){
		return false;
	}

	mode = 0;
	if(!AT_CWMODE_CUR_Query(NULL, 0, &mode, sizeof(mode))){
		return false;
	}

	for(i = 0; i < 6; i++){
		cipmac.mac[i] = 0;
	}
	if(!AT_CIPAPMAC_CUR_Query(NULL, 0, &cipmac, sizeof(cipmac))){
		return false;
	}

//	for(i = 0; i < 6; i++){
//		cipmac.mac[i] = 0;
//	}
//	if(!AT_CIPAPMAC_DEF_Query(NULL, 0, &cipmac, sizeof(cipmac))){
//		return false;
//	}

	for(i = 0; i < 6; i++){
		cipmac.mac[i] = 0;
	}
	if(!AT_CIPSTAMAC_CUR_Query(NULL, 0, &cipmac, sizeof(cipmac))){
		return false;
	}

//	for(i = 0; i < 6; i++){
//		cipmac.mac[i] = 0;
//	}
//	if(!AT_CIPSTAMAC_DEF_Query(NULL, 0, &cipmac, sizeof(cipmac))){
//		return false;
//	}

	for(i = 0; i < 4; i++){
		cipaddr.ip_station[i] = 0xcc;
		cipaddr.ip_gateway[i] = 0xcc;
		cipaddr.ip_netmask[i] = 0xcc;
	}
	if(!AT_CIPSTA_CUR_Query(NULL, 0, &cipaddr, sizeof(cipaddr))){
		return false;
	}

	for(i = 0; i < 4; i++){
		cipaddr.ip_station[i] = 0xcc;
		cipaddr.ip_gateway[i] = 0xcc;
		cipaddr.ip_netmask[i] = 0xcc;
	}
	if(!AT_CIPSTA_DEF_Query(NULL, 0, &cipaddr, sizeof(cipaddr))){
		return false;
	}

	for(i = 0; i < 4; i++){
		cipaddr.ip_station[i] = 0xcc;
		cipaddr.ip_gateway[i] = 0xcc;
		cipaddr.ip_netmask[i] = 0xcc;
	}
	if(!AT_CIPAP_CUR_Query(NULL, 0, &cipaddr, sizeof(cipaddr))){
		return false;
	}

	for(i = 0; i < 4; i++){
		cipaddr.ip_station[i] = 0xcc;
		cipaddr.ip_gateway[i] = 0xcc;
		cipaddr.ip_netmask[i] = 0xcc;
	}
	if(!AT_CIPAP_DEF_Query(NULL, 0, &cipaddr, sizeof(cipaddr))){
		return false;
	}

	return true;
}
