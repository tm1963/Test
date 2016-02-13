/*
 * esp8266_at_command.c
 *
 *  Created on: 2016/01/11
 *      Author: masuda
 */

#include	"gw.h"
#include	"esp8266_at_command.h"

static char		cmd_req[128];
static char		cmd_param[128];
static uint8_t	cmd_resp[2048];
static uint32_t	resp_len = 0;
static uint32_t	delimiter_id = DLMT_Normal;
static bool		multi_conn = false;
static bool		show_remote_info = false;
static bool		send_mode = false;
static volatile bool	at_completed = false;
static volatile bool	ipd_received = false;

///////////////////////////////////////////////////////////////////////////////
//
// AT Command Table
//
///////////////////////////////////////////////////////////////////////////////
static const char*	cmd_string[] = {
		// AT Command
		"AT",				// 0 : Test AT startup
		"AT+RST",			// 1 : Restart module
		"AT+GMR",			// 2 : View version info
		"AT+GSLP",			// 3 : Enter deep-sleep mode
		"ATE0",				// 4 : AT commands echo off
		"ATE1",				// 5 : AT commands echo on
		"AT+RESTORE",		// 6 : Factory Reset
		"AT+UART_CUR",		// 7 : UART current configuration
		"AT+UART_DEF",		// 8 : UART default configuration, save to flash
		"AT+SLEEP",			// 9 : Sleep mode
		"AT+RFPOWER",		// 10 : Set maximum value of RF TX Power
		"AT+RFVDD",			// 11 : Set RF TX Power according to VDD33
		// AT WiFi Command
		"AT+CWMODE_CUR",	// 12 : Wi-Fi mode（sta/AP/sta+AP）
		"AT+CWMODE_DEF",	// 13 : Wi-Fi default mode（sta/AP/sta+AP）. Save to Flash
		"AT+CWJAP_CUR",		// 14 : Connect to AP, won’t save to Flash
		"AT+CWJAP_DEF",		// 15 : Connect to AP, save to Flash
		"AT+CWLAP",			// 16 : Lists available APs
		"AT+CWQAP",			// 17 : Disconnect from AP
		"AT+CWSAP_CUR",		// 18 : Set configuration of ESP8266 soft-AP, won’t save to Flash
		"AT+CWSAP_DEF",		// 19 : Set configuration of ESP8266 soft-AP, save to Flash
		"AT+CWLIF",			// 20 : Get station’s IP which is connected to ESP8266 soft-AP
		"AT+CWDHCP_CUR",	// 21 : Enable/Disable DHCP, won’t save to Flash
		"AT+CWDHCP_DEF",	// 22 : Enable/Disable DHCP, save to Flash
		"AT+CWDHCPS_CUR",	// 23 : Set IP range of DHCP server, won’t save to Flash
		"AT+CWDHCPS_DEF",	// 24 : Set IP range of DHCP server, save to Flash
		"AT+CWAUTOCONN",	// 25 : Connect to AP automatically when power on
		"AT+CIPSTAMAC_CUR",	// 26 : Set MAC address of ESP8266 station, won’t save to Flash
		"AT+CIPSTAMAC_DEF",	// 27 : Set MAC address of ESP8266 station, save to Flash
		"AT+CIPAPMAC_CUR",	// 28 : Set MAC address of ESP8266 soft-AP, won’t save to Flash
		"AT+CIPAPMAC_DEF",	// 29 : Set MAC address of ESP8266 soft-AP, save to Flash
		"AT+CIPSTA_CUR",	// 30 : Set IP address of ESP8266 station, won’t save to Flash
		"AT+CIPSTA_DEF",	// 31 : Set IP address of ESP8266 station, save to Flash
		"AT+CIPAP_CUR",		// 32 : Set IP address of ESP8266 soft-AP, won’t save to Flash
		"AT+CIPAP_DEF",		// 33 : Set IP address of ESP8266 soft-AP, save to Flash
		"AT+STARTSMART",	// 34 : Start SmartConfig
		"AT+STOPSMART",		// 35 : Stop SmartConfig
		// AT TCP/IP Command
		"AT+CIPSTATUS",		// 36 : Get connection status
		"AT+CIPSTART",		// 37 : Establish TCP connection or register UDP port
		"AT+CIPSEND",		// 38 : Send data
		"AT+CIPSENDEX",		// 39 : Send data, if <length> or “\0” is met, data will be sent
		"AT+CIPSENDBUF",	// 40 : Write data into TCP-send-buffer
		"AT+CIPBUFRESET",	// 41 : Reset segment ID count
		"AT+CIPBUFSTATUS",	// 42 : Check status of TCP-send-buffer
		"AT+CIPCHECKSEQ",	// 43 : Check if a specific segment is sent or not
		"AT+CIPCLOSE",		// 44 : Close TCP/UDP connection
		"AT+CIFSR",			// 45 : Get local IP address
		"AT+CIPMUX",		// 46 : Set multiple connections mode
		"AT+CIPSERVER",		// 47 : Configure as server
		"AT+CIPMODE",		// 48 : Set transmission mode
		"AT+SAVETRANSLINK",	// 49 : Save transparent transmission link to Flash
		"AT+CIPSTO",		// 50 : Set timeout when ESP8266 runs as TCP server
		"AT+CIUPDATE",		// 51 : Upgrade firmware through network
		"AT+PING",			// 52 : Function PING
		"AT+CIPDINFO",		// 53 : Show remote IP and remote port with “+IPD”
};

static const char*	delimiter_str[][3] = {
		{"OK\r\n", "FAIL\r\n", "ERROR\r\n"},
		{"OK\r\n", "ERROR\r\n", "ALREADY CONNECT\r\n"},
		{"OK\r\n", "Link is builded\r\n", NULL},
		{">", NULL, NULL},
		{"SEND OK\r\n", NULL, NULL}
};

///////////////////////////////////////////////////////////////////////////////
//
// Receive Completed Callback Functions Prototype
// These are called from UART interrupt handler
//
///////////////////////////////////////////////////////////////////////////////
static uint32_t At_Command(uint32_t cmd_id, AT_CMD_TYPE cmd_type);
static bool IsOK(char* string);
static bool AT_Get_Param(char* src, uint32_t index, char* dst, uint32_t dst_size);
static bool IPD_Get_Param(char* src, uint32_t index, char* dst, uint32_t dst_size);
static void AT_Set_Delimiter(DLMT_STR_TYPE id);
static bool Is_AT_Completed(void);
static bool AT_Receive_Completed(char* buffer, uint32_t length);
static bool IPD_Receive_Completed(char* buffer, uint32_t length);
static bool _Receive_Completed(char* buffer, uint32_t length);
static bool IPD_Get_Param(char* src, uint32_t index, char* dst, uint32_t dst_size);

void AT_Prepare(void)
{
	// Clear response buffer
	memset(cmd_resp, 0, sizeof(cmd_resp));

	// Register AT Command Receive Completed Handler(It will be called from UART interrupt handler.)
	Uart_Register_Receive_Completed_Handler(0, _Receive_Completed);
}

///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
//		At_Command
// 機能：
//		指定されたIDとタイプに合致するESP8266のATコマンドを実行する。
//　引数：
//		cmd_id : ATコマンドテーブルにおけるインデックス
//		cdm_type : Test, Query, Set, Executeのどれかを示すコマンドタイプ
// 戻り値
//		受信データサイズ
//
///////////////////////////////////////////////////////////////////////////////
#if 0
uint32_t At_Command(uint32_t cmd_id, AT_CMD_TYPE cmd_type)
{
	uint32_t	length;
	char**		delimiter;
	uint32_t	num;

	strcpy(cmd_req, cmd_string[cmd_id]);

	//
	// Make command string by command type
	//
	switch(cmd_type){
	case Cmd_Test:
		strcat(cmd_req, "=?\r\n");
		break;
	case Cmd_Query:
		strcat(cmd_req, "?\r\n");
		break;
	case Cmd_Set:
		strcat(cmd_req, "=");
		strcat(cmd_req, cmd_param);
		strcat(cmd_req, "\r\n");
		break;
	case Cmd_Execute:
		strcat(cmd_req, "\r\n");
		break;
	default:
		break;
	}

	// Register AT Command Receive Completed Handler(It will be called from UART interrupt handler.)
	Uart_Register_Receive_Completed_Handler(0, _Receive_Completed);

	// Set receive buffer
	Uart_Set_Receive_Buffer(UART_INDEX, cmd_resp, sizeof(cmd_resp));

	// Send data asynchronously
	Uart_Send_Data_Sync(UART_INDEX, cmd_req, strlen(cmd_req));

	length = Uart_Receive_Data_Sync(UART_INDEX, cmd_resp, sizeof(cmd_resp));

	return length;
}
#else
uint32_t At_Command(uint32_t cmd_id, AT_CMD_TYPE cmd_type)
{
	char**		delimiter;
	uint32_t	num;
	uint32_t	status;

	// Check if send mode
	if(send_mode){
		return false;
	}

	strcpy(cmd_req, cmd_string[cmd_id]);

	//
	// Make command string by command type
	//
	switch(cmd_type){
	case Cmd_Test:
		strcat(cmd_req, "=?\r\n");
		break;
	case Cmd_Query:
		strcat(cmd_req, "?\r\n");
		break;
	case Cmd_Set:
		strcat(cmd_req, "=");
		strcat(cmd_req, cmd_param);
		strcat(cmd_req, "\r\n");
		break;
	case Cmd_Execute:
		strcat(cmd_req, "\r\n");
		break;
	default:
		break;
	}

	// Receive buffer clear
	cmd_resp[0] = '\0';
	// Set receive buffer
	Uart_Set_Receive_Buffer(UART_INDEX, cmd_resp, sizeof(cmd_resp));

	// Send data asynchronously
	if(!Uart_Send_Data_Async(UART_INDEX, cmd_req, strlen(cmd_req))){
		return false;
	}

	// Wait until receiving response from ESP8266 completed
	while(!Is_AT_Completed()){
		continue;
	}

	// Get received data count
	resp_len = Uart_Receive_Data_Count(UART_INDEX);

	// Reset receive buffer
	Uart_Set_Receive_Buffer(UART_INDEX, NULL, 0);

	return resp_len;
}
#endif
///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
//		IsOK
// 機能：
//		指定された文字列に"OK"が含まれるかどうかを示す。
//　引数：
//		string : 検査する文字列
// 戻り値
//		true - "OK"を検出、false - 適合せず
//
///////////////////////////////////////////////////////////////////////////////
bool IsOK(char* string)
{
	if(NULL == strstr(string, "OK")){
		return false;
	}

	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
//		AT_Get_Param
// 機能：
//		受信バッファに含まれるパラメータの値を抽出
//　引数：
//		src : 検査する文字列
//		index - 文字列に含まれるパラメータのインデックス
//		dst - パラメータを文字列のまま格納するバッファ
//		dst_size - dstのバッファサイズ
// 戻り値
//		true - 成功、false - 失敗
//
///////////////////////////////////////////////////////////////////////////////
bool AT_Get_Param(char* src, uint32_t index, char* dst, uint32_t dst_size)
{
	uint32_t	i;
	char*	ptr;
	char*	next;

	// Parameter starts after ":"
	ptr = strchr(src, ':');
	if(NULL == ptr){
		ptr = src;
	}

	ptr++;
	if(index > 0){
		for(i = 0; i < index; i++){
			// Parameter starts after ","
			ptr = strchr(ptr, ',');
			if(NULL == ptr){
				return false;
			}
			ptr++;
		}
	}

	// Skip first double quotation and space
	while(*ptr == '\"' || *ptr == ' '){
		ptr++;
	}

	// Get next delimiter, ',' or '\r'
	next = strchr(ptr, ',');
	if(!next){
		next = strchr(ptr, '\r');
		if(!next){
			return false;
		}
	}

	i = 0;
	while(ptr != next && *ptr != '\"'){
		dst[i++] = *ptr++;
		if(dst_size < i){
			return false;
		}
	}

	// Remove space from the end of string
	while(dst[i - 1] == ' '){
		i--;
	}

	// Null terminated
	dst[i] = '\0';

	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
//		IPD_Get_Param
// 機能：
//		IPDコマンドの受信データに含まれるパラメータの値を抽出
//　引数：
//		src : 検査する文字列
//		index - 文字列に含まれるパラメータのインデックス
//		dst - パラメータを文字列のまま格納するバッファ
//		dst_size - dstのバッファサイズ
// 戻り値
//		true - 成功、false - 失敗
//
///////////////////////////////////////////////////////////////////////////////
bool IPD_Get_Param(char* src, uint32_t index, char* dst, uint32_t dst_size)
{
	uint32_t	i;
	uint32_t	length;
	char*		ptr;
	char*		next;

	// index is less than or equal to 3
	if(index > 3){
		return false;
	}

	// First string is "+IPD,", so search for comma
	ptr = strchr(src, ',');
	ptr++;

	for(i = 0; i < index; i++){
		ptr = strchr(ptr, ',');
		ptr++;
	}

	// Search for the delimiter pointer next to the data we want to get
	next = strchr(ptr, ',');
	if(NULL == next){
		next = strchr(ptr, ':');
		if(NULL == next){
			return false;
		}
	}

	// next is always greater than ptr
	if((uint32_t)next <= (uint32_t)ptr){
		return false;
	}

	// Get data length
	length = (uint32_t)(next - ptr);
	// Check if data buffer has enough size
	if(length > dst_size){
		return false;
	}

	// Skip first double quotation and space
	while(*ptr == '\"' || *ptr == ' '){
		ptr++;
	}

	i = 0;
	while(ptr != next && *ptr != '\"'){
		dst[i++] = *ptr++;
	}

	// Remove space from the end of string
	while(dst[i - 1] == ' '){
		i--;
	}

	// Null terminated
	dst[i] = '\0';

	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
//		AT_Set_Delimiter
// 機能：
//		コマンドが返す最後の文字列を設定する。
//　引数：
//		id : デリミタ文字列を示すインデックス
//
///////////////////////////////////////////////////////////////////////////////
void AT_Set_Delimiter(DLMT_STR_TYPE id)
{
	delimiter_id = id;
}

///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
//		Is_AT_Completed
// 機能：
//		ATコマンドが完了したかどうかを示す。
//　引数：
//		なし
// 戻り値
//		true : 完了、false : 未完了
//
///////////////////////////////////////////////////////////////////////////////
bool Is_AT_Completed(void)
{
	if(at_completed){
		at_completed = false;
		return true;
	}
	else{
		return false;
	}
}

///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
//		Is_IPD_Received
// 機能：
//		IPDコマンドを受信したかどうかを示す。
//　引数：
//		なし
// 戻り値
//		true : 受信、false : 未受信
//
///////////////////////////////////////////////////////////////////////////////
bool Is_IPD_Received(void)
{
	if(ipd_received){
		ipd_received = false;
		return true;
	}
	else{
		return false;
	}
}

///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
//		AT_Receive_Completed
// 機能：
//		ATコマンドの受信完了を判定する。
//　引数：
//		buffer : 受信バッファのポインタ
//		length - 受信データ長
// 戻り値
//		true - 受信完了、false - 受信継続
//
///////////////////////////////////////////////////////////////////////////////
bool AT_Receive_Completed(char* buffer, uint32_t length)
{
	uint32_t	i, j;
	uint32_t	cnt;
	const char*	delimiter;
	bool		flag;

	for(i = 0; i < 3 && NULL != delimiter_str[delimiter_id][i]; i++){
		delimiter = delimiter_str[delimiter_id][i];
		cnt = strlen(delimiter);
		flag = true;
		for(j = 0; j < cnt; j++){
			if(buffer[length - cnt + j] != delimiter[j]){
				flag = false;
				break;
			}
		}
		if(flag){
			break;
		}
	}

	return flag;
}

///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
//		IPD_Receive_Completed
// 機能：
//		ESP8266からのIPDコマンドの受信完了を判定する。
//　引数：
//		buffer : 受信バッファのポインタ
//		length - 受信データ長
// 戻り値
//		true - 受信完了、false - 受信継続
//
///////////////////////////////////////////////////////////////////////////////
#if 0
bool IPD_Receive_Completed(char* buffer, uint32_t length)
{
	uint32_t	i;
	uint32_t	datalen;
	uint32_t	id;
	uint32_t	port;
	char*		ptr;

	ptr = strchr(buffer, ':');
	if(!ptr){
		return false;
	}
	ptr++;

	// Multiple connection
	if(multi_conn){
		// Show remote information
		if(show_remote_info){
			sscanf(buffer, "+IPD,%d,%d,", &id, &datalen);
		}
		// Doesn't show remote information
		else{
			sscanf(buffer, "+IPD,%d,%d:", &id, &datalen);
		}
	}
	// Single connection
	else{
		// Show remote information
		if(show_remote_info){
			sscanf(buffer, "+IPD,%d,", &datalen);
		}
		// Doesn't show remote information
		else{
			sscanf(buffer, "+IPD,%d:", &datalen);
		}
	}

	datalen += (uint32_t)(ptr - buffer);

	return (length >= datalen);
}
#else
bool IPD_Receive_Completed(char* buffer, uint32_t length)
{
	uint32_t	datalen;
	uint32_t	len;
	char*		ptr;
	char*		data_start;

	// Search first comma
	ptr = strchr(buffer, ',');
	if(NULL == ptr){
		return false;
	}

	// Multi connection mode
	if(multi_conn){
		// Search next comma
		ptr++;
		ptr = strchr(ptr, ',');
		if(NULL == ptr){
			return false;
		}
	}

	// Search colon
	ptr++;
	data_start = strchr(ptr, ':');
	if(NULL == data_start){
		return false;
	}
	data_start++;

	// Show remote information
	if(show_remote_info){
		sscanf(ptr, "%d,", &datalen);
	}
	// Not show remote information
	else{
		sscanf(ptr, "%d:", &datalen);
	}

	if(buffer + length != data_start + datalen){
		ipd_received = false;
	}
	else{
		ipd_received = true;
	}

	return ipd_received;
}
#endif

///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
//		_Receive_Completed
// 機能：
//		UART割込み処理の中で受信完了を判定する。
//　引数：
//		buffer : 受信バッファのポインタ
//		length - 受信データ長
// 戻り値
//		true - 受信完了、false - 受信継続
//
///////////////////////////////////////////////////////////////////////////////
bool _Receive_Completed(char* buffer, uint32_t length)
{
	bool		ipd_cmd = false;

	if(length < 4){
		return false;
	}

	if(0 == strncmp(buffer, "+IPD", 4)){
		ipd_cmd = true;
	}

	// IPD command received
	if(ipd_cmd){
		ipd_received = IPD_Receive_Completed(buffer, length);
		if(ipd_received){
			// Command received from WiFi, invoke Task
		}

		return ipd_received;
	}
	// AT command completed
	else{
		at_completed = AT_Receive_Completed(buffer, length);
		if(at_completed){
			// AT completed, do something ...
		}

		return at_completed;
	}
}

///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
//		IPD_Get_Command
// 機能：
//		受信したIPDコマンドのデータを取得する。
//　引数：
//		ipd_info : IPDコマンドに含まれる情報
// 戻り値
//		true - 受信データあり、false - 受信データなし
//
///////////////////////////////////////////////////////////////////////////////
bool IPD_Get_Command(IPD_INFO* ipd_info)
{
	uint32_t	data_num;
	char*		ptr;
	char		ret_val[128];
	bool		result;

	// IPD command received
	if(!ipd_received){
		return false;
	}

	ipd_info->multi_conn = multi_conn;
	ipd_info->show_remote = show_remote_info;

	// Get first parameter
	result = IPD_Get_Param(cmd_resp, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}

	// Multiple connection
	if(multi_conn){
		// Get connection ID
		sscanf(ret_val, "%d", &ipd_info->conn_id);

		// Get data length
		result = IPD_Get_Param(cmd_resp, 1, ret_val, sizeof(ret_val));
		if(!result){
			return false;
		}
		sscanf(ret_val, "%d", &ipd_info->data_len);

		// Data count = 2
		data_num = 2;
	}
	// Single connection
	else{
		// Get data length
		sscanf(ret_val, "%d", &ipd_info->data_len);
		data_num = 1;
	}

	// Check data buffer length
	if(ipd_info->data_len > ipd_info->data_buf_size){
		return false;
	}

	// In case of showing remote information
	if(show_remote_info){
		data_num += 2;

		// Get remote IP address
		result = IPD_Get_Param(cmd_resp, data_num - 2, ipd_info->remote_ip, ipd_info->remote_ip_len);
		if(!result){
			return false;
		}

		// Get remote port number
		result = IPD_Get_Param(cmd_resp, data_num - 1, ret_val, sizeof(ret_val));
		if(!result){
			return false;
		}
		sscanf(ret_val, "%d", &ipd_info->remote_port);
	}

	// Search for first data pointer
	ptr = strchr(cmd_resp, ':');
	if(NULL == ptr){
		return false;
	}
	ptr++;

	// Get received data
	memcpy(ipd_info->data, ptr, ipd_info->data_len);
	// Null terminate received data
	ipd_info->data[ipd_info->data_len] = '\0';

	// Reset IPD received flag
	ipd_received = false;

	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// Basic AT Command Set
//
///////////////////////////////////////////////////////////////////////////////

bool AT_Exe(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT Command Process
	length = At_Command(CMD_AT, Cmd_Execute);

	// Confirm OK string
	return IsOK(cmd_resp);
}

bool AT_RST_Exe(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT Command Process
	length = At_Command(CMD_RST, Cmd_Execute);

	// Confirm OK string
	return IsOK(cmd_resp);
}

bool AT_GMR_Exe(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_GMR_OUT*	param = (AT_GMR_OUT*)output;
	bool		result;
	char*		ptr;
	char*		line_end;

	if(NULL == output || sizeof(AT_GMR_OUT) > output_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT Command Process
	length = At_Command(CMD_GMR, Cmd_Execute);

	result = IsOK(cmd_resp);
	if(!result){
		return false;
	}

	// Get AT version info
	ptr = strstr(cmd_resp, "AT version");
	line_end = strchr(ptr, '\r');
	length = (uint32_t)(line_end - ptr) + 1;
	if(param->at_ver_len < length){
		return false;
	}
	strncpy(param->at_ver, ptr, length - 1);

	// Get SDK version info
	ptr = strstr(cmd_resp, "SDK version");
	line_end = strchr(ptr, '\r');
	length = (uint32_t)(line_end - ptr) + 1;
	if(param->sdk_ver_len < length){
		return false;
	}
	strncpy(param->sdk_ver, ptr, length - 1);

	// Get compile time
	ptr = strstr(cmd_resp, "compile time");
	line_end = strchr(ptr, '\r');
	length = (uint32_t)(line_end - ptr) + 1;
	if(param->compile_time_len < length){
		return false;
	}
	strncpy(param->compile_time, ptr, length - 1);

	// Confirm OK string
	return true;
}

bool AT_GSLP_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	uint32_t*	msec = (uint32_t*)input;

	// Check input buffer length
	if(NULL == input || sizeof(*msec) > input_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// Create param string
	sprintf(cmd_param, "%d", *msec);
	// AT Command Process
	length = At_Command(CMD_GSLP, Cmd_Set);

	// Confirm OK string
	return IsOK(cmd_resp);
}

bool AT_ATE_Exe(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	bool*		echo = (bool*)input;

	// Check input buffer length
	if(NULL == input || sizeof(*echo) > input_len){
		return false;
	}

	// Set "OK" and "ERROR" as Delimiter
	AT_Set_Delimiter(DLMT_Error);
	if(*echo){
		length = At_Command(CMD_ATE1, Cmd_Execute);
	}
	else{
		length = At_Command(CMD_ATE0, Cmd_Execute);
	}

	// Confirm OK string
	return IsOK(cmd_resp);
}

bool AT_RESTORE_Exe(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT Command Process
	length = At_Command(CMD_RESTORE, Cmd_Execute);

	// Confirm OK string
	return IsOK(cmd_resp);
}

bool AT_UART_CUR_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_UART_IN*	param = (AT_UART_IN*)input;

	// Check input buffer length
	if(NULL == input || sizeof(AT_UART_IN) > input_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// Create param string
	sprintf(cmd_param, "%d,%d,%d,%d,%d", param->baudrate, param->databits, param->stopbits, param->parity, param->flow);
	// AT Command Process
	length = At_Command(CMD_UART_CUR, Cmd_Set);

	// Confirm OK string
	return IsOK(cmd_resp);
}

bool AT_UART_DEF_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_UART_IN*	param = (AT_UART_IN*)input;

	// Check input buffer length
	if(NULL == input || sizeof(AT_UART_IN) > input_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// Create param string
	sprintf(cmd_param, "%d,%d,%d,%d,%d", param->baudrate, param->databits, param->stopbits, param->parity, param->flow);
	// AT Command Process
	length = At_Command(CMD_UART_DEF, Cmd_Set);

	// Confirm OK string
	return IsOK(cmd_resp);
}

bool AT_SLEEP_Query(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	uint32_t*	mode = (uint32_t*)output;
	char		ret_val[4];
	bool		result;

	// Check output buffer length
	if(NULL == output || sizeof(*mode) > output_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT Command Process
	length = At_Command(CMD_SLEEP, Cmd_Query);

	// Confirm OK string
	result = IsOK(cmd_resp);
	// Get Sleep Mode
	if(!result){
		return false;
	}
	result = AT_Get_Param(cmd_resp, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", mode);

	return result;
}

bool AT_SLEEP_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	uint32_t*	mode = (uint32_t*)input;

	// Check input buffer length
	if(NULL == input || sizeof(*mode) > input_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// Create param string
	sprintf(cmd_param, "%d", *mode);
	// AT Command Process
	length = At_Command(CMD_SLEEP, Cmd_Set);

	// Confirm OK string
	return IsOK(cmd_resp);
}

bool AT_RFPOWER_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	uint32_t*	power = (uint32_t*)input;

	// Check input buffer length
	if(NULL == input || sizeof(*power) > input_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// Create param string
	sprintf(cmd_param, "%d", *power);
	// AT Command Process
	length = At_Command(CMD_RFPOWER, Cmd_Set);

	// Confirm OK string
	return IsOK(cmd_resp);
}

bool AT_RFVDD_Query(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	uint32_t*	vdd = (uint32_t*)output;
	char		ret_val[16];
	bool		result;

	// Check output buffer length
	if(NULL == output || sizeof(*vdd) > output_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT Command Process
	length = At_Command(CMD_RFVDD, Cmd_Query);

	// Confirm OK string
	result = IsOK(cmd_resp);
	if(!result){
		return false;
	}
	// Get VDD33
	result = AT_Get_Param(cmd_resp, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", vdd);

	return result;
}

bool AT_RFVDD_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	uint32_t*	vdd = (uint32_t*)input;

	// Check input buffer length
	if(NULL == input || sizeof(*vdd) > input_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// Create param string
	sprintf(cmd_param, "%d", *vdd);
	// AT Command Process
	length = At_Command(CMD_RFVDD, Cmd_Set);

	// Confirm OK string
	return IsOK(cmd_resp);
}

bool AT_RFVDD_Exe(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT Command Process
	length = At_Command(CMD_RFVDD, Cmd_Execute);

	// Confirm OK string
	return IsOK(cmd_resp);
}

///////////////////////////////////////////////////////////////////////////////
//
// WiFi AT Command Set
//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CWMODE_CUR=?
//
// Function: Current WiFi Mode Query
//
// 1 : Station Mode
// 2 : softAP Mode
// 3 : softAP + Station Mode
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CWMODE_CUR_Query(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	uint32_t*	mode = (uint32_t*)output;
	char		ret_val[4];
	bool		result;

	// Check output buffer length
	if(NULL == output || sizeof(*mode) > output_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT Command Process
	length = At_Command(CMD_CWMODE_CUR, Cmd_Query);

	// Confirm OK string
	result = IsOK(cmd_resp);
	if(!result){
		return false;
	}
	// Get WiFi Mode
	result = AT_Get_Param(cmd_resp, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", mode);

	return result;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CWMODE_CUR=<mode>
//
// Function: Current WiFi Mode Set(won't save flash)
//
// 1 : Station Mode
// 2 : softAP Mode
// 3 : softAP + Station Mode
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CWMODE_CUR_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	uint32_t*	mode = (uint32_t*)input;

	// Check input buffer length
	if(NULL == input || sizeof(*mode) > input_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// Create param string
	sprintf(cmd_param, "%d", *mode);
	// AT Command Process
	length = At_Command(CMD_CWMODE_CUR, Cmd_Set);

	// Confirm OK string
	return IsOK(cmd_resp);
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CWMODE_DEF=?
//
// Function: Default WiFi Mode Query
//
// 1 : Station Mode
// 2 : softAP Mode
// 3 : softAP + Station Mode
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CWMODE_DEF_Query(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	uint32_t*	mode = (uint32_t*)output;
	char		ret_val[4];
	bool		result;

	// Check output buffer length
	if(NULL == output || sizeof(*mode) > output_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT Command Process
	length = At_Command(CMD_CWMODE_DEF, Cmd_Query);

	// Confirm OK string
	result = IsOK(cmd_resp);
	if(!result){
		return false;
	}
	// Get WiFi Mode
	result = AT_Get_Param(cmd_resp, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", mode);

	return result;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CWMODE_DEF=<mode>
//
// Function: Default WiFi Mode Set(save to flash)
//
// 1 : Station Mode
// 2 : softAP Mode
// 3 : softAP + Station Mode
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CWMODE_DEF_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	uint32_t*	mode = (uint32_t*)input;

	// Check input buffer length
	if(NULL == input || sizeof(*mode) > input_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// Create param string
	sprintf(cmd_param, "%d", *mode);
	// AT Command Process
	length = At_Command(CMD_CWMODE_DEF, Cmd_Set);

	// Confirm OK string
	return IsOK(cmd_resp);
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CWJAP_CUR?
//
// Function: Connect to AP
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CWJAP_CUR_Query(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t		length;
	AT_CWJAP_OUT*	param = (AT_CWJAP_OUT*)output;
	char			ret_val[8];
	bool			result;

	// Check output buffer length
	if(NULL == output || sizeof(AT_CWJAP_OUT) > output_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT Command Process
	length = At_Command(CMD_CWJAP_CUR, Cmd_Query);

	result = IsOK(cmd_resp);
	if(!result){
		return false;
	}

	// Return string is "+CWJAP_CUR:<ssid>,<bssid>,<channel>,<rssi>\r\n"
	// Get ssid
	result = AT_Get_Param(cmd_resp, 0, param->ssid, param->ssid_len);
	if(!result){
		return false;
	}
	// Get bssid
	result = AT_Get_Param(cmd_resp, 1, param->bssid, param->bssid_len);
	if(!result){
		return false;
	}
	result = AT_Get_Param(cmd_resp, 2, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", &param->channel);
	result = AT_Get_Param(cmd_resp, 3, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", &param->rssi);

	// Confirm OK string
	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CWJAP_CUR=<ssid>,<password>[,<bssid>]
//
// Function: Connect to AP(won't save to flash)
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CWJAP_CUR_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t		length;
	AT_CWJAP_IN*	param = (AT_CWJAP_IN*)input;
	uint32_t*		error = (uint32_t*)output;
	char			ret_val[8];
	bool			result;

	// Check input buffer length
	if(NULL == input || sizeof(AT_CWJAP_IN) > input_len){
		return false;
	}

	// Check output buffer length
	if(NULL == output || sizeof(*error) > output_len){
		return false;
	}

	// Set "OK" and "FAIL" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// Create param string
	if(param->bssid){
		sprintf(cmd_param, "\"%s\",\"%s\",\"%s\"", param->ssid, param->passwd, param->bssid);
	}
	else{
		sprintf(cmd_param, "\"%s\",\"%s\"", param->ssid, param->passwd);
	}
	// AT Command Process
	length = At_Command(CMD_CWJAP_CUR, Cmd_Set);

	// Confirm OK string
	result = IsOK(cmd_resp);
	if(!result){
		// If failed, get error code
		result = AT_Get_Param(cmd_resp, 0, ret_val, sizeof(ret_val));
		if(!result){
			return false;
		}
		sscanf(ret_val, "%d", error);
		return false;
	}

	// Command successful
	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CWJAP_DEF?
//
// Function: Connect to AP
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CWJAP_DEF_Query(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t		length;
	AT_CWJAP_OUT*	param = (AT_CWJAP_OUT*)output;
	char			ret_val[8];
	bool			result;

	// Check input buffer length
	if(NULL == output || sizeof(AT_CWJAP_OUT) > output_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT Command Process
	length = At_Command(CMD_CWJAP_DEF, Cmd_Query);

	result = IsOK(cmd_resp);
	if(!result){
		return false;
	}

	// Return string is "+CWJAP_CUR:<ssid>,<bssid>,<channel>,<rssi>\r\n"
	// Get ssid
	result = AT_Get_Param(cmd_resp, 0, param->ssid, param->ssid_len);
	if(!result){
		return false;
	}
	// Get bssid
	result = AT_Get_Param(cmd_resp, 1, param->bssid, param->bssid_len);
	if(!result){
		return false;
	}
	result = AT_Get_Param(cmd_resp, 2, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", &param->channel);
	result = AT_Get_Param(cmd_resp, 3, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", &param->rssi);

	// Confirm OK string
	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CWJAP_DEF=<ssid>,<password>[,<bssid>]
//
// Function: Connect to AP(won't save to flash)
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CWJAP_DEF_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t		length;
	AT_CWJAP_IN*	param = (AT_CWJAP_IN*)input;
	uint32_t*		error = (uint32_t*)output;
	char			ret_val[8];
	bool			result;

	// Check input buffer length
	if(NULL == input || sizeof(AT_CWJAP_IN) > input_len){
		return false;
	}

	// Check output buffer length
	if(NULL == output || sizeof(*error) > output_len){
		return false;
	}

	// Set "OK" and "FAIL" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// Create param string
	if(param->bssid){
		sprintf(cmd_param, "\"%s\",\"%s\",\"%s\"", param->ssid, param->passwd, param->bssid);
	}
	else{
		sprintf(cmd_param, "\"%s\",\"%s\"", param->ssid, param->passwd);
	}
	// AT Command Process
	length = At_Command(CMD_CWJAP_DEF, Cmd_Set);

	// Confirm OK string
	result = IsOK(cmd_resp);
	if(!result){
		// If failed, get error code
		result = AT_Get_Param(cmd_resp, 0, ret_val, sizeof(ret_val));
		if(!result){
			return false;
		}
		sscanf(ret_val, "%d", error);
		return false;
	}

	// Command successful
	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CWLAP=<ssid>[,<mac>,<ch>]
//
// Function: List available AP's
//
// ecn:
//	0 : OPEN
//	1 : WEP
//	2 : WPA_PSK
//	3 : WPA2_PSK
//	4 : WPA_WPA2_PSK
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CWLAP_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t		length;
	AT_CWLAP_IN*	param = (AT_CWLAP_IN*)input;
	uint32_t*		ecn = (uint32_t*)output;
	char			ret_val[8];
	bool			result;

	// Check input buffer length
	if(NULL == input || sizeof(AT_CWLAP_IN) > input_len){
		return false;
	}

	// Check output buffer length
	if(NULL == output || sizeof(*ecn) > output_len){
		return false;
	}

	// Set "OK" and "ERROR" as Delimiter
	AT_Set_Delimiter(DLMT_Error);
	if(!param->set_ssid_only){
		sprintf(cmd_param, "\"%s\",\"%02x:%02x:%02x:%02x:%02x:%02x\",%d",
				param->ssid, param->mac[0], param->mac[1], param->mac[2],
				param->mac[3], param->mac[4], param->mac[5], param->channel);
	}
	else{
		sprintf(cmd_param, "\"%s\"", param->ssid);
	}
	// AT Command Process
	length = At_Command(CMD_CWLAP, Cmd_Set);

	result = IsOK(cmd_resp);
	if(!result){
		return false;
	}
	result = AT_Get_Param(cmd_resp, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", ecn);

	// Confirm OK string
	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CWLAP
//
// Function: List available AP's
//
// ecn:
//	0 : OPEN
//	1 : WEP
//	2 : WPA_PSK
//	3 : WPA2_PSK
//	4 : WPA_WPA2_PSK
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CWLAP_Exe(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CWLAP_OUT*	param = (AT_CWLAP_OUT*)output;
	char		ret_val[20];
	bool		result;
	uint32_t	i;
	uint32_t	mac[6];

	// Check output buffer length
	if(NULL == output || sizeof(AT_CWLAP_OUT) > output_len){
		return false;
	}

	// Set "OK" and "ERROR" as Delimiter
	AT_Set_Delimiter(DLMT_Error);
	// AT Command Process
	length = At_Command(CMD_CWLAP, Cmd_Execute);

	result = IsOK(cmd_resp);
	if(!result){
		return false;
	}

	// Return string is "+CWLAP:<ecn>,<ssid>,<rssi>,<mac>,<ch>,<freq offset>, <freq calibration>\r\n"
	// Get <ecn>
	result = AT_Get_Param(cmd_resp, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", &param->ecn);
	// Get <ssid>
	result = AT_Get_Param(cmd_resp, 1, param->ssid, param->ssid_len);
	if(!result){
		return false;
	}
	// Get <rssi>
	result = AT_Get_Param(cmd_resp, 2, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", &param->rssi);
	// Get mac address
	result = AT_Get_Param(cmd_resp, 3, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%02x:%02x:%02x:%02x:%02x:%02x", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
	for(i = 0; i < 6; i++){
		param->mac[i] = (uint8_t)mac[i];
	}
	// Get <channel>
	result = AT_Get_Param(cmd_resp, 4, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", &param->channel);
	// Get <freq_off>
	result = AT_Get_Param(cmd_resp, 5, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", &param->freq_off);
	// Get <freq_cal>
	result = AT_Get_Param(cmd_resp, 6, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", &param->freq_cal);

	// Confirm OK string
	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CWQAP
//
// Function: Disconnect from AP
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CWQAP_Exe(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT Command Process
	length = At_Command(CMD_CWQAP, Cmd_Execute);

	// Confirm OK string
	return IsOK(cmd_resp);
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CWSAP_CUR?
//
// Function: Current config of softAP mode
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CWSAP_CUR_Query(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CWSAP_IN_OUT*	param = (AT_CWSAP_IN_OUT*)output;
	char			ret_val[8];
	bool			result;

	// Check output buffer length
	if(NULL == output || sizeof(AT_CWSAP_IN_OUT) > output_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT Command Process
	length = At_Command(CMD_CWSAP_CUR, Cmd_Query);
	result = IsOK(cmd_resp);
	if(!result){
		return false;
	}

	// Return string is "+CWSAP_CUR:<ssid>,<passwd>,<channel>,<ecn>,<max conn>,<ssid hidden>\r\n"
	result = AT_Get_Param(cmd_resp, 0, param->ssid, param->ssid_len);
	if(!result){
		return false;
	}
	result = AT_Get_Param(cmd_resp, 1, param->passwd, param->passwd_len);
	if(!result){
		return false;
	}
	result = AT_Get_Param(cmd_resp, 2, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", param->channel);
	result = AT_Get_Param(cmd_resp, 3, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", param->ecn);
	result = AT_Get_Param(cmd_resp, 4, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", param->max_conn);
	result = AT_Get_Param(cmd_resp, 5, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", param->ssid_hidden);

	// Confirm OK string
	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CWSAP_CUR=<ssid>,<passwd>,<channel>,<ecn>,<max conn>,<ssid hidden>
//
// Function: Current config of softAP mode
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CWSAP_CUR_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CWSAP_IN_OUT*	param = (AT_CWSAP_IN_OUT*)input;

	// Check input buffer length
	if(NULL == input || sizeof(AT_CWSAP_IN_OUT) > input_len){
		return false;
	}

	// Set "OK" and "ERROR" as Delimiter
	AT_Set_Delimiter(DLMT_Error);
	// Create param string
	sprintf(cmd_param, "\"%s\",\"%s\",%d,%d,%d,%d",
			param->ssid, param->passwd, param->channel, param->ecn,
			param->max_conn, param->ssid_hidden);
	// AT Command Process
	length = At_Command(CMD_CWSAP_CUR, Cmd_Set);

	// Confirm OK string
	return IsOK(cmd_resp);
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CWSAP_DEF?
//
// Function: Current config of softAP mode
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CWSAP_DEF_Query(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CWSAP_IN_OUT*	param = (AT_CWSAP_IN_OUT*)output;
	char			ret_val[8];
	bool			result;

	// Check output buffer length
	if(NULL == output || sizeof(AT_CWSAP_IN_OUT) > output_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT Command Process
	length = At_Command(CMD_CWSAP_DEF, Cmd_Query);
	result = IsOK(cmd_resp);
	if(!result){
		return false;
	}

	// Return string is "+CWSAP_CUR:<ssid>,<passwd>,<channel>,<ecn>,<max conn>,<ssid hidden>\r\n"
	result = AT_Get_Param(cmd_resp, 0, param->ssid, param->ssid_len);
	if(!result){
		return false;
	}
	result = AT_Get_Param(cmd_resp, 1, param->passwd, param->passwd_len);
	if(!result){
		return false;
	}
	result = AT_Get_Param(cmd_resp, 2, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", param->channel);
	result = AT_Get_Param(cmd_resp, 3, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", param->ecn);
	result = AT_Get_Param(cmd_resp, 4, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", param->max_conn);
	result = AT_Get_Param(cmd_resp, 5, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", param->ssid_hidden);

	// Confirm OK string
	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CWSAP_DEF=<ssid>,<passwd>,<channel>,<ecn>,<max conn>,<ssid hidden>
//
// Function: Set Current config of softAP mode(will save to flash)
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CWSAP_DEF_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CWSAP_IN_OUT*	param = (AT_CWSAP_IN_OUT*)input;

	// Check input buffer length
	if(NULL == input || sizeof(AT_CWSAP_IN_OUT) > input_len){
		return false;
	}

	// Set "OK" and "ERROR" as Delimiter
	AT_Set_Delimiter(DLMT_Error);
	// Create param string
	sprintf(cmd_param, "\"%s\",\"%s\",%d,%d,%d,%d",
			param->ssid, param->passwd, param->channel, param->ecn,
			param->max_conn, param->ssid_hidden);
	// AT Command Process
	length = At_Command(CMD_CWSAP_DEF, Cmd_Set);

	// Confirm OK string
	return IsOK(cmd_resp);
}

#if 1
///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CWLIF
//
// Function: IP of stations
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CWLIF_Exe(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CWLIF_OUT*	param = (AT_CWLIF_OUT*)output;
	char			ret_val[20];
	bool			result;
	uint32_t		i;
	uint32_t		ip[4], mac[6];

	// Check output buffer length
	if(NULL == output || sizeof(AT_CWLIF_OUT) > output_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT Command Process
	length = At_Command(CMD_CWLIF, Cmd_Execute);

	result = IsOK(cmd_resp);
	if(!result){
		return false;
	}

	// Return string is "<IP addr>\r\n"
	result = AT_Get_Param(cmd_resp, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d.%d.%d.%d", &ip[0], &ip[1], &ip[2], &ip[3]);
	for(i = 0; i < 4; i++){
		param->ip[i] = (uint8_t)ip[i];
	}

	result = AT_Get_Param(cmd_resp, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%02x:%02x:%02x:%02x:%02x:%02x", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
	for(i = 0; i < 6; i++){
		param->mac[i] = (uint8_t)mac[i];
	}

	// Confirm OK string
	return true;
}
#endif

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CWDHCP_CUR
//
// Function: Enable/Disable DHCP
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CWDHCP_CUR_Query(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	uint32_t*	mode = (uint32_t*)output;
	char		ret_val[4];
	bool		result;

	// Check output buffer length
	if(NULL == output || sizeof(*mode) < output_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT Command Process
	length = At_Command(CMD_CWDHCP_CUR, Cmd_Query);

	result = IsOK(cmd_resp);
	if(!result){
		return false;
	}

	result = AT_Get_Param(cmd_resp, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", mode);

	// Confirm OK string
	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CWDHCP_CUR=<mode>,<en>
//	<mode>
//		0 : set ESP8266 softAP
//		1 : set ESP8266 station
//		2 : set both softAP and station
//	<en>
//		0 : Disable DHCP
//		1 : Enable DHCP
//
// Function: Enable/Disable DHCP
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CWDHCP_CUR_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CWDHCP_IN*	param = (AT_CWDHCP_IN*)input;

	// Check input buffer length
	if(NULL == input || sizeof(AT_CWDHCP_IN) > input_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// Create param string
	sprintf(cmd_param, "%d,%d", param->mode, param->en);
	// AT Command Process
	length = At_Command(CMD_CWDHCP_CUR, Cmd_Set);

	// Confirm OK string
	return IsOK(cmd_resp);
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CWDHCP_DEF?
//
// Function: Enable/Disable DHCP
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CWDHCP_DEF_Query(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	uint32_t*	mode = (uint32_t*)output;
	char		ret_val[4];
	bool		result;

	// Check output buffer length
	if(NULL == output || sizeof(*mode) > output_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT Command Process
	length = At_Command(CMD_CWDHCP_DEF, Cmd_Query);

	result = IsOK(cmd_resp);
	if(!result){
		return false;
	}

	result = AT_Get_Param(cmd_resp, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", mode);

	// Confirm OK string
	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CWDHCP_DEF=<mode>,<en>
//	<mode>
//		0 : set ESP8266 softAP
//		1 : set ESP8266 station
//		2 : set both softAP and station
//	<en>
//		0 : Disable DHCP
//		1 : Enable DHCP
//
// Function: Enable/Disable DHCP
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CWDHCP_DEF_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CWDHCP_IN*	param = (AT_CWDHCP_IN*)input;

	// Check input buffer length
	if(NULL == input || sizeof(AT_CWDHCP_IN) > input_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// Create param string
	sprintf(cmd_param, "%d,%d", param->mode, param->en);
	// AT Command Process
	length = At_Command(CMD_CWDHCP_DEF, Cmd_Set);

	// Confirm OK string
	return IsOK(cmd_resp);
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CWDHCPS_CUR?
//
// Function:
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CWDHCPS_CUR_Query(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CWDHCPS_IN_OUT*	param = (AT_CWDHCPS_IN_OUT*)output;
	char		ret_val[20];
	bool		result;

	// Check input buffer length
	if(NULL == output || sizeof(AT_CWDHCPS_IN_OUT) > output_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT Command Process
	length = At_Command(CMD_CWDHCPS_CUR, Cmd_Query);

	// Return string is "+CWDHCPS_CUR=<lease time>,<start IP>,<end IP>\r\n"
	result = AT_Get_Param(cmd_resp, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", &param->lease_time);
	result = AT_Get_Param(cmd_resp, 1, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d.%d.%d.%d",
			&param->start_ip[0], &param->start_ip[1], &param->start_ip[2], &param->start_ip[3]);
	result = AT_Get_Param(cmd_resp, 2, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d.%d.%d.%d",
			&param->end_ip[0], &param->end_ip[1], &param->end_ip[2], &param->end_ip[3]);

	// Confirm OK string
	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CWDHCPS_CUR=<enable>,<lease time>,<start IP>,<end IP>
//
// Function:
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CWDHCPS_CUR_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CWDHCPS_IN_OUT*	param = (AT_CWDHCPS_IN_OUT*)input;

	// Check input buffer length
	if(NULL == input || sizeof(AT_CWDHCPS_IN_OUT) > input_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// Create param string
	sprintf(cmd_param, "%d,%d,\"%d.%d.%d.%d\",\"%d.%d.%d.%d\"",
			param->enable, param->lease_time,
			param->start_ip[0], param->start_ip[1], param->start_ip[2], param->start_ip[3],
			param->end_ip[0], param->end_ip[1], param->end_ip[2], param->end_ip[3]);
	// AT Command Process
	length = At_Command(CMD_CWDHCPS_CUR, Cmd_Set);

	// Confirm OK string
	return IsOK(cmd_resp);
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CWDHCPS_DEF?
//
// Function:
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CWDHCPS_DEF_Query(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CWDHCPS_IN_OUT*	param = (AT_CWDHCPS_IN_OUT*)output;
	char		ret_val[20];
	bool		result;

	// Check output buffer length
	if(NULL == output || sizeof(AT_CWDHCPS_IN_OUT) > output_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT Command Process
	length = At_Command(CMD_CWDHCPS_DEF, Cmd_Query);

	// Return string is "+CWDHCPS_CUR=<lease time>,<start IP>,<end IP>\r\n"
	result = AT_Get_Param(cmd_resp, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", &param->lease_time);
	result = AT_Get_Param(cmd_resp, 1, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d.%d.%d.%d",
			&param->start_ip[0], &param->start_ip[1], &param->start_ip[2], &param->start_ip[3]);
	result = AT_Get_Param(cmd_resp, 2, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d.%d.%d.%d",
			&param->end_ip[0], &param->end_ip[1], &param->end_ip[2], &param->end_ip[3]);

	// Confirm OK string
	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CWDHCPS_DEF=<enable>,<lease time>,<start IP>,<end IP>
//
// Function:
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CWDHCPS_DEF_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CWDHCPS_IN_OUT*	param = (AT_CWDHCPS_IN_OUT*)input;

	// Check input buffer length
	if(NULL == input || sizeof(AT_CWDHCPS_IN_OUT) > input_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// Create param string
	sprintf(cmd_param, "%d,%d,\"%d.%d.%d.%d\",\"%d.%d.%d.%d\"",
			param->enable, param->lease_time,
			param->start_ip[0], param->start_ip[1], param->start_ip[2], param->start_ip[3],
			param->end_ip[0], param->end_ip[1], param->end_ip[2], param->end_ip[3]);
	// AT Command Process
	length = At_Command(CMD_CWDHCPS_DEF, Cmd_Set);

	// Confirm OK string
	return IsOK(cmd_resp);
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CWAUTOCONN=<enable>
//
// Function: Connect to AP automatically or not
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CWAUTOCONN_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	uint32_t*	enable = (uint32_t*)input;

	// Check input buffer length
	if(NULL == input || sizeof(*enable) > input_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// Create param string
	sprintf(cmd_param, "%d", *enable);
	// AT Command Process
	length = At_Command(CMD_CWAUTOCONN, Cmd_Set);

	// Confirm OK string
	return IsOK(cmd_resp);
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPSTAMAC_CUR?
//
// Function: Set MAC address of ESP8266 station(won't save to Flash)
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPSTAMAC_CUR_Query(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CIPMAC_IN_OUT*	param = (AT_CIPMAC_IN_OUT*)output;
	char				ret_val[20];
	bool				result;
	uint32_t			i;
	uint32_t			mac[6];

	// Check output buffer length
	if(NULL == output || sizeof(AT_CIPMAC_IN_OUT) > output_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT Command Process
	length = At_Command(CMD_CIPSTAMAC_CUR, Cmd_Query);
	result = IsOK(cmd_resp);
	if(!result){
		return false;
	}

	// Return string is "+CIPSTAMAC_CUR:<mac>\r\n"
	result = AT_Get_Param(cmd_resp, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%02x:%02x:%02x:%02x:%02x:%02x", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
	for(i = 0; i < 6; i++){
		param->mac[i] = (uint8_t)mac[i];
	}

	// Confirm OK string
	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPSTAMAC_CUR=<mac>
//
// Function: Set MAC address of ESP8266 station(won't save to Flash)
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPSTAMAC_CUR_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CIPMAC_IN_OUT*	param = (AT_CIPMAC_IN_OUT*)input;

	// Check input buffer length
	if(NULL == input || sizeof(AT_CIPMAC_IN_OUT) > input_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// Create param string
	sprintf(cmd_param, "\"%02x:%02x:%02x:%02x:%02x:%02x\"",
			param->mac[0], param->mac[1], param->mac[2],
			param->mac[3], param->mac[4], param->mac[5]);
	// AT Command Process
	length = At_Command(CMD_CIPSTAMAC_CUR, Cmd_Set);

	// Confirm OK string
	return IsOK(cmd_resp);
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPSTAMAC_DEF?
//
// Function: Set MAC address of ESP8266 station(Save to Flash)
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPSTAMAC_DEF_Query(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CIPMAC_IN_OUT*	param = (AT_CIPMAC_IN_OUT*)output;
	char				ret_val[20];
	bool				result;
	uint32_t			i;
	uint32_t			mac[6];

	// Check output buffer length
	if(NULL == output || sizeof(AT_CIPMAC_IN_OUT) > output_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT Command Process
	length = At_Command(CMD_CIPSTAMAC_DEF, Cmd_Query);
	result = IsOK(cmd_resp);
	if(!result){
		return false;
	}

	// Return string is "+CIPSTAMAC_CUR:<mac>\r\n"
	result = AT_Get_Param(cmd_resp, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%02x:%02x:%02x:%02x:%02x:%02x", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
	for(i = 0; i < 6; i++){
		param->mac[i] = (uint8_t)mac[i];
	}

	// Confirm OK string
	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPSTAMAC_DEF=<mac>
//
// Function: Set MAC address of ESP8266 station(Save to Flash)
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPSTAMAC_DEF_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CIPMAC_IN_OUT*	param = (AT_CIPMAC_IN_OUT*)input;

	// Check input buffer length
	if(NULL == input || sizeof(AT_CIPMAC_IN_OUT) > input_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// Create param string
	sprintf(cmd_param, "\"%02x:%02x:%02x:%02x:%02x:%02x\"",
			param->mac[0], param->mac[1], param->mac[2],
			param->mac[3], param->mac[4], param->mac[5]);
	// AT Command Process
	length = At_Command(CMD_CIPSTAMAC_DEF, Cmd_Set);

	// Confirm OK string
	return IsOK(cmd_resp);
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPAPMAC_CUR?
//
// Function: Set MAC address of softAP(won't save to Flash)
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPAPMAC_CUR_Query(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CIPMAC_IN_OUT*	param = (AT_CIPMAC_IN_OUT*)output;
	char				ret_val[20];
	bool				result;
	uint32_t			i;
	uint32_t			mac[6];

	// Check output buffer length
	if(NULL == output || sizeof(AT_CIPMAC_IN_OUT) > output_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT Command Process
	length = At_Command(CMD_CIPAPMAC_CUR, Cmd_Query);
	result = IsOK(cmd_resp);
	if(!result){
		return false;
	}

	// Return string is "+CIPSTAMAC_CUR:<mac>\r\n"
	result = AT_Get_Param(cmd_resp, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%02x:%02x:%02x:%02x:%02x:%02x", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
	for(i = 0; i < 6; i++){
		param->mac[i] = (uint8_t)mac[i];
	}

	// Confirm OK string
	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPAPMAC_CUR=<mac>
//
// Function: Set MAC address of softAP(won't save to Flash)
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPAPMAC_CUR_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CIPMAC_IN_OUT*	param = (AT_CIPMAC_IN_OUT*)input;

	// Check input buffer length
	if(NULL == input || sizeof(AT_CIPMAC_IN_OUT) > input_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// Create param string
	sprintf(cmd_param, "\"%02x:%02x:%02x:%02x:%02x:%02x\"",
			param->mac[0], param->mac[1], param->mac[2],
			param->mac[3], param->mac[4], param->mac[5]);
	// AT Command Process
	length = At_Command(CMD_CIPAPMAC_CUR, Cmd_Set);

	// Confirm OK string
	return IsOK(cmd_resp);
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPAPMAC_DEF?
//
// Function: Set MAC address of softAP(Save to Flash)
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPAPMAC_DEF_Query(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CIPMAC_IN_OUT*	param = (AT_CIPMAC_IN_OUT*)output;
	char				ret_val[20];
	bool				result;
	uint32_t			i;
	uint32_t			mac[6];

	// Check output buffer length
	if(NULL == output || sizeof(AT_CIPMAC_IN_OUT) > output_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT Command Process
	length = At_Command(CMD_CIPAPMAC_DEF, Cmd_Query);
	result = IsOK(cmd_resp);
	if(!result){
		return false;
	}

	// Return string is "+CIPSTAMAC_CUR:<mac>\r\n"
	result = AT_Get_Param(cmd_resp, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%02x:%02x:%02x:%02x:%02x:%02x", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
	for(i = 0; i < 6; i++){
		param->mac[i] = (uint8_t)mac[i];
	}

	// Confirm OK string
	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPAPMAC_DEF=<mac>
//
// Function: Set MAC address of ESP8266 station(Save to Flash)
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPAPMAC_DEF_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CIPMAC_IN_OUT*	param = (AT_CIPMAC_IN_OUT*)input;

	// Check input buffer length
	if(NULL == input || sizeof(AT_CIPMAC_IN_OUT) > input_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// Create param string
	sprintf(cmd_param, "\"%02x:%02x:%02x:%02x:%02x:%02x\"",
			param->mac[0], param->mac[1], param->mac[2],
			param->mac[3], param->mac[4], param->mac[5]);
	// AT Command Process
	length = At_Command(CMD_CIPAPMAC_DEF, Cmd_Set);

	// Confirm OK string
	return IsOK(cmd_resp);
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPSTA_CUR?
//
// Function: Set IP address of station(won't save to Flash)
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPSTA_CUR_Query(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CIPADDR_IN_OUT*	param = (AT_CIPADDR_IN_OUT*)output;
	char			*ptr;
	char			ret_val[32];
	bool			result;
	uint32_t		i;
	uint32_t		ip[4];

	// Check output buffer length
	if(NULL == output || sizeof(AT_CIPADDR_IN_OUT) > output_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT Command Process
	length = At_Command(CMD_CIPSTA_CUR, Cmd_Query);

	result = IsOK(cmd_resp);
	if(!result){
		return false;
	}

	ptr = strstr(cmd_resp, "+CIPSTA_CUR:");
	ptr = strstr(ptr, "ip:");
	// Return string is "+CIPSTA_CUR:ip:\"xx.xx.xx.xx\"\r\n"
	result = AT_Get_Param(ptr, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d.%d.%d.%d", &ip[0], &ip[1], &ip[2], &ip[3]);
	for(i = 0; i < 4; i++){
		param->ip_station[i] = (uint8_t)ip[i];
	}

	ptr++;
	ptr = strstr(ptr, "gateway:");
	// Return string is "+CIPSTA_CUR:gateway:\"xx.xx.xx.xx\"\r\n"
	result = AT_Get_Param(ptr, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d.%d.%d.%d", &ip[0], &ip[1], &ip[2], &ip[3]);
	for(i = 0; i < 4; i++){
		param->ip_gateway[i] = (uint8_t)ip[i];
	}

	ptr++;
	ptr = strstr(ptr, "netmask:");
	// Return string is "+CIPSTA_CUR:netmask:\"xx.xx.xx.xx\"\r\n"
	result = AT_Get_Param(ptr, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d.%d.%d.%d", &ip[0], &ip[1], &ip[2], &ip[3]);
	for(i = 0; i < 4; i++){
		param->ip_netmask[i] = (uint8_t)ip[i];
	}

	// Confirm OK string
	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPSTA_CUR=<IP>[,<gateway>,<netmask>]
//
// Function: Set IP address of station(won't save to Flash)
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPSTA_CUR_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CIPADDR_IN_OUT*	param = (AT_CIPADDR_IN_OUT*)input;

	// Check input buffer length
	if(NULL == input || sizeof(AT_CIPADDR_IN_OUT) > input_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// Create param string
	sprintf(cmd_param, "\"%d.%d.%d.%d\",\"%d.%d.%d.%d\",\"%d.%d.%d.%d\"",
			param->ip_station[0], param->ip_station[1], param->ip_station[2], param->ip_station[3],
			param->ip_gateway[0], param->ip_gateway[1], param->ip_gateway[2], param->ip_gateway[3],
			param->ip_netmask[0], param->ip_netmask[1], param->ip_netmask[2], param->ip_netmask[3]);
	// AT Command Process
	length = At_Command(CMD_CIPSTA_CUR, Cmd_Set);

	// Confirm OK string
	return IsOK(cmd_resp);
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPSTA_DEF?
//
// Function: Set IP address of station(Save to Flash)
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPSTA_DEF_Query(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CIPADDR_IN_OUT*	param = (AT_CIPADDR_IN_OUT*)output;
	char			*ptr;
	char			ret_val[32];
	bool			result;
	uint32_t		i;
	uint32_t		ip[4];

	// Check output buffer length
	if(NULL == output || sizeof(AT_CIPADDR_IN_OUT) > output_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT Command Process
	length = At_Command(CMD_CIPSTA_DEF, Cmd_Query);

	result = IsOK(cmd_resp);
	if(!result){
		return false;
	}

	ptr = strstr(cmd_resp, "+CIPSTA_DEF:");
	ptr = strstr(ptr, "ip:");
	// Return string is "+CIPSTA_CUR:ip:\"xx.xx.xx.xx\"\r\n"
	result = AT_Get_Param(ptr, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d.%d.%d.%d", &ip[0], &ip[1], &ip[2], &ip[3]);
	for(i = 0; i < 4; i++){
		param->ip_station[i] = (uint8_t)ip[i];
	}

	ptr++;
	ptr = strstr(ptr, "gateway:");
	// Return string is "+CIPSTA_CUR:gateway:\"xx.xx.xx.xx\"\r\n"
	result = AT_Get_Param(ptr, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d.%d.%d.%d", &ip[0], &ip[1], &ip[2], &ip[3]);
	for(i = 0; i < 4; i++){
		param->ip_gateway[i] = (uint8_t)ip[i];
	}

	ptr++;
	ptr = strstr(ptr, "netmask:");
	// Return string is "+CIPSTA_CUR:netmask:\"xx.xx.xx.xx\"\r\n"
	result = AT_Get_Param(ptr, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d.%d.%d.%d", &ip[0], &ip[1], &ip[2], &ip[3]);
	for(i = 0; i < 4; i++){
		param->ip_netmask[i] = (uint8_t)ip[i];
	}

	// Confirm OK string
	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPSTA_DEF=<IP>[,<gateway>,<netmask>]
//
// Function: Set IP address of station(Save to Flash)
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPSTA_DEF_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CIPADDR_IN_OUT*	param = (AT_CIPADDR_IN_OUT*)input;

	// Check input buffer length
	if(NULL == input || sizeof(AT_CIPADDR_IN_OUT) > input_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// Create param string
	sprintf(cmd_param, "\"%d.%d.%d.%d\",\"%d.%d.%d.%d\",\"%d.%d.%d.%d\"",
			param->ip_station[0], param->ip_station[1], param->ip_station[2], param->ip_station[3],
			param->ip_gateway[0], param->ip_gateway[1], param->ip_gateway[2], param->ip_gateway[3],
			param->ip_netmask[0], param->ip_netmask[1], param->ip_netmask[2], param->ip_netmask[3]);
	// AT Command Process
	length = At_Command(CMD_CIPSTA_DEF, Cmd_Set);

	// Confirm OK string
	return IsOK(cmd_resp);
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPAP_CUR?
//
// Function: Set IP address of ESP8266 softAP(won't save to Flash)
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPAP_CUR_Query(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CIPADDR_IN_OUT*	param = (AT_CIPADDR_IN_OUT*)output;
	char			*ptr;
	char			ret_val[32];
	bool			result;
	uint32_t		i;
	uint32_t		ip[4];

	// Check output buffer length
	if(NULL == output || sizeof(AT_CIPADDR_IN_OUT) > output_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT Command Process
	length = At_Command(CMD_CIPAP_CUR, Cmd_Query);

	result = IsOK(cmd_resp);
	if(!result){
		return false;
	}

	ptr = strstr(cmd_resp, "+CIPAP_CUR:");
	ptr = strstr(ptr, "ip:");
	// Return string is "+CIPAP_CUR:ip:\"xx.xx.xx.xx\"\r\n"
	result = AT_Get_Param(ptr, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d.%d.%d.%d", &ip[0], &ip[1], &ip[2], &ip[3]);
	for(i = 0; i < 4; i++){
		param->ip_station[i] = (uint8_t)ip[i];
	}

	ptr++;
	ptr = strstr(ptr, "gateway:");
	// Return string is "+CIPAP_CUR:gateway:\"xx.xx.xx.xx\"\r\n"
	result = AT_Get_Param(ptr, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d.%d.%d.%d", &ip[0], &ip[1], &ip[2], &ip[3]);
	for(i = 0; i < 4; i++){
		param->ip_gateway[i] = (uint8_t)ip[i];
	}

	ptr++;
	ptr = strstr(ptr, "netmask:");
	// Return string is "+CIPAP_CUR:netmask:\"xx.xx.xx.xx\"\r\n"
	result = AT_Get_Param(ptr, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d.%d.%d.%d", &ip[0], &ip[1], &ip[2], &ip[3]);
	for(i = 0; i < 4; i++){
		param->ip_netmask[i] = (uint8_t)ip[i];
	}

	// Confirm OK string
	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPAP_CUR=<IP>[,<gateway>,<netmask>]
//
// Function: Set IP address of ESP8266 softAP(won't save to Flash)
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPAP_CUR_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CIPADDR_IN_OUT*	param = (AT_CIPADDR_IN_OUT*)input;

	// Check output buffer length
	if(NULL == input || sizeof(AT_CIPADDR_IN_OUT) > input_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// Create param string
	sprintf(cmd_param, "\"%d.%d.%d.%d\",\"%d.%d.%d.%d\",\"%d.%d.%d.%d\"",
			param->ip_station[0], param->ip_station[1], param->ip_station[2], param->ip_station[3],
			param->ip_gateway[0], param->ip_gateway[1], param->ip_gateway[2], param->ip_gateway[3],
			param->ip_netmask[0], param->ip_netmask[1], param->ip_netmask[2], param->ip_netmask[3]);
	// AT Command Process
	length = At_Command(CMD_CIPAP_CUR, Cmd_Set);

	// Confirm OK string
	return IsOK(cmd_resp);
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPAP_DEF?
//
// Function: Set IP address of ESP8266 softAP(won't save to Flash)
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPAP_DEF_Query(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CIPADDR_IN_OUT*	param = (AT_CIPADDR_IN_OUT*)output;
	char			*ptr;
	char			ret_val[32];
	bool			result;
	uint32_t		i;
	uint32_t		ip[4];

	// Check output buffer length
	if(NULL == output || sizeof(AT_CIPADDR_IN_OUT) > output_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT Command Process
	length = At_Command(CMD_CIPAP_DEF, Cmd_Query);

	result = IsOK(cmd_resp);
	if(!result){
		return false;
	}

	ptr = strstr(cmd_resp, "+CIPAP_DEF:");
	ptr = strstr(ptr, "ip:");
	// Return string is "+CIPAP_DEF:ip:\"xx.xx.xx.xx\"\r\n"
	result = AT_Get_Param(ptr, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d.%d.%d.%d", &ip[0], &ip[1], &ip[2], &ip[3]);
	for(i = 0; i < 4; i++){
		param->ip_station[i] = (uint8_t)ip[i];
	}

	ptr++;
	ptr = strstr(ptr, "gateway:");
	// Return string is "+CIPAP_DEF:gateway:\"xx.xx.xx.xx\"\r\n"
	result = AT_Get_Param(ptr, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d.%d.%d.%d", &ip[0], &ip[1], &ip[2], &ip[3]);
	for(i = 0; i < 4; i++){
		param->ip_gateway[i] = (uint8_t)ip[i];
	}

	ptr++;
	ptr = strstr(ptr, "netmask:");
	// Return string is "+CIPAP_DEF:netmask:\"xx.xx.xx.xx\"\r\n"
	result = AT_Get_Param(ptr, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d.%d.%d.%d", &ip[0], &ip[1], &ip[2], &ip[3]);
	for(i = 0; i < 4; i++){
		param->ip_netmask[i] = (uint8_t)ip[i];
	}

	// Confirm OK string
	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPAP_DEF=<IP>[,<gateway>,<netmask>]
//
// Function: Set IP address of ESP8266 softAP(won't save to Flash)
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPAP_DEF_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CIPADDR_IN_OUT*	param = (AT_CIPADDR_IN_OUT*)input;

	// Check input buffer length
	if(NULL == input || sizeof(AT_CIPADDR_IN_OUT) > input_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// Create param string
	sprintf(cmd_param, "\"%d.%d.%d.%d\",\"%d.%d.%d.%d\",\"%d.%d.%d.%d\"",
			param->ip_station[0], param->ip_station[1], param->ip_station[2], param->ip_station[3],
			param->ip_gateway[0], param->ip_gateway[1], param->ip_gateway[2], param->ip_gateway[3],
			param->ip_netmask[0], param->ip_netmask[1], param->ip_netmask[2], param->ip_netmask[3]);
	// AT Command Process
	length = At_Command(CMD_CIPAP_DEF, Cmd_Set);

	// Confirm OK string
	return IsOK(cmd_resp);
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CWSTARTSMART=<type>
//
// Function: Start SmartConfig
//
//	<type>
//		1 : ESP-Touch
//		2 : AirKiss
//		3 : ESP-Touch + AirKiss
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CWSTARTSMART_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	uint32_t*	type = (uint32_t*)input;

	// Check input buffer length
	if(NULL == input || sizeof(*type) > input_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// Create param string
	sprintf(cmd_param, "%d", type);
	// AT Command Process
	length = At_Command(CMD_CWSTARTSMART, Cmd_Set);

	// Confirm OK string
	return IsOK(cmd_resp);
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CWSTARTSMART
//
// Function: Start SmartConfig
//
//	<type>
//		1 : ESP-Touch
//		2 : AirKiss
//		3 : ESP-Touch + AirKiss
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CWSTARTSMART_Exe(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT Command Process
	length = At_Command(CMD_CWSTARTSMART, Cmd_Execute);

	// Confirm OK string
	return IsOK(cmd_resp);
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CWSTOPSMART
//
// Function: Stop SmartConfig
//
//	<type>
//		1 : ESP-Touch
//		2 : AirKiss
//		3 : ESP-Touch + AirKiss
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CWSTOPSMART_Exe(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT Command Process
	length = At_Command(CMD_CWSTOPSMART, Cmd_Execute);

	// Confirm OK string
	return IsOK(cmd_resp);
}

///////////////////////////////////////////////////////////////////////////////
//
// TCP/IP AT Command Set
//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPSTATUS
//
// Function: Check network connection status
//
//	<stat>
//		2 : Got IP
//		3 : Connected
//		4 : Disconnected
//	<link ID> ID of the connection(0~4), for multi-connect
//	<type> "TCP" or "UDP"
//	<remote IP> remote IP address
//	<remote port> remote port number
//	<local port> ESP8266 local port number
//	<tetype>
//		0 : ESP8266 runs as client
//		1 : ESP8266 runs as server
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPSTATUS_Exe(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CIPSTATUS_OUT*	param = (AT_CIPSTATUS_OUT*)output;
	char				ret_val[20];
	char*				ptr;
	bool				result;

	// Check output buffer length
	if(NULL == output || sizeof(AT_CIPSTATUS_OUT) > output_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT Command Process
	length = At_Command(CMD_CIPSTATUS, Cmd_Execute);

	result = IsOK(cmd_resp);
	if(!result){
		return false;
	}

	// Return string is "STATUS:<stat>\r\n"
	result = AT_Get_Param(cmd_resp, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", &param->stat);

	ptr = strstr(cmd_resp, "+CIPSTATUS:");
	// Return string is "+CIPSTATUS:<link ID>,<type>,<remote_IP>,<remote_port>,<local_port>,<tetype>\r\n"
	// Get <Link ID>
	result = AT_Get_Param(ptr, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", &param->link_id);
	// Get <type>, "TCP" or "UDP"
	result = AT_Get_Param(ptr, 1, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	if(strcmp(ret_val, "TCP") == 0){
		// Type is "TCP"
		param->type = 0;
	}
	else{
		// Type is "UDP"
		param->type = 1;
	}
	// Get <remote IP>
	result = AT_Get_Param(ptr, 2, param->remote_ip, param->remote_ip_len);
	if(!result){
		return false;
	}
	// Get <remote port>
	result = AT_Get_Param(ptr, 3, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", &param->remote_port);
	// Get <local port>
	result = AT_Get_Param(ptr, 4, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", &param->local_port);
	// Get <tetype>
	result = AT_Get_Param(ptr, 5, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", &param->tetype);

	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPSTART
//
// Function: Establish TCP connection
//
//	<link ID> ID of network connection(used for multi-connection)
//	<type> "TCP" or "UDP"
//	<remote IP> remote IP address
//	<remote port> remote port number
//	<TCP keep alive>
//		0 : disable TCP keep-alive
//		1 ~ 7200 : detection time interval, unit : second
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPSTART_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CIPSTART_IN*	param = (AT_CIPSTART_IN*)input;

	// Check input buffer length
	if(NULL == input || sizeof(AT_CIPSTART_IN) > input_len){
		return false;
	}

	// Set "OK", "ERROR", and "ALREADY CONNECT" as Delimiter
	AT_Set_Delimiter(DLMT_Error);

	// TCP
	if(param->type){
		// Multiple connection
		if(multi_conn){
			if(param->TCP.set_keep_alive){
				sprintf(cmd_param, "%d, \"TCP\",\"%s\",%d,%d", param->link_id, param->remote_ip, param->remote_port, param->TCP.keep_alive);
			}
			else{
				sprintf(cmd_param, "%d,\"TCP\",\"%s\",%d", param->link_id, param->remote_ip, param->remote_port);
			}
		}
		// Single connection
		else{
			if(param->TCP.set_keep_alive){
				sprintf(cmd_param, "\"TCP\",\"%s\",%d,%d", param->remote_ip, param->remote_port, param->TCP.keep_alive);
			}
			else{
				sprintf(cmd_param, "\"TCP\",\"%s\",%d", param->remote_ip, param->remote_port);
			}
		}
	}

	// UDP
	else{
		// Multiple connection
		if(multi_conn){
			if(param->UDP.set_local_port){
				sprintf(cmd_param, "%d,\"UDP\",\"%s\",%d,%d,%d",
						param->link_id, param->remote_ip, param->remote_port, param->UDP.local_port, param->UDP.mode);
			}
			else{
				sprintf(cmd_param, "%d,\"UDP\",\"%s\",%d",
						param->link_id, param->remote_ip, param->remote_port);
			}
		}
		// Single connection
		else{
			if(param->UDP.set_local_port){
				sprintf(cmd_param, "\"UDP\",\"%s\",%d,%d,%d",
						param->remote_ip, param->remote_port, param->UDP.local_port, param->UDP.mode);
			}
			else{
				sprintf(cmd_param, "\"UDP\",\"%s\",%d",
						param->remote_ip, param->remote_port);
			}
		}
	}
	// Set "OK" and "ERROR" as Delimiter
	AT_Set_Delimiter(DLMT_Error);
	// AT Command Process
	length = At_Command(CMD_CIPSTART, Cmd_Set);

	// Confirm OK string
	return IsOK(cmd_resp);
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPSEND
//
// Function: Send data
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPSEND_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CIPSEND_IN*	param = (AT_CIPSEND_IN*)input;

	// Check input buffer length
	if(NULL == input || sizeof(AT_CIPSEND_IN) > input_len){
		return false;
	}

	// Multiple connection
	if(multi_conn){
		if(param->show_udp){
			sprintf(cmd_param, "%d,%d,\"%s\",%d", param->link_id, param->length, param->remote_ip, param->remote_port);
		}
		else{
			sprintf(cmd_param, "%d,%d", param->link_id, param->length);
		}
	}
	// Single connection
	else{
		if(param->show_udp){
			sprintf(cmd_param, "%d,\"%s\",%d", param->length, param->remote_ip, param->remote_port);
		}
		else{
			sprintf(cmd_param, "%d", param->length);
		}
	}

	// Set ">" as Delimiter
	AT_Set_Delimiter(DLMT_Send);
	// AT Command Process
	length = At_Command(CMD_CIPSEND, Cmd_Set);

	if(!strchr(cmd_resp, '>')){
		return false;
	}

	send_mode = true;

	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPSENDEX
//
// Function: Send data
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPSENDEX_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CIPSEND_IN*	param = (AT_CIPSEND_IN*)input;

	// Check input buffer length
	if(NULL == input || sizeof(AT_CIPSEND_IN) > input_len){
		return false;
	}

	// Multiple connection
	if(multi_conn){
		if(param->show_udp){
			sprintf(cmd_param, "%d,%d,\"%s\",%d", param->link_id, param->length, param->remote_ip, param->remote_port);
		}
		else{
			sprintf(cmd_param, "%d,%d", param->link_id, param->length);
		}
	}
	// Single connection
	else{
		if(param->show_udp){
			sprintf(cmd_param, "%d,\"%s\",%d", param->length, param->remote_ip, param->remote_port);
		}
		else{
			sprintf(cmd_param, "%d", param->length);
		}
	}

	// Set ">" as Delimiter
	AT_Set_Delimiter(DLMT_Send);
	// AT Command Process
	length = At_Command(CMD_CIPSENDEX, Cmd_Set);

	if(!strchr(cmd_resp, '>')){
		return false;
	}

	send_mode = true;

	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPBUFSTATUS
//
// Function: Check status of TCP-send-buffer
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPBUFSTATUS_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	uint32_t*				link_id = (uint32_t*)input;
	AT_CIPBUFSTATUS_OUT*	param = (AT_CIPBUFSTATUS_OUT*)output;
	char					ret_val[20];
	bool					result;

	// Check input buffer length
	if(NULL == input || sizeof(*link_id) > input_len){
		return false;
	}

	// Check output buffer length
	if(NULL == output || sizeof(AT_CIPBUFSTATUS_OUT) > output_len){
		return false;
	}

	if(multi_conn){
		sprintf(cmd_param, "%d", *link_id);
		// Set "OK" and "ERROR" as Delimiter
		AT_Set_Delimiter(DLMT_Error);
		// AT Command Process
		length = At_Command(CMD_CIPBUFSTATUS, Cmd_Set);
	}
	else{
		// Set "OK" and "ERROR" as Delimiter
		AT_Set_Delimiter(DLMT_Error);
		// AT Command Process
		length = At_Command(CMD_CIPBUFSTATUS, Cmd_Execute);
	}

	result = IsOK(cmd_resp);
	if(!result){
		return false;
	}

	// Returned string is "<next segment ID>, < segment ID of which has sent >, < segment ID of which sent successfully>, <remain buffer size>, <queue number>"
	// Get next segment ID
	result = AT_Get_Param(cmd_resp, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", &param->next_segment);
	// Get segment ID of which has sent
	result = AT_Get_Param(cmd_resp, 1, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", &param->sent_segment);
	// Get segment ID of which sent successfully
	result = AT_Get_Param(cmd_resp, 2, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", &param->successful_segment);
	// Get remain buffer size
	result = AT_Get_Param(cmd_resp, 3, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", &param->remain_buf_size);
	// Get queue number
	result = AT_Get_Param(cmd_resp, 4, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", &param->queue_number);

	return true;
}
///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPCHECKSEQ
//
// Function: Check if specific segment sent successfully or not
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPCHECKSEQ_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CIPCHECKSEQ_IN*	param = (AT_CIPCHECKSEQ_IN*)input;
	bool*				status = (bool*)output;
	uint32_t			index;
	char				ret_val[20];
	bool				result;

	// Check input buffer length
	if(NULL == input || sizeof(AT_CIPCHECKSEQ_IN) > input_len){
		return false;
	}

	// Check input buffer length
	if(NULL == output || sizeof(*status) > output_len){
		return false;
	}

	if(multi_conn){
		sprintf(cmd_param, "%d,%d", param->link_id, param->segment_id);
		index = 2;
	}
	else{
		sprintf(cmd_param, "%d", param->segment_id);
		index = 1;
	}

	// Set "OK" and "ERROR" as Delimiter
	AT_Set_Delimiter(DLMT_Error);
	// AT Command Process
	length = At_Command(CMD_CIPCHECKSEQ, Cmd_Set);
	if(IsOK(cmd_resp)){
		return false;
	}
	// Get status
	result = AT_Get_Param(cmd_resp, index, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}

	if(strcmp(ret_val, "TRUE") == 0){
		*status = true;
	}
	else{
		*status = false;
	}

	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPBUFRESET
//
// Function: Reset segment ID count
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPBUFRESET_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	uint32_t*	link_id = (uint32_t*)input;

	// Check input buffer length
	if(NULL == input || sizeof(*link_id) > input_len){
		return false;
	}

	// Set "OK" and "ERROR" as Delimiter
	AT_Set_Delimiter(DLMT_Error);
	// Multiple connection
	if(multi_conn){
		sprintf(cmd_param, "%d", *link_id);
		// AT+CIPBUFRESET=<link ID>
		length = At_Command(CMD_CIPBUFRESET, Cmd_Set);
	}
	// Single connection
	else{
		// AT+CIPBUFRESET
		length = At_Command(CMD_CIPBUFRESET, Cmd_Execute);
	}

	// Confirm OK string
	return IsOK(cmd_resp);
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPCLOSE
//
// Function: Close TCP or UDP connection
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPCLOSE_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	uint32_t*	link_id = (uint32_t*)input;

	// Check input buffer length
	if(NULL == input || sizeof(*link_id) > input_len){
		return false;
	}

	// Set "OK" and "ERROR" as Delimiter
	AT_Set_Delimiter(DLMT_Error);
	// Multiple connection
	if(multi_conn){
		sprintf(cmd_param, "%d", *link_id);
		// AT+CIPBUFRESET=<link ID>
		length = At_Command(CMD_CIPCLOSE, Cmd_Set);
	}
	// Single connection
	else{
		// AT+CIPBUFRESET
		length = At_Command(CMD_CIPCLOSE, Cmd_Execute);
	}

	// Confirm OK string
	return IsOK(cmd_resp);
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIFSR
//
// Function: Get local IP address
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIFSR_Exe(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CIPADDR_OUT*	param = (AT_CIPADDR_OUT*)output;
	char			ret_val[20];
	bool			result;
	uint32_t		i;
	uint32_t		ip[4];

	// Check output buffer length
	if(NULL == output || sizeof(AT_CIPADDR_OUT) > output_len){
		return false;
	}

	// Set "OK" and "ERROR" as Delimiter
	AT_Set_Delimiter(DLMT_Error);
	// AT+CIFSR
	length = At_Command(CMD_CIFSR, Cmd_Execute);
	result = IsOK(cmd_resp);
	if(!result){
		return false;
	}

	// Get IP address
	result = AT_Get_Param(cmd_resp, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "\"%d.%d.%d.%d\"", &ip[0], &ip[1], &ip[2], &ip[3]);
	for(i = 0; i < 4; i++){
		param->ip_station[i] = (uint8_t)ip[i];
	}

	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPMUX?
//
// Function: Enable multiple connections or not
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPMUX_Query(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	bool*		multi_mode = (bool*)output;
	char		ret_val[8];
	bool		result;

	// Check output buffer length
	if(NULL == output || sizeof(*multi_mode) > output_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT+CIPMUX?
	length = At_Command(CMD_CIPMUX, Cmd_Query);

	result = IsOK(cmd_resp);
	if(!result){
		return false;
	}

	result = AT_Get_Param(cmd_resp, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", multi_mode);

	multi_conn = *multi_mode;

	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPMUX=<mode>
//
// Function: Enable multiple connections or not
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPMUX_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	bool*		multi_mode = (bool*)input;
	bool		result;

	// Check input buffer length
	if(NULL == input || sizeof(*multi_mode) > input_len){
		return false;
	}

	// Set "OK" and "Link is builded" as Delimiter
	AT_Set_Delimiter(DLMT_Linked);
	sprintf(cmd_param, "%d", *multi_mode);
	// AT+CIPMUX?
	length = At_Command(CMD_CIPMUX, Cmd_Set);

	result = IsOK(cmd_resp);
	if(result){
		multi_conn = *multi_mode;
	}

	return result;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPSERVER=<mode>[,<port>]
//
// Function: Configure as TCP server
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPSERVER_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_CIPSERVER_IN*	param = (AT_CIPSERVER_IN*)input;
	bool				result;

	// Check input buffer length
	if(NULL == input || sizeof(AT_CIPSERVER_IN) > input_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// Set port
	if(param->server_mode){
		sprintf(cmd_param, "%d,%d", param->server_mode, param->port);
	}
	// Don't set port
	else{
		sprintf(cmd_param, "%d", param->server_mode);
	}
	// AT+CIPSERVER=<mode>[,<port>]
	length = At_Command(CMD_CIPSERVER, Cmd_Set);

	return IsOK(cmd_resp);
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPMODE?
//
// Function: Set transfer mode
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPMODE_Query(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	bool*		mode = (bool*)output;
	char		ret_val[8];
	bool		result;

	// Check output buffer length
	if(NULL == output || sizeof(*mode) > output_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT+CIPMODE?
	length = At_Command(CMD_CIPMODE, Cmd_Query);

	result = IsOK(cmd_resp);
	if(!result){
		return false;
	}

	result = AT_Get_Param(cmd_resp, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", mode);

	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPMODE=<mode>
//
// Function: Set transfer mode
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPMODE_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	bool*		mode = (bool*)input;
	bool		result;

	// Check input buffer length
	if(NULL == input || sizeof(*mode) > input_len){
		return false;
	}

	// Set "OK" and "Link is builded" as Delimiter
	AT_Set_Delimiter(DLMT_Linked);
	sprintf(cmd_param, "%d", *mode);
	// AT+AT+CIPMODE=<mode>
	length = At_Command(CMD_CIPMODE, Cmd_Set);

	return IsOK(cmd_resp);
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+SAVETRANSLINK=<mode>,<remote IP>,<remote port>[,<type>][,<TCP keep alive>][,<UDP local port>]
//
// Function: Save transparent transmission link to Flash
//
///////////////////////////////////////////////////////////////////////////////
bool AT_SAVETRANSLINK_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_SAVETRANSLINK_IN*	param = (AT_SAVETRANSLINK_IN*)input;
	bool		result;

	// Check input buffer length
	if(NULL == input || sizeof(AT_SAVETRANSLINK_IN) > input_len){
		return false;
	}

	// Set "OK" and "ERROR" as Delimiter
	AT_Set_Delimiter(DLMT_Error);
	// TCP
	if(param->type_tcp){
		if(param->TCP.set_keep_alive){
			sprintf(cmd_param, "%d,\"%s\",%d,\"TCP\",%d", param->mode, param->remote_ip, param->remote_port, param->TCP.keep_alive);
		}
		else{
			sprintf(cmd_param, "%d,\"%s\",%d,\"TCP\"", param->mode, param->remote_ip, param->remote_port);
		}
	}
	// UDP
	else{
		if(param->UDP.set_local_port){
			sprintf(cmd_param, "%d,\"%s\",%d,\"UDP\",%d", param->mode, param->remote_ip, param->remote_port, param->UDP.local_port);
		}
		else{
			sprintf(cmd_param, "%d,\"%s\",%d,\"UDP\"", param->mode, param->remote_ip, param->remote_port);
		}
	}
	// AT+SAVETRANSLINK=<mode>,<remote IP>,<remote port>...
	length = At_Command(CMD_SAVETRANSLINK, Cmd_Set);

	return IsOK(cmd_resp);
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPSTO?
//
// Function: Set TCP server timeout
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPSTO_Query(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	uint32_t*	timeout = (uint32_t*)output;
	char		ret_val[8];
	bool		result;

	// Check output buffer length
	if(NULL == output || sizeof(*timeout) > output_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT+CIPMODE?
	length = At_Command(CMD_CIPSTO, Cmd_Query);

	result = IsOK(cmd_resp);
	if(!result){
		return false;
	}

	// Get timeout value
	result = AT_Get_Param(cmd_resp, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", timeout);

	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPSTO=<timeout>
//
// Function: Set TCP server timeout
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPSTO_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	uint32_t*	timeout = (uint32_t*)input;
	bool		result;

	// Check input buffer length
	if(NULL == input || sizeof(*timeout) > input_len){
		return false;
	}

	// Set "OK" and "Link is builded" as Delimiter
	AT_Set_Delimiter(DLMT_Linked);
	sprintf(cmd_param, "%d", *timeout);
	// AT+CIPSTO=<timeout>
	length = At_Command(CMD_CIPSTO, Cmd_Set);

	return IsOK(cmd_resp);
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+PING=<ip>
//
// Function: Function Ping
//
///////////////////////////////////////////////////////////////////////////////
bool AT_PING_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	AT_PING_IN*	param = (AT_PING_IN*)input;
	bool		result;

	// Check input buffer length
	if(NULL == input || sizeof(AT_PING_IN) > input_len){
		return false;
	}

	// Set "OK" and "ERROR" as Delimiter
	AT_Set_Delimiter(DLMT_Error);
	sprintf(cmd_param, "\"%s\"", param->ip_string);
	// AT+PING=<ip>
	length = At_Command(CMD_PING, Cmd_Set);

	return IsOK(cmd_resp);
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIUPDATE
//
// Function: update through network
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIUPDATE_Exe(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	uint32_t*	status = (uint32_t*)output;
	char		ret_val[8];
	bool		result;

	// Check output buffer length
	if(NULL == output || sizeof(*status) > output_len){
		return false;
	}

	// Set "OK" as Delimiter
	AT_Set_Delimiter(DLMT_Normal);
	// AT+CIUPDATE
	length = At_Command(CMD_CIUPDATE, Cmd_Execute);

	result = IsOK(cmd_resp);
	if(!result){
		return false;
	}

	// Get status value
	result = AT_Get_Param(cmd_resp, 0, ret_val, sizeof(ret_val));
	if(!result){
		return false;
	}
	sscanf(ret_val, "%d", status);

	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// Command : AT+CIPDINFO=<mode>
//
// Function: Show remote IP and port with “+IPD” ( received data from network )
//
///////////////////////////////////////////////////////////////////////////////
bool AT_CIPDINFO_Set(void* input, uint32_t input_len, void* output, uint32_t output_len)
{
	uint32_t	length;
	uint32_t*	mode = (uint32_t*)input;
	bool		result;

	// Check input buffer length
	if(NULL == input || sizeof(*mode) > input_len){
		return false;
	}

	// Set "OK" and "ERROR" as Delimiter
	AT_Set_Delimiter(DLMT_Error);
	sprintf(cmd_param, "%d", *mode);
	// AT+CIPSTO=<timeout>
	length = At_Command(CMD_CIPDINFO, Cmd_Set);

	if(!*mode){
		show_remote_info = false;
	}
	else{
		show_remote_info = true;
	}

	return IsOK(cmd_resp);
}

///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
//		At_Send_Tcp_Data
// 機能：
//		TCP/UDPにおいてデータを送信する。
//　引数：
//		buffer : 送信データバッファのポインタ
//		length : 送信データ長
// 戻り値
//		true - 成功、false - 失敗
//
///////////////////////////////////////////////////////////////////////////////
bool At_Send_Data(uint8_t* buffer, uint32_t length)
{
	uint32_t	status;

	// Check if send mode
	if(!send_mode){
		return false;
	}

	// Set delimiter
	AT_Set_Delimiter(DLMT_Sending);

	// Receive buffer clear
	cmd_resp[0] = '\0';
	// Set receive buffer
	Uart_Set_Receive_Buffer(UART_INDEX, cmd_resp, sizeof(cmd_resp));

	// Send data asynchronously
	Uart_Send_Data_Async(UART_INDEX, buffer, length);

//	do{
//		status = Uart_Get_Status(UART_INDEX);
//	}while(status != eUART_RX);
//	// Wait until receiving completed
//	do{
//		status = Uart_Get_Status(UART_INDEX);
//	}while(status != eUART_IDLE && status != eUART_TIMEOUT);

	// Wait until sending data completed
	while(!Is_AT_Completed()){
		continue;
	}

	// Reset receive buffer
	Uart_Set_Receive_Buffer(UART_INDEX, NULL, 0);

	// Check if sending data successful
	if(!strstr(cmd_resp, "SEND OK")){
		return false;
	}

	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
// 関数名：
//		At_Exit_Send_Mode
// 機能：
//		"+++"を送信して送信データモードから抜ける。
//
///////////////////////////////////////////////////////////////////////////////
bool At_Exit_Send_Mode(void)
{
	bool	result;

	result = At_Send_Data("+++", 3);
	if(!result){
		return false;
	}

	send_mode = false;

	return true;
}
