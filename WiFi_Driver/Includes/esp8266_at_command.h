/*
 * esp8266_at_command.h
 *
 *  Created on: 2016/01/11
 *      Author: masuda
 */

#ifndef INCLUDES_ESP8266_AT_COMMAND_H_
#define INCLUDES_ESP8266_AT_COMMAND_H_

#include	"uart.h"

#ifdef __cplusplus
extern "C" {
#endif

#define	UART_INDEX	0

typedef enum _AT_CMD_TYPE{
	Cmd_Test = 0,
	Cmd_Query,
	Cmd_Set,
	Cmd_Execute
} AT_CMD_TYPE;

typedef enum _DLMT_STR_TYPE{
	DLMT_Normal = 0,
	DLMT_Error,
	DLMT_Linked,
	DLMT_Send,
	DLMT_Sending
} DLMT_STR_TYPE;

typedef struct _AT_GMR_OUT{
	char*		at_ver;
	uint32_t	at_ver_len;
	char*		sdk_ver;
	uint32_t	sdk_ver_len;
	char*		compile_time;
	uint32_t	compile_time_len;
} AT_GMR_OUT;

typedef struct _AT_UART_IN{
	uint32_t	baudrate;
	uint32_t	databits;
	uint32_t	stopbits;
	uint32_t	parity;
	uint32_t	flow;
} AT_UART_IN;

typedef struct _AT_CWJAP_IN{
	char*		ssid;
	uint32_t	ssid_len;
	char*		passwd;
	uint32_t	passwd_len;
	char*		bssid;
	uint32_t	bssid_len;
} AT_CWJAP_IN;

typedef struct _AT_CWJAP_OUT{
//	uint8_t		error;
	char*		ssid;
	uint32_t	ssid_len;
	char*		bssid;
	uint32_t	bssid_len;
	uint32_t	channel;
	uint32_t	rssi;
} AT_CWJAP_OUT;

typedef struct _AT_CWLAP_IN{
	bool		set_ssid_only;
	char*		ssid;
	uint32_t	ssid_len;
	uint8_t		mac[6];
	uint32_t	channel;
} AT_CWLAP_IN;

typedef struct _AT_CWLAP_OUT{
	uint32_t	ecn;
	char*		ssid;
	uint32_t	ssid_len;
	uint32_t	rssi;
	uint8_t		mac[6];
	uint32_t	channel;
	uint32_t	freq_off;
	uint32_t	freq_cal;
} AT_CWLAP_OUT;

typedef struct _AT_CWSAP_IN_OUT{
	char*		ssid;
	uint32_t	ssid_len;
	char*		passwd;
	uint32_t	passwd_len;
	uint32_t	channel;
	uint32_t	ecn;
	uint32_t	max_conn;
	uint32_t	ssid_hidden;
} AT_CWSAP_IN_OUT;

typedef struct _AT_CWLIF_OUT{
	uint8_t		ip[4];
	uint8_t		mac[6];
} AT_CWLIF_OUT;

typedef struct _AT_CWDHCP_IN{
	uint32_t	mode;
	uint32_t	en;
} AT_CWDHCP_IN;

typedef struct _AT_CWDHCPS_IN_OUT{
	bool		enable;
	uint32_t	lease_time;
	uint32_t	start_ip[4];
	uint32_t	end_ip[4];
} AT_CWDHCPS_IN_OUT;

typedef struct _AT_CIPMAC_IN_OUT{
	uint8_t		mac[6];
} AT_CIPMAC_IN_OUT;

typedef struct _AT_CIPADDR_IN_OUT{
	uint8_t		ip_station[4];
	uint8_t		ip_gateway[4];
	uint8_t		ip_netmask[4];
} AT_CIPADDR_IN_OUT;

typedef struct _AT_CIPADDR_OUT{
	uint8_t		ip_station[4];
} AT_CIPADDR_OUT;

typedef struct _AT_CIPSTATUS_OUT{
	uint32_t	stat;
	uint32_t	link_id;
	uint32_t	type;
	char*		remote_ip;
	uint32_t	remote_ip_len;
	uint32_t	remote_port;
	uint32_t	local_port;
	uint32_t	tetype;
} AT_CIPSTATUS_OUT;

typedef struct _AT_CIPSTART_IN{
	uint32_t	link_id;
	uint32_t	type;
	char*		remote_ip;
	uint32_t	remote_ip_len;
	uint32_t	remote_port;
	union{
		struct{
			bool		set_keep_alive;
			uint32_t	keep_alive;
		} TCP;

		struct{
			bool		set_local_port;
			uint32_t	local_port;
			uint32_t	mode;
		} UDP;
	};
} AT_CIPSTART_IN;

typedef struct _AT_CIPSEND_IN{
	uint32_t		link_id;
	uint32_t	length;
	bool		show_udp;
	char*		remote_ip;
	uint32_t	remote_ip_len;
	uint32_t	remote_port;
} AT_CIPSEND_IN;

typedef struct _AT_CIPBUFSTATUS_OUT{
	uint32_t	next_segment;
	uint32_t	sent_segment;
	uint32_t	successful_segment;
	uint32_t	remain_buf_size;
	uint32_t	queue_number;
} AT_CIPBUFSTATUS_OUT;

typedef struct _AT_CIPCHECKSEQ_IN{
	uint32_t		link_id;
	uint32_t	segment_id;
	bool		status;
} AT_CIPCHECKSEQ_IN;

typedef struct _AT_CIPSERVER_IN{
	bool		server_mode;
	uint32_t	port;
} AT_CIPSERVER_IN;

typedef struct _AT_SAVETRANSLINK_IN{
	uint32_t		mode;
	char*		remote_ip;
	uint32_t	remote_ip_len;
	uint32_t	remote_port;
	bool		type_tcp;
	union{
		struct{
			bool		set_keep_alive;
			uint32_t	keep_alive;
		} TCP;

		struct{
			bool		set_local_port;
			uint32_t	local_port;
		} UDP;
	};
} AT_SAVETRANSLINK_IN;

typedef struct _AT_PING_IN{
	char*		ip_string;
	uint32_t	ip_len;
} AT_PING_IN;

typedef struct _IPD_INFO{
	bool		multi_conn;
	uint32_t	conn_id;
	bool		show_remote;
	char*		remote_ip;
	uint32_t	remote_ip_len;
	uint32_t	remote_port;
	uint32_t	data_len;
	uint8_t*	data;
	uint32_t	data_buf_size;
} IPD_INFO;

///////////////////////////////////////////////////////////////////////////////
//
// AT Command ID Definition
//
///////////////////////////////////////////////////////////////////////////////
// AT Command
#define	CMD_AT				0
#define	CMD_RST				1
#define	CMD_GMR				2
#define	CMD_GSLP			3
#define	CMD_ATE0			4
#define	CMD_ATE1			5
#define	CMD_RESTORE			6
#define	CMD_UART_CUR		7
#define	CMD_UART_DEF		8
#define	CMD_SLEEP			9
#define	CMD_RFPOWER			10
#define	CMD_RFVDD			11
// AT WiFi Command
#define	CMD_CWMODE_CUR		12
#define	CMD_CWMODE_DEF		13
#define	CMD_CWJAP_CUR		14
#define	CMD_CWJAP_DEF		15
#define	CMD_CWLAP			16
#define	CMD_CWQAP			17
#define	CMD_CWSAP_CUR		18
#define	CMD_CWSAP_DEF		19
#define	CMD_CWLIF			20
#define	CMD_CWDHCP_CUR		21
#define	CMD_CWDHCP_DEF		22
#define	CMD_CWDHCPS_CUR		23
#define	CMD_CWDHCPS_DEF		24
#define	CMD_CWAUTOCONN		25
#define	CMD_CIPSTAMAC_CUR	26
#define	CMD_CIPSTAMAC_DEF	27
#define	CMD_CIPAPMAC_CUR	28
#define	CMD_CIPAPMAC_DEF	29
#define	CMD_CIPSTA_CUR		30
#define	CMD_CIPSTA_DEF		31
#define	CMD_CIPAP_CUR		32
#define	CMD_CIPAP_DEF		33
#define	CMD_CWSTARTSMART	34
#define	CMD_CWSTOPSMART		35
// AT TCP/IP Command
#define	CMD_CIPSTATUS		36
#define	CMD_CIPSTART		37
#define	CMD_CIPSEND			38
#define	CMD_CIPSENDEX		39
#define	CMD_CIPSENDBUF		40
#define	CMD_CIPBUFRESET		41
#define	CMD_CIPBUFSTATUS	42
#define	CMD_CIPCHECKSEQ		43
#define	CMD_CIPCLOSE		44
#define	CMD_CIFSR			45
#define	CMD_CIPMUX			46
#define	CMD_CIPSERVER		47
#define	CMD_CIPMODE			48
#define	CMD_SAVETRANSLINK	49
#define	CMD_CIPSTO			50
#define	CMD_CIUPDATE		51
#define	CMD_PING			52
#define	CMD_CIPDINFO		53

void AT_Prepare(void);
///////////////////////////////////////////////////////////////////////////////
//
// AT Command Functions
//
///////////////////////////////////////////////////////////////////////////////
bool AT_Exe(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_RST_Exe(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_GMR_Exe(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_GSLP_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_ATE_Exe(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_RESTORE_Exe(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_UART_CUR_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_UART_DEF_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_SLEEP_Query(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_SLEEP_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_RFPOWER_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_RFVDD_Query(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_RFVDD_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_RFVDD_Exe(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CWMODE_CUR_Query(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CWMODE_CUR_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CWMODE_DEF_Query(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CWMODE_DEF_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CWJAP_CUR_Query(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CWJAP_CUR_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CWJAP_DEF_Query(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CWJAP_DEF_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CWLAP_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CWLAP_Exe(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CWQAP_Exe(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CWSAP_CUR_Query(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CWSAP_CUR_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CWSAP_DEF_Query(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CWSAP_DEF_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CWLIF_Exe(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CWDHCP_CUR_Query(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CWDHCP_CUR_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CWDHCP_DEF_Query(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CWDHCP_DEF_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CWDHCPS_CUR_Query(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CWDHCPS_CUR_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CWDHCPS_DEF_Query(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CWDHCPS_DEF_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CWAUTOCONN_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPSTAMAC_CUR_Query(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPSTAMAC_CUR_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPSTAMAC_DEF_Query(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPSTAMAC_DEF_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPAPMAC_CUR_Query(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPAPMAC_CUR_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPAPMAC_DEF_Query(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPAPMAC_DEF_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPSTA_CUR_Query(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPSTA_CUR_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPSTA_DEF_Query(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPSTA_DEF_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPAP_CUR_Query(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPAP_CUR_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPAP_DEF_Query(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPAP_DEF_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CWSTARTSMART_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CWSTARTSMART_Exe(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CWSTOPSMART_Exe(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPSTATUS_Exe(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPSTART_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPSEND_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPSENDEX_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPBUFSTATUS_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPCHECKSEQ_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPBUFRESET_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPCLOSE_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIFSR_Exe(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPMUX_Query(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPMUX_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPSERVER_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPMODE_Query(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPMODE_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_SAVETRANSLINK_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPSTO_Query(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPSTO_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_PING_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIUPDATE_Exe(void* input, uint32_t input_len, void* output, uint32_t output_len);
bool AT_CIPDINFO_Set(void* input, uint32_t input_len, void* output, uint32_t output_len);

bool At_Send_Data(uint8_t* buffer, uint32_t length);
bool At_Exit_Send_Mode(void);
bool IPD_Get_Command(IPD_INFO* ipd_info);
bool Is_IPD_Received(void);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDES_ESP8266_AT_COMMAND_H_ */
