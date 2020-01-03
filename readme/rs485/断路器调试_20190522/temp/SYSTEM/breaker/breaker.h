#ifndef __BREAKER_H
#define __BREAKER_H
#include "stdio.h"	
#include "sys.h" 

#define BREAKER_TXCMDLINKCNT     20   //发送断路器数据帧的队列长度，暂时设定最大20
#define BREAKER_FRAMELEN     (0x0F)

#define BREAKER_FRAME_HEAD (0x5A)

#define BREAKER_CMD_ReportID (0xA1)   // 断路器主动 上报  ID
#define BREAKER_CMD_ReportIDACK (0xA0)   //回复 A1
#define BREAKER_CMD_OpenBreaker (0xA2)         //网关发送 【 打开电路器 】 命令
#define BREAKER_CMD_OpenBreakerACK (0xA3)         //回复A2
#define BREAKER_CMD_ReadBreaker (0xA4)      //网关 发送 【 查询 】 命令
#define BREAKER_CMD_ReadBreakerACK (0xA5)      //回复 A5
#define BREAKER_CMD_CloseBreaker (0xA6)      // 网关 发送 【 关闭断路 】 器命令
#define BREAKER_CMD_CloseBreakerACK (0xA7)      //回复A6
#define BREAKER_CMD_Calibrate (0xAA)      //发送校准命令
#define BREAKER_CMD_CalibrateACK (0xAB)      // 回复 AA


typedef struct Breaker_UART_TX_node
{
	u8 USART_TX_BUF[BREAKER_FRAMELEN];
}Breaker_UART_TX;  //断路器发送数据帧

typedef struct Breaker_UART_TX_link
{
    Breaker_UART_TX Cmd[BREAKER_TXCMDLINKCNT];
    u8 Head;
    u8 Tail;
    u8 Count;
}Breaker_TX_link;   //发送数据帧队列

typedef struct _Breaker_Frame_type{
		u8 head;
		u8 len;
		u8 cmd;
		
		u8 id1;
		u8 id2;
		u8 id3;
		u8 id4;
		
		u8 voltage;
		u8 current_i;
		u8 current_d;
		u8 power_i;
		u8 power_d;
		u8 Electricity_i;
		u8 Electricity_d;
		
		//u8 error_code;
		u8 crc;
} Breaker_Frame_t;

typedef struct Breaker_Frame_Type
{
    union {
        Breaker_Frame_t frame;
        u8 buf[BREAKER_FRAMELEN];
    }u;
}Breaker_Frame;  //断路器数据帧格式

extern void Display_TxFrameData(u8 *data,u8 len);

extern Breaker_Frame BreakerTxFrame;
extern Breaker_Frame BreakerRxFrame;
extern Breaker_TX_link BreakerTxLink ;

extern u8 Breaker_RxDataProsess(u8 * buf,u8 len);
extern void Breaker_USART_SendData(u8 *buf, u16 len);

void Breaker_ConstructTxCmdFrame(Breaker_Frame *frame,u8 cmd,u8 *session_id,u8 *data,u8 datalen);
u8 Breaker_SendDataToTxLink(Breaker_Frame *frame);

#endif


