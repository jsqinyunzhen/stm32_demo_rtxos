#ifndef __BREAKER_H
#define __BREAKER_H
#include "stdio.h"	
#include "sys.h" 

#define BREAKER_TXCMDLINKCNT     20   //���Ͷ�·������֡�Ķ��г��ȣ���ʱ�趨���20
#define BREAKER_FRAMELEN     (0x0F)

#define BREAKER_FRAME_HEAD (0x5A)

#define BREAKER_CMD_ReportID (0xA1)   // ��·������ �ϱ�  ID
#define BREAKER_CMD_ReportIDACK (0xA0)   //�ظ� A1
#define BREAKER_CMD_OpenBreaker (0xA2)         //���ط��� �� �򿪵�·�� �� ����
#define BREAKER_CMD_OpenBreakerACK (0xA3)         //�ظ�A2
#define BREAKER_CMD_ReadBreaker (0xA4)      //���� ���� �� ��ѯ �� ����
#define BREAKER_CMD_ReadBreakerACK (0xA5)      //�ظ� A5
#define BREAKER_CMD_CloseBreaker (0xA6)      // ���� ���� �� �رն�· �� ������
#define BREAKER_CMD_CloseBreakerACK (0xA7)      //�ظ�A6
#define BREAKER_CMD_Calibrate (0xAA)      //����У׼����
#define BREAKER_CMD_CalibrateACK (0xAB)      // �ظ� AA


typedef struct Breaker_UART_TX_node
{
	u8 USART_TX_BUF[BREAKER_FRAMELEN];
}Breaker_UART_TX;  //��·����������֡

typedef struct Breaker_UART_TX_link
{
    Breaker_UART_TX Cmd[BREAKER_TXCMDLINKCNT];
    u8 Head;
    u8 Tail;
    u8 Count;
}Breaker_TX_link;   //��������֡����

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
}Breaker_Frame;  //��·������֡��ʽ

extern void Display_TxFrameData(u8 *data,u8 len);

extern Breaker_Frame BreakerTxFrame;
extern Breaker_Frame BreakerRxFrame;
extern Breaker_TX_link BreakerTxLink ;

extern u8 Breaker_RxDataProsess(u8 * buf,u8 len);
extern void Breaker_USART_SendData(u8 *buf, u16 len);

void Breaker_ConstructTxCmdFrame(Breaker_Frame *frame,u8 cmd,u8 *session_id,u8 *data,u8 datalen);
u8 Breaker_SendDataToTxLink(Breaker_Frame *frame);

#endif


