#include "sys.h"
#include "usart.h"	  
#include <string.h>
#include "breaker.h"
#include "dianchuan.h"

Breaker_Frame BreakerTxFrame = {0};
Breaker_Frame BreakerRxFrame = {0};
Breaker_TX_link BreakerTxLink = {0};

#define BREAKER_CRC_POLY (0x01)
u8 Breaker_CRC8(u8 *ptr, u8 len)
{
    u8 i; 
    u8 crc=0x00; /* ����ĳ�ʼcrcֵ */ 

    while(len--)
    {
        crc ^= *ptr++;  /* ÿ��������Ҫ������������,������ָ����һ���� */  
        for (i=8; i>0; --i)   /* ������μ�����������һ���ֽ�crcһ�� */  
        { 
            if (crc & 0x80)
                crc = (crc << 1) ^ BREAKER_CRC_POLY;
            else
                crc = (crc << 1);
        }
    }

    return (crc); 
}


u8 Breaker_AnalysisDataFrame(u8 *FrameBuf , u8 len,Breaker_Frame *frame)
{
    u8 sum = 0;
    u8 i = 0;
    
    if(FrameBuf == 0 || frame == 0)
    {
        return 0xff;
    }
    
    if(Breaker_CRC8(FrameBuf,len-1) != FrameBuf[len-1])
    {
        return 0xff;
    }
    
    memcpy(frame->u.buf,FrameBuf,BREAKER_FRAMELEN);
    return 0;
}

void Breaker_RxCmd_ReportID (Breaker_Frame *frame)
{
    Breaker_ConstructTxCmdFrame(&BreakerTxFrame,BREAKER_CMD_ReportIDACK,&(frame->u.frame.id1),&(frame->u.frame.voltage),7);
    Breaker_SendDataToTxLink(&BreakerTxFrame);
}

extern u8 BreakerRXCmd;
extern DianChuan_Board DianChuanBoard;

/*
��������֡���ֱ������֡���д���
*/
u8 Breaker_RXFrameProsess(u8 *FrameBuf , u8 len)
{
    u8 ret = 0;
    
#if 1  //����ʹ��
    BreakerRXCmd = FrameBuf[2];
    Display_RxFrameData(FrameBuf,len);
#endif 
    ret = Breaker_AnalysisDataFrame(FrameBuf, len, &BreakerRxFrame);
    
    if(ret != 0)
    {
        return ret;
    }
    
    switch (BreakerRxFrame.u.frame.cmd)
    {
        case BREAKER_CMD_ReportID: // ��·������ �ϱ�  ID
        {

#if 1 //����
            u8 port = BreakerRxFrame.u.frame.id4;
            u8 status = 0;
            
            status += BreakerRxFrame.u.frame.voltage;
            status += BreakerRxFrame.u.frame.current_i;
            status += BreakerRxFrame.u.frame.current_d;
            status += BreakerRxFrame.u.frame.power_i;
            status += BreakerRxFrame.u.frame.power_d;
            status += BreakerRxFrame.u.frame.Electricity_i;
            status += BreakerRxFrame.u.frame.Electricity_d;
            if(status != 0)
            {
                DianChuanBoard.st[port -1].sst = 0x02;
            }
            else
            {
                DianChuanBoard.st[port -1].sst = 0x01;
            }
#endif
            Breaker_RxCmd_ReportID(&BreakerRxFrame);
        }
        break;
        case BREAKER_CMD_OpenBreakerACK://�ظ� ���ط��� �� �򿪵�·�� �� ����
        {

        }
        break;
        case  BREAKER_CMD_ReadBreakerACK:        //�ظ� ���� ���� �� ��ѯ �� ����
        {
#if 1
            u8 port = BreakerRxFrame.u.frame.id4;
            u8 status = 0;
            
            status += BreakerRxFrame.u.frame.voltage;
            status += BreakerRxFrame.u.frame.current_i;
            status += BreakerRxFrame.u.frame.current_d;
            status += BreakerRxFrame.u.frame.power_i;
            status += BreakerRxFrame.u.frame.power_d;
            status += BreakerRxFrame.u.frame.Electricity_i;
            status += BreakerRxFrame.u.frame.Electricity_d;
            if(status != 0)
            {
                DianChuanBoard.st[port -1].sst = 0x02;
            }
            else
            {
                DianChuanBoard.st[port -1].sst = 0x01;
            }
#endif
        }
        break;
        case  BREAKER_CMD_CloseBreakerACK:         //�ظ� ���� ���� �� �رն�· �� ������
        {
        }
        break;
        case  BREAKER_CMD_CalibrateACK:         //����У׼����
        {
        }
        break;
    }
    return 0;
}

/*

���� 0 �������ݱ������꣬
���� �������������û����
*/
u8 Breaker_RxDataProsess(u8 * buf,u8 len)
{
    if(*buf == BREAKER_FRAME_HEAD)
    {
        if(BREAKER_FRAMELEN == len && *(buf+1) != BREAKER_FRAME_HEAD)//�ж�����֡�Ƿ�����
        {
            Breaker_RXFrameProsess(buf,len);
            len = 0;
        }
        else if( len > BREAKER_FRAMELEN)
        {
            Breaker_RXFrameProsess(buf+(len - BREAKER_FRAMELEN),BREAKER_FRAMELEN);
            len = 0;
        }
    }
    else
    {
        len = 0; //֡ͷ�������֡buf
    }
    
    return len;
}

/*
�������id���飬�����ݣ������·������֡
*/
void Breaker_ConstructTxCmdFrame(Breaker_Frame *frame,u8 cmd,u8 * id,u8 *data,u8 datalen)
{
    u8 i =0;
    if(frame == 0 || 0 == id)
    {
        return;
    }
    frame->u.frame.head = BREAKER_FRAME_HEAD;
    
    frame->u.frame.head = BREAKER_FRAME_HEAD; 
    frame->u.frame.len = BREAKER_FRAMELEN;
    frame->u.frame.cmd = cmd;
    
    frame->u.frame.id1 = id[0];
    frame->u.frame.id2 = id[1];
    frame->u.frame.id3 = id[2];
    frame->u.frame.id4 = id[3];
    
    if(data == 0 || datalen == 0)
    {
        memset(frame->u.buf+7,0,7);
    }
    else
    {
        memcpy(frame->u.buf+7,data,datalen);
    }
    frame->u.frame.crc = Breaker_CRC8(frame->u.buf, sizeof(frame->u.buf)-1);
    //frame->u.frame.crc = 0xF0;
    
}
/*
��������֡�����Ͷ���
*/
u8 Breaker_SendDataToTxLink(Breaker_Frame *frame)
{
    u16 len = 0;
    
    if(BreakerTxLink.Count == BREAKER_TXCMDLINKCNT || frame == 0)
    {
        return 0;
    }
    
    memset(BreakerTxLink.Cmd[BreakerTxLink.Tail].USART_TX_BUF,0,BREAKER_FRAMELEN);
    memcpy(BreakerTxLink.Cmd[BreakerTxLink.Tail].USART_TX_BUF,frame->u.buf,BREAKER_FRAMELEN);
    
    BreakerTxLink.Tail = (BreakerTxLink.Tail+1)%BREAKER_TXCMDLINKCNT;
    BreakerTxLink.Count ++;
    
    return BREAKER_FRAMELEN;
}

/*
�ӷ��Ͷ��з�������֡
*/
void Breaker_SendDataFromTxLink (void)
{
    if(BreakerTxLink.Count > 0)
    {
        Breaker_USART_SendData(BreakerTxLink.Cmd[BreakerTxLink.Head].USART_TX_BUF,BREAKER_FRAMELEN);
        BreakerTxLink.Cmd[BreakerTxLink.Head].USART_TX_BUF[0]=0;
        BreakerTxLink.Head = (BreakerTxLink.Head+1)%BREAKER_TXCMDLINKCNT;
        BreakerTxLink.Count --;
    }
}
void Breaker_USART_SendData(u8 *buf, u16 len)
{
    u8 i =0;
#if 1  //����ʹ��
        Display_TxFrameData(buf,len);
#endif 
    for(i=0; i < len; i++)
    {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
        USART_SendData(USART1,*(buf+i));
    }
}

