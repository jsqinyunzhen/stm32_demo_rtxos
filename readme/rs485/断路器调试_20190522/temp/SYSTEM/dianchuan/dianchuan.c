#include "sys.h"
#include "usart.h"	  
#include <string.h>
#include "dianchuan.h"

DianChuan_Frame DianChuanTxFrame = {0};
DianChuan_Frame DianChuanRxFrame = {0};
DianChuan_TX_link DianChuanTxLink = {0};
DianChuan_Board DianChuanBoard = {0};

u8 DianChuan_AnalysisDataFrame(u8 *FrameBuf , u8 len,DianChuan_Frame *frame)
{
    u8 sum = 0;
    u8 i = 0;
    
    if(FrameBuf == 0 || frame == 0)
    {
        return 0xff;
    }
    
    sum = FrameBuf[1];
    for(i = 2; i < len; i++)
    {
        sum ^= FrameBuf[i];
    }
    
    if(sum != 0)
    {
        return sum;
    }
    
    memset(frame,0,sizeof(DianChuan_Frame));
    frame->sop = FrameBuf[0];
    frame->len = FrameBuf[1];
    frame->cmd = FrameBuf[2];
    memcpy(frame->session_id,FrameBuf+3,6);
    memcpy(frame->data,FrameBuf+9,frame->len-1-6-1);    
    frame->sum = FrameBuf[len-1];

    return 0;
}

/*
��ȡ�豸ÿ���˿ڵ�״̬      ����-->�紨
PORT_NUM	PORT1_STATUS	PORT2_STATUS	��
0x0A	0x02	0x01	��
*/
void DianChuan_RxCmd_GetAllPortStatus(DianChuan_Frame *frame)
{
    u8 portnum = 0;
    u8 i = 0;
    u8 datalen = 0;
    u8 *pPortSt =0;
    if(frame == 0)
    {
        return;
    }

    portnum = frame->data[0];
    pPortSt = frame->data +1;
    datalen = frame->len -1-6-1;
        
    for(i = 0; i < portnum && i < datalen && i < DIANCHUANPORTNUM ;i++ )
    {
        DianChuanBoard.st[i].sst = pPortSt[i];
    }
}


/*
����ɹ���ʼ���     ����-->�紨
PORT	RESULT
0x03	0x01

PORT	��	�û�ѡ��ĳ��˿ںţ�0x02��ʾ2�Ŷ˿ڡ�
RESULT	��	0x01���ɹ� 0x02�����ʧ��

*/
void DianChuan_RxCmd_StartPower(DianChuan_Frame *frame)
{
    u8 portnum = 0;
    u8 result = 0;
    

    portnum = frame->data[0];
    result = frame->data[1];
    //Display_RxFrameData(frame->data,frame->len -1-6-1);
    
    printf("port .... %02X,result = %02X \r\n",portnum,result);
}

/*
//Ͷ���ϱ�             �紨-->����
ģ�鲻����
*/
void DianChuan_RxCmd_CoinReport (DianChuan_Frame *frame){}

/*
�ύ������״̬     �紨-->����
PORT	TIME/POWER	REASON	����	 �˷ѽ��	������
0x03	0x01E0	    0x01   0x01020304	0x01	0x0055

PORT	��	���˿ںţ�0x02��ʾ2�Ŷ˿ڡ�
TIME/POWER	��	�û�ʣ��ĳ��ʱ�����ʣ�������1.����ǳ��ʱ�����Է���Ϊ��λ���� 0x0064��ʾ100���ӡ�2.����ǳ���������0.01�ȣ�ǧ��ʱ��Ϊ��λ����0x0064��ʾ100����λ=1�ȵ������Է���0.01��
����ֵΪ0xFFFF��ʱ��ϵͳ��ȫ���˿��ʾ�ôν��ײ��ɹ���һ��Ϊ�豸�𻵣������û��޷�������磩��
REASON	��	ֹͣ��ԭ��
0x00������ĳ��ʱ�䡢����������
0x01���û��ֶ�ֹͣ���β�ͷ�����ǰ���ֹͣ��ť��
0x02��������ˣ��Զ�ֹͣ
0x03���豸���Ƕ˿ڳ������⣬����ֹͣ
0x04�����������ʳ������վ�ĵ�·���������ʣ��ж����
0x05��ˢ���˷ѽ���	
0x06����ʼ���δ�ӳ����
����	��	0x010 0x02 0x03 0x04 ��ֻ����ˢ���������Ҫ�˷ѵ�����·��Ϳ��ţ�������˶˿���ˢ�������ʹ�4���ֽڿ������˷ѽ��
�˷ѽ��	��	0x01��ʾ  0.1Ԫ  ��ֻ����ˢ���������Ҫ�˷ѵ�����·��ͽ�������˶˿���ˢ�������ʹ�1���ֽ��˷ѽ��
������	��	�����ֽڵĿ�����


��Ҫ�ظ����ݸ��紨
RESULT
0x01
RESULT	��	0x01��ʾģ����ճɹ�

*/
void DianChuan_RxCmd_PowerComplete (DianChuan_Frame *frame)         //
{
    DianChuan_Frame ack = {0};
    u8 data[1]={0x01};
    u8 port = 0;
    u16 time = 0;
    u8 reason = 0;
    u32 cardNumber =0;
    u8 refund = 0;
    u16 cardtype = 0;
    
    port = frame->data[0];
    time = ((u16)frame->data[1]<<8)+frame->data[2];
    reason = frame->data[3];
    cardNumber = ((u32)frame->data[4]<<24)+((u32)frame->data[5]<<16)+((u32)frame->data[6]<<8)+frame->data[7];
    refund = frame->data[8];
    cardtype = ((u16)frame->data[9]<<8)+frame->data[10];

    
    printf("port .... %02X,time = %04X ,reason = %02X\r\n",port,time,reason);
    
    printf("cardNumber .... %04X,refund = %02X ,cardtype = %04X\r\n",cardNumber,refund,cardtype);
    DianChuan_ConstructTxCmdFrame(&ack,frame->cmd,frame->session_id,data,sizeof(data));
    DianChuan_SendDataToTxLink(&ack);
    
}

/*
��ѯ�˿ڵ�ǰ�ĳ��״̬      ����-->�紨
PORT	TIME\POWER	INSTANT POWER
0x03	0x0001	0x0001

PORT	��	0x03��ʾ�˿�3
TIME\POWER	��	��ʾ��һ·���˿ڵ�ʣ����ʱ����߳�������1.����ǳ��ʱ�����Է���Ϊ��λ���� 0x0064��ʾ100���ӡ�2.����ǳ���������0.01�ȣ�ǧ��ʱ��Ϊ��λ����0x0064��ʾ100����λ=1�ȵ������Է���0.01��
��0x00��ʾ���ڳ�磨�������С����ϣ�
INSTANT POWER	��	��һ·��ǰ����˲ʱ���ʣ���λΪ0.1W����0x0001��ʾ0.1W��

*/
void DianChuan_RxCmd_GetPortStatus (DianChuan_Frame *frame)      //
{
    u8 port = 0;
    u16 time = 0;
    u16 power = 0;
    
    port = frame->data[0];
    time = ((u16)frame->data[1]<<8)+frame->data[2];
    power = ((u16)frame->data[3]<<8)+frame->data[4];
    
    printf("port .... %02X,time = %04X ,power = %04X\r\n",port,time,power);
#if 1 //����
        DianChuanBoard.st[port-1].sst = time?0x02:0x01;
        DianChuanBoard.st[port-1].tl = time;
        DianChuanBoard.st[port-1].pow = power;
#endif

}

/*��ѯ�����ܶ�����      ����-->�紨
��������ȥ��ѯ�����洢Ͷ�ҡ�ˢ���������ܶ����ݡ�
CARD_MONEY	COIN_MONEY
0x0001	0x01E0
*/
void DianChuan_RxCmd_GetTotalConsumption (DianChuan_Frame *frame){}

/*IC����Ͷ�ҡ����������     ����-->�紨
NULL
0x01
NULL	��	�̶�Ϊ0x01,��ʾ�ɹ���0x00��ʾʧ��
*/
void DianChuan_RxCmd_SetMaxPower (DianChuan_Frame *frame){}

/*����IC����Ͷ�����Ƿ����     ����-->�紨
NULL
0x01
*/
void DianChuan_RxCmd_SetICEnable (DianChuan_Frame *frame){}

/*Զ��ֹͣĳ���˿ڵĳ��     ����-->�紨
PORT	TIME/POWER
0x03	0x01E0
PORT	��	���˿ںţ�0x02��ʾ2�Ŷ˿ڡ�
TIME\POWER	��	�û�ʣ��ĳ��ʱ����߳���������λ�ο���4

*/
void DianChuan_RxCmd_ClosePower (DianChuan_Frame *frame)      //
{
    u8 port =0;
    u8 time = 0;
    
    port = frame->data[0];
    time = ((u16)frame->data[1]<<8)+frame->data[2];
    
    printf("port .... %02X,time = %04X \r\n",port,time);
}

/*��ȡ�豸IC����Ͷ�ҡ���������á�ˢ���ͳ�����ͣʹ��  ?
MAX_POWER	IC_MONEY	TIME1/POWER1	TIME2/POWER2	TIME3/POWER3	ˢ���˷�ʹ��	������ͣʹ��
0x01F4	0x01	0x0001	0x0001	0x0001	0x00	0x00

MAX_POWER	��	���������ʣ���W���ߣ�Ϊ��λ��
IC_MONEY	��	�û�ÿ��ˢIC����Ҫ���ĵĽ���λΪ�ǡ���0x01��ʾ0.1Ԫ
TIME1/POWER1	��	��һ���ҵĳ��ʱ��(0-999���ӣ����߳�������0-9.99�ȣ�
TIME2/POWER2	��	�ڶ����ҵĳ��ʱ��(0-999���ӣ����߳�������0-9.99�ȣ�
TIME3/POWER3	��	�������ҵĳ��ʱ��(0-999���ӣ����߳�������0-9.99�ȣ�
ˢ���˷�ʹ��	��	0x00�رգ�0x01����
������ͣʹ��	��	0x00�رգ�0x01����

*/
void DianChuan_RxCmd_ReadICCoinPower (DianChuan_Frame *frame){}

/*�ϴ��豸����             �紨-->����
PORT	ERROR_CODE
0x03	0x01

PORT	��	���˿ںţ�0x02��ʾ2�Ŷ˿ڡ�
        ������ĵط��������˿ںţ�����д0xFF��0xff��������
ERROR_CODE	��	������
�������б�

0x01	�˿��������
0x02	���������繦�ʹ���
0x03	��Դ����
0x04	Ԥ��

�ظ����� �̶�
NULL
0x01

*/
void DianChuan_RxCmd_ReportError (DianChuan_Frame *frame)         //
{
    u8 port = 0;
    u8 error_code = 0;
    DianChuan_Frame ack = {0};
    u8 data[1]={0x01};
    
    port = frame->data[0];
    error_code = ((u16)frame->data[1]<<8)+frame->data[2];
    if(port == 0xFF)
    {
        //��������
    }    
    
    printf("port .... %02X,error_code = %02X \r\n",port,error_code);
    DianChuan_ConstructTxCmdFrame(&ack,frame->cmd,frame->session_id,data,sizeof(data));
    DianChuan_SendDataToTxLink(&ack);
}

/*ˢ�����˷ѳɹ���������ʼ�����ģ�鷢�Ϳ��š��۷ѽ������͡��˿� �紨-->����
�豸��ģ�鷢��IC���źͿ۷ѽ������͡��˿ڡ�
*/
void DianChuan_RxCmd_ReportData (DianChuan_Frame *frame){}

/*���߿�������0x0055��ˢ�����̨���Ϳ��š��۷ѽ���                    
�紨-->����
*/
void DianChuan_RxCmd_ReportOnlineData22 (DianChuan_Frame *frame){}
/*��ģ���ٻظ��յ����ݣ�ȷ�ο�ʼ�۷�ָ��    �紨-->����*/
void DianChuan_RxCmd_ReportOnlineData23 (DianChuan_Frame *frame){}

/*���̨�������ͬ��        �紨-->����
�豸��ģ�鷢��IC���š���ͬ��ָ��
*/
void DianChuan_RxCmd_ReportSyncData12 (DianChuan_Frame *frame){}
/*ͬ���ɹ��豸�ڷ���ָ��    �紨-->����*/
void DianChuan_RxCmd_ReportSyncData16 (DianChuan_Frame *frame){}

/*���ó��վ������ͣ��ˢ���Ƿ��˷�   ����-->�紨
NULL
0x01
NULL	��	�̶�Ϊ0x00
*/
void DianChuan_RxCmd_SetAutoFinish (DianChuan_Frame *frame)       //
{
    //DianChuan_Frame ack = {0};
    //u8 data[1]={0x00};
    u8 data =0 ;
    data = frame->data[0];
    
    printf("data .... %d,\r\n",data);
}


/*���ó��վ5���Ʒѹ��ʡ�����   ����-->�紨
null	��	0x01���óɹ��� 0x00 ����ʧ��
*/
void DianChuan_RxCmd_Set5Power (DianChuan_Frame *frame) {}

/*��ȡ�豸5���Ʒѹ��ʡ�����  �紨-->����*/
void DianChuan_RxCmd_Read5Power (DianChuan_Frame *frame){}


/*��ѯ�������г��˿ڵĳ��״̬   ����-->�紨
�ظ����ݣ������ʾ���ݺ��壬�����ʾʾ�����ݣ���
�ܵ���[2B]+�����¶�[1B]+�����·�̵���״̬[2B]+�����·����[20B]+ʣ����ʱ��[20B]

�ܵ���	�����¶�	�����·�̵���״̬	��1·��繦��	������������	��10·��繦��	��1·���ʣ��ʱ��	������������	��10·���ʣ��ʱ��
0x0001	0xff	0x0001	0x0001		0x0001	0x0001		0x0001

�ܵ���	��	0x0001��ʾ 0.1��  ��ʾ���������ܵ���
�����¶�	��	��ʾ���ڻ�����¶ȣ����û�нӴ��������ش�0xff
�����·�̵���״̬	��	Bitλ 0000000001����ʾ��һ·�ڳ�磩0��ʾ�޳�磬1��ʾ��硣
��1·��繦��	��	��һ·��ǰ����˲ʱ���ʣ���λΪ0.1W����0x0001��ʾ0.1W��
��������	��������	��������
��10·��繦��	��	��һ·��ǰ����˲ʱ���ʣ���λΪ0.1W����0x0001��ʾ0.1W��
��1·ʣ����ʱ��	��	��ʾ��һ·���˿ڵ�ʣ����ʱ�䣬�޳��Ϊ0
0x0001��ʾ1����
��������	��������	����������
��10·ʣ����ʱ��	��	��ʾ��һ·���˿ڵ�ʣ����ʱ�䣬�޳��Ϊ0
0x0001��ʾ1����
*/
void DianChuan_RxCmd_GetAllPortPowerStatus (DianChuan_Frame *frame)       //
{
    u16 TotalCurrent = 0;
    u8 temperature = 0;
    u16 relaystatus = 0;
    u16 portpower[DIANCHUANPORTNUM] = {0};
    u16 lasttime[DIANCHUANPORTNUM] = {0};
    u8 i =0;
    
    TotalCurrent = ((u16)frame->data[0]<<8)+frame->data[1];
    temperature = frame->data[2];
    relaystatus = ((u16)frame->data[3]<<8)+frame->data[4];
    printf("TotalCurrent .... %x,temperature = %x, relaystatus = %x \r\n",TotalCurrent,temperature,relaystatus);

    for(i = 0; i < DIANCHUANPORTNUM; i++)
    {
        portpower[i] = ((u16)frame->data[i+5]<<8)+frame->data[i+6];
        lasttime[i] = ((u16)frame->data[i+25]<<8)+frame->data[i+26];
#if 1 //����
        DianChuanBoard.st[i].sst = relaystatus&(1<<i)?0x02:0x01;
        DianChuanBoard.st[i].pow = portpower[i];
        DianChuanBoard.st[i].tl = lasttime[i];
#endif
    }


}

/*��ȡ�豸�İ汾��   ����-->�紨
�汾��
0x0001

�汾��	��	��������汾��

*/
void DianChuan_RxCmd_GetVersonNumber (DianChuan_Frame *frame)       //
{
    u16 version = 0;
    version = ((u16)frame->data[0]<<8)+frame->data[1];
    printf("version .... %d\r\n",version);
}

/*
����汾����   ����-->�紨

�汾��	״̬
0x0001	0x01

�汾��	��	��������汾��
״̬	��	0x01,�������״̬

���߸���Э�飺Xmodem-1k  ������9600
*/
void DianChuan_RxCmd_SetUpdateVersonNumber (DianChuan_Frame *frame)       
{
    u16 version = 0;
    u8 status = 0;
    
    version = ((u16)frame->data[0]<<8)+frame->data[1];
    status = frame->data[2];
    if(status == 0x01)
    {
        
    }
    
    printf("version .... %d,status %d\r\n",version,status);
}

/*
������ѳ��ģʽ����������
NULL
0x01
NULL	��	�̶�Ϊ0x01

*/
void DianChuan_RxCmd_SetFreemodeVolume (DianChuan_Frame *frame)       
{
    u16 data= 0;
    data = frame->data[0];
    
    printf("data .... %d,\r\n",data);
}

/*
������͸��书�ʡ�����ʱ��   ����-->�紨

NULL
0x01
NULL	��	�̶�Ϊ0x01

*/
void DianChuan_RxCmd_SetMinPowerTime (DianChuan_Frame *frame)       
{
    u16 data= 0;
    data = frame->data[0];
    
    printf("data .... %d,\r\n",data);
}


/*
ˢ�����ʱ��   ����-->�紨
*/
void DianChuan_RxCmd_SetOnceICPowerTime (DianChuan_Frame *frame){}

/* 
��ȡ�豸ˢ�����ʱ�䡢���ģʽ���������������书�ʡ�����ʱ��   ����-->�紨
TIME1/POWER1	TIME2/POWER2	TIME3/POWER3	��ѳ��ģʽ	��������	Floating powr	Floating Time
0x0001	0x0001	0x0001	0x00	0x01	0x0001	0x0001
TIME1/PWOER1	��	ˢ��һ�ο��ĳ��ʱ��(0-999���ӣ����߳�������0-9.99�ȣ�
TIME2/POWER2	��	ˢ�ڶ��ο��ĳ��ʱ��(0-999���ӣ����߳�������0-9.99�ȣ�
TIME3/POWER3	��	ˢ�����ο��ĳ��ʱ��(0-999���ӣ����߳�������0-9.99�ȣ�
��ѳ��ģʽ	��	0x00�رգ�0x01��
��������	��	��Χ��1-8��8���
Floating powr	��	��͵ĸ��书�ʣ��������ֵ�ر��������Χ0-99.9   ��0x0001=0.1W
Floating Time	��	������͸��书�ʽ��м���ʱ�䣺120��-999�� 0x0001=1��

*/
void DianChuan_RxCmd_ReadPowrModeVolumeTime (DianChuan_Frame *frame){}
/**/

u8 DianChuan_RXFrameProsess(u8 *FrameBuf , u8 len)
{
    u8 ret = 0;
    
#if 1  //����ʹ��
        Display_RxFrameData(FrameBuf,len);
#endif 
    ret = DianChuan_AnalysisDataFrame(FrameBuf, len, &DianChuanRxFrame);
    
    if(ret != 0)
    {
        return ret;
    }
    

    switch(DianChuanRxFrame.cmd)
    {
        case DIANCHUAN_CMD_GetAllPortStatus:
        {
            DianChuan_RxCmd_GetAllPortStatus(&DianChuanRxFrame);
        }
        break;
        
        case DIANCHUAN_CMD_StartPower:
        {
            DianChuan_RxCmd_StartPower(&DianChuanRxFrame);
            //Display_RxFrameData(FrameBuf,len);
        }
        break;
        case  DIANCHUAN_CMD_CoinReport:        //Ͷ���ϱ�             �紨-->����
        {
             DianChuan_RxCmd_CoinReport (&DianChuanRxFrame);         //Ͷ���ϱ�             �紨-->����
           
        }
        break;
        case  DIANCHUAN_CMD_PowerComplete:         //�ύ������״̬     �紨-->����
        {
             DianChuan_RxCmd_PowerComplete (&DianChuanRxFrame);         //�ύ������״̬     �紨-->����
            //Display_RxFrameData(FrameBuf,len);
        }
        break;

        case  DIANCHUAN_CMD_GetPortStatus:      //��ѯ�˿ڵ�ǰ�ĳ��״̬      ����-->�紨
        {
            DianChuan_RxCmd_GetPortStatus (&DianChuanRxFrame);      //��ѯ�˿ڵ�ǰ�ĳ��״̬      ����-->�紨
            //Display_RxFrameData(FrameBuf,len);
        }
        break;
        case  DIANCHUAN_CMD_GetTotalConsumption:      //��ѯ�����ܶ�����      ����-->�紨
        {
             DianChuan_RxCmd_GetTotalConsumption (&DianChuanRxFrame);      //��ѯ�����ܶ�����      ����-->�紨
            //Display_RxFrameData(FrameBuf,len);
        }
        break;
        case  DIANCHUAN_CMD_SetMaxPower:      //IC����Ͷ�ҡ����������     ����-->�紨
        {
             DianChuan_RxCmd_SetMaxPower (&DianChuanRxFrame);      //IC����Ͷ�ҡ����������     ����-->�紨
            //Display_RxFrameData(FrameBuf,len);
        }
        break;
        case  DIANCHUAN_CMD_SetICEnable:      //����IC����Ͷ�����Ƿ����     ����-->�紨
        {
             DianChuan_RxCmd_SetICEnable (&DianChuanRxFrame);      //����IC����Ͷ�����Ƿ����     ����-->�紨
            //Display_RxFrameData(FrameBuf,len);
        }
        break;
        case  DIANCHUAN_CMD_ClosePower:      //Զ��ֹͣĳ���˿ڵĳ��     ����-->�紨
        {
             DianChuan_RxCmd_ClosePower (&DianChuanRxFrame);      //Զ��ֹͣĳ���˿ڵĳ��     ����-->�紨
            //Display_RxFrameData(FrameBuf,len);
        }
        break;
        case  DIANCHUAN_CMD_ReadICCoinPower:      //��ȡ�豸IC����Ͷ�ҡ���������á�ˢ���ͳ�����ͣʹ��     ?
        {
             DianChuan_RxCmd_ReadICCoinPower (&DianChuanRxFrame);      //��ȡ�豸IC����Ͷ�ҡ���������á�ˢ���ͳ�����ͣʹ��     ?
            //Display_RxFrameData(FrameBuf,len);
        }
        break;
        case  DIANCHUAN_CMD_ReportError:         //�ϴ��豸����             �紨-->����
        {
             DianChuan_RxCmd_ReportError (&DianChuanRxFrame);         //�ϴ��豸����             �紨-->����
            //Display_RxFrameData(FrameBuf,len);
        }
        break;
        case  DIANCHUAN_CMD_ReportData:         //ˢ�����˷ѳɹ���������ʼ�����ģ�鷢�Ϳ��š��۷ѽ������͡��˿�            �紨-->����
        {
             DianChuan_RxCmd_ReportData (&DianChuanRxFrame);         //ˢ�����˷ѳɹ���������ʼ�����ģ�鷢�Ϳ��š��۷ѽ������͡��˿�            �紨-->����
            //Display_RxFrameData(FrameBuf,len);
        }
        break;

        case  DIANCHUAN_CMD_ReportOnlineData22:   // ���Ϳ��š�����ģ��                     �紨-->����
        {
             DianChuan_RxCmd_ReportOnlineData22 (&DianChuanRxFrame);   // ���Ϳ��š�����ģ��                     �紨-->����
            //Display_RxFrameData(FrameBuf,len);
        }
        break;

        case  DIANCHUAN_CMD_ReportOnlineData23:   //��ģ���ٻظ��յ����ݣ�ȷ�ο�ʼ�۷�ָ��    �紨-->����
        {
             DianChuan_RxCmd_ReportOnlineData23 (&DianChuanRxFrame);   //��ģ���ٻظ��յ����ݣ�ȷ�ο�ʼ�۷�ָ��    �紨-->����
            //Display_RxFrameData(FrameBuf,len);
        }
        break;

        case  DIANCHUAN_CMD_ReportSyncData12:       //���̨�������ͬ��        �紨-->����
        {
             DianChuan_RxCmd_ReportSyncData12 (&DianChuanRxFrame);       //���̨�������ͬ��        �紨-->����
           // Display_RxFrameData(FrameBuf,len);
        }
        break;

        case  DIANCHUAN_CMD_ReportSyncData16:       //ͬ���ɹ��豸�ڷ���ָ��    �紨-->����
        {
             DianChuan_RxCmd_ReportSyncData16 (&DianChuanRxFrame);       //ͬ���ɹ��豸�ڷ���ָ��    �紨-->����
            //Display_RxFrameData(FrameBuf,len);
        }
        break;

        case  DIANCHUAN_CMD_SetAutoFinish:       //���ó��վ������ͣ��ˢ���Ƿ��˷�   ����-->�紨
        {
             DianChuan_RxCmd_SetAutoFinish (&DianChuanRxFrame);       //���ó��վ������ͣ��ˢ���Ƿ��˷�   ����-->�紨
            //Display_RxFrameData(FrameBuf,len);
        }
        break;

        case  DIANCHUAN_CMD_Set5Power:       //���ó��վ5���Ʒѹ��ʡ�����   ����-->�紨
        {
             DianChuan_RxCmd_Set5Power (&DianChuanRxFrame);       //���ó��վ5���Ʒѹ��ʡ�����   ����-->�紨
            //Display_RxFrameData(FrameBuf,len);
        }
        break;

        case  DIANCHUAN_CMD_Read5Power:       //��ȡ�豸5���Ʒѹ��ʡ�����  �紨-->����
        {
             DianChuan_RxCmd_Read5Power (&DianChuanRxFrame);       //��ȡ�豸5���Ʒѹ��ʡ�����  �紨-->����
            //Display_RxFrameData(FrameBuf,len);
        }
        break;

        case  DIANCHUAN_CMD_GetAllPortPowerStatus:       //��ѯ�������г��˿ڵĳ��״̬   ����-->�紨
        {
             DianChuan_RxCmd_GetAllPortPowerStatus (&DianChuanRxFrame);       //��ѯ�������г��˿ڵĳ��״̬   ����-->�紨
            //Display_RxFrameData(FrameBuf,len);
        }
        break;

        case  DIANCHUAN_CMD_GetVersonNumber:       //��ȡ�豸�İ汾��   ����-->�紨
        {
             DianChuan_RxCmd_GetVersonNumber (&DianChuanRxFrame);       //��ȡ�豸�İ汾��   ����-->�紨
            //Display_RxFrameData(FrameBuf,len);
        }
        break;

        case  DIANCHUAN_CMD_SetUpdateVersonNumber:       //����汾����   ����-->�紨
        {
             DianChuan_RxCmd_SetUpdateVersonNumber (&DianChuanRxFrame);       //����汾����   ����-->�紨
            //Display_RxFrameData(FrameBuf,len);
        }
        break;

        case  DIANCHUAN_CMD_SetFreemodeVolume:       //������ѳ��ģʽ����������   ����-->�紨
        {
             DianChuan_RxCmd_SetFreemodeVolume (&DianChuanRxFrame);       //������ѳ��ģʽ����������   ����-->�紨
            //Display_RxFrameData(FrameBuf,len);
        }
        break;

        case  DIANCHUAN_CMD_SetMinPowerTime:       //������͸��书�ʡ�����ʱ��   ����-->�紨
        {
             DianChuan_RxCmd_SetMinPowerTime (&DianChuanRxFrame);       //������͸��书�ʡ�����ʱ��   ����-->�紨
           // Display_RxFrameData(FrameBuf,len);
        }
        break;

        case  DIANCHUAN_CMD_SetOnceICPowerTime:       //ˢ�����ʱ��   ����-->�紨
        {
             DianChuan_RxCmd_SetOnceICPowerTime (&DianChuanRxFrame);       //ˢ�����ʱ��   ����-->�紨
           // Display_RxFrameData(FrameBuf,len);
        }
        break;

        case  DIANCHUAN_CMD_ReadPowrModeVolumeTime:       //��ȡ�豸ˢ�����ʱ�䡢���ģʽ���������������书�ʡ�����ʱ��   ����-->�紨
        {
            
             DianChuan_RxCmd_ReadPowrModeVolumeTime (&DianChuanRxFrame);       //��ȡ�豸ˢ�����ʱ�䡢���ģʽ���������������书�ʡ�����ʱ��   ����-->�紨
            //Display_RxFrameData(FrameBuf,len);
        }
        break;
    }
    return 0;
}

/*
SOP	    LEN	    CMD	    SESSION_ID	    DATA	SUM
1Byte   1Byte   1Byte   6Bytes          n       1Byte  

���� 0 �������ݱ������꣬
���� �������������û����
*/
u8 DianChuan_RxDataProsess(u8 * buf,u8 len)
{
    if(buf[0] == DIANCHUAN_RX_SOP || buf[0] == DIANCHUAN_TX_SOP)
    {
        if(buf[1] > 7 && buf[1] + 2 == len)//�ж�����֡�Ƿ�����
        {
            DianChuan_RXFrameProsess(buf,len);
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
SOP	    LEN	    CMD	    SESSION_ID	    DATA	SUM
1Byte   1Byte   1Byte   6Bytes          n       1Byte  
*/
void DianChuan_ConstructTxCmdFrame(DianChuan_Frame *frame,u8 cmd,u8 *session_id,u8 *data,u8 datalen)
{
    u8 i =0;

    
    if(frame == 0)
    {
        return;
    }
    frame->len = 0;
    frame->sop = DIANCHUAN_TX_SOP;
    
    frame->cmd = cmd; 
    frame->len++; 
    frame->sum = cmd;
    
    if(session_id == 0)
    {
        memset(frame->session_id,0,6);
    }
    else
    {
        memcpy(frame->session_id,session_id,6); 
    }
    frame->len += 6;
    frame->sum ^= session_id[0]^session_id[1]^session_id[2]^session_id[3]^session_id[4]^session_id[5];
    
    if(data == 0)
    {
        frame->data[0] = 0;
        frame->len ++;
        frame->sum ^= frame->data[0];
    }
    else
    {
        memcpy(frame->data,data,datalen);
        frame->len += datalen;
        for(i = 0; i < datalen; i++)
        {
            frame->sum ^= data[i];
        }
    }

    frame->len ++;
    frame->sum ^= frame->len;
}

u8 DianChuan_SendDataToTxLink(DianChuan_Frame *frame)//,u8 *buf, u16 len
{
    u16 len = 0;
    
    if(DianChuanTxLink.Count == TXCMDBUFCNT || frame == 0)
    {
        return 0;
    }
    memset(DianChuanTxLink.Cmd[DianChuanTxLink.Tail].USART_TX_BUF,0,USART_REC_LEN);
    
    
    DianChuanTxLink.Cmd[DianChuanTxLink.Tail].USART_TX_BUF[len] = frame->sop; len++;
    DianChuanTxLink.Cmd[DianChuanTxLink.Tail].USART_TX_BUF[len] = frame->len; len++;
    DianChuanTxLink.Cmd[DianChuanTxLink.Tail].USART_TX_BUF[len] = frame->cmd; len++;
    memcpy(DianChuanTxLink.Cmd[DianChuanTxLink.Tail].USART_TX_BUF+len,frame->session_id,6); len+=6;
    memcpy(DianChuanTxLink.Cmd[DianChuanTxLink.Tail].USART_TX_BUF+len,frame->data,frame->len-8); len+=frame->len-8;
    DianChuanTxLink.Cmd[DianChuanTxLink.Tail].USART_TX_BUF[len] = frame->sum; len++;
    
    DianChuanTxLink.Cmd[DianChuanTxLink.Tail].USART_TX_Data_Len = len;
    DianChuanTxLink.Tail = (DianChuanTxLink.Tail+1)%TXCMDBUFCNT;
    DianChuanTxLink.Count ++;
    return len;
}

void DianChuan_SendDataFromTxLink (void)
{
    while(DianChuanTxLink.Count > 0)
    {
        DianChuan_USART_SendData(DianChuanTxLink.Cmd[DianChuanTxLink.Head].USART_TX_BUF,
                                DianChuanTxLink.Cmd[DianChuanTxLink.Head].USART_TX_Data_Len);
        DianChuanTxLink.Cmd[DianChuanTxLink.Head].USART_TX_Data_Len = 0;
        DianChuanTxLink.Head = (DianChuanTxLink.Head+1)%TXCMDBUFCNT;
        DianChuanTxLink.Count --;
    }
}
extern void Display_TxFrameData(u8 *data,u8 len);
void DianChuan_USART_SendData(u8 *buf, u16 len)
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

