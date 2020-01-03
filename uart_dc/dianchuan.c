//#include "sys.h"
//#include "usart.h"
#include <string.h>
#include "dianchuan.h"
#include "app4g.h"
#include "project_config.h"
#include "cjson_middleware.h"

DianChuan_Frame DianChuanTxFrame = {0};
DianChuan_Frame DianChuanRxFrame = {0};

/*
SOP	    LEN	    CMD	    SESSION_ID	    DATA	SUM
1Byte   1Byte   1Byte   6Bytes          n       1Byte
*/
void DianChuan_ConstructTxCmdFrame(DianChuan_Frame *frame,u8 cmd,u8 *session_id,u8 *data,u8 datalen)
{
    u8 i =0;


    if(frame == NULL)
    {
        return;
    }
    else
        memset(frame,0,sizeof(DianChuan_Frame));

    frame->sop = DIANCHUAN_TX_SOP;
    frame->len = 0;
    frame->cmd = cmd;
    frame->len++;
    frame->sum = cmd;

    if(session_id == NULL)
    {
        memset(frame->session_id,0,6);
    }
    else
    {
        memcpy(frame->session_id,session_id,6);
    }
    frame->len += 6;
    frame->sum ^= frame->session_id[0]^frame->session_id[1]^frame->session_id[2]^ \
                  frame->session_id[3]^frame->session_id[4]^frame->session_id[5];
    //printf("frame->sum=0x%02x\r\n",frame->sum);
    if(data == NULL)
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

#if 0
DianChuan_TX_link DianChuanTxLink = {0};
DianChuan_Board DianChuanBoard = {0};

u8 DianChuan_AnalysisDataFrame(u8 *FrameBuf, u8 len,DianChuan_Frame *frame)
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

    for(i = 0; i < portnum && i < datalen && i < DIANCHUANPORTNUM ; i++ )
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
void DianChuan_RxCmd_CoinReport (DianChuan_Frame *frame) {}

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
    u8 data[1]= {0x01};
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
void DianChuan_RxCmd_GetTotalConsumption (DianChuan_Frame *frame) {}

/*IC����Ͷ�ҡ����������     ����-->�紨
NULL
0x01
NULL	��	�̶�Ϊ0x01,��ʾ�ɹ���0x00��ʾʧ��
*/
void DianChuan_RxCmd_SetMaxPower (DianChuan_Frame *frame) {}

/*����IC����Ͷ�����Ƿ����     ����-->�紨
NULL
0x01
*/
void DianChuan_RxCmd_SetICEnable (DianChuan_Frame *frame) {}

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
void DianChuan_RxCmd_ReadICCoinPower (DianChuan_Frame *frame) {}

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
    u8 data[1]= {0x01};

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
void DianChuan_RxCmd_ReportData (DianChuan_Frame *frame) {}

/*���߿�������0x0055��ˢ�����̨���Ϳ��š��۷ѽ���
�紨-->����
*/
void DianChuan_RxCmd_ReportOnlineData22 (DianChuan_Frame *frame) {}
/*��ģ���ٻظ��յ����ݣ�ȷ�ο�ʼ�۷�ָ��    �紨-->����*/
void DianChuan_RxCmd_ReportOnlineData23 (DianChuan_Frame *frame) {}

/*���̨�������ͬ��        �紨-->����
�豸��ģ�鷢��IC���š���ͬ��ָ��
*/
void DianChuan_RxCmd_ReportSyncData12 (DianChuan_Frame *frame) {}
/*ͬ���ɹ��豸�ڷ���ָ��    �紨-->����*/
void DianChuan_RxCmd_ReportSyncData16 (DianChuan_Frame *frame) {}

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
void DianChuan_RxCmd_Read5Power (DianChuan_Frame *frame) {}


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
void DianChuan_RxCmd_SetOnceICPowerTime (DianChuan_Frame *frame) {}

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
void DianChuan_RxCmd_ReadPowrModeVolumeTime (DianChuan_Frame *frame) {}
/**/

u8 DianChuan_RXFrameProsess(u8 *FrameBuf, u8 len)
{
    u8 ret = 0;

    ret = DianChuan_AnalysisDataFrame(FrameBuf, len, &DianChuanRxFrame);

    if(ret != 0)
    {
        return ret;
    }

#if 1  //����ʹ��
    Display_RxFrameData(FrameBuf,len);
#endif

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
    if(buf[0] == DIANCHUAN_RX_SOP)
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



u8 DianChuan_SendDataToTxLink(DianChuan_Frame *frame)//,u8 *buf, u16 len
{
    u16 len = 0;

    if(DianChuanTxLink.Count == TXCMDBUFCNT || frame == 0)
    {
        return 0;
    }
    memset(DianChuanTxLink.Cmd[DianChuanTxLink.Tail].USART_TX_BUF,0,USART_REC_LEN);


    DianChuanTxLink.Cmd[DianChuanTxLink.Tail].USART_TX_BUF[len] = frame->sop;
    len++;
    DianChuanTxLink.Cmd[DianChuanTxLink.Tail].USART_TX_BUF[len] = frame->len;
    len++;
    DianChuanTxLink.Cmd[DianChuanTxLink.Tail].USART_TX_BUF[len] = frame->cmd;
    len++;
    memcpy(DianChuanTxLink.Cmd[DianChuanTxLink.Tail].USART_TX_BUF+len,frame->session_id,6);
    len+=6;
    memcpy(DianChuanTxLink.Cmd[DianChuanTxLink.Tail].USART_TX_BUF+len,frame->data,frame->len-8);
    len+=frame->len-8;
    DianChuanTxLink.Cmd[DianChuanTxLink.Tail].USART_TX_BUF[len] = frame->sum;
    len++;

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

void DianChuan_USART_SendData(u8 *buf, u16 len)
{
    u8 i =0;

    for(i=0; i < len; i++)
    {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
        USART_SendData(USART1,*(buf+i));
    }
}
void Display_UpdatePortStatus(DianChuan_Board *port,u8 Portnum)
{
    /*
    u8 x,y = 5;
    u8 Xport = 5;
    u8 Xpower = 100;
    u8 Xstatus = 50;

    u8 Xtime = 160;
    */
    u8 i =0;
    //LCD_ShowString(100,50,200,16,16,"AB");

    //printf("Display_TemperatureTime .... \r\n");
    //LCD_ShowAEChar(Xport,  y,((u16*)"��")[0],16,0);
    //LCD_ShowAEChar(Xport+20,  y,((u16*)"��")[0],16,0);
    //LCD_ShowAEChar(Xpower,   y,((u16*)"��")[0],16,0);
    //LCD_ShowAEChar(Xpower+12,y,((u16*)"��")[0],16,0);
    //LCD_ShowAEChar(Xstatus,   y,((u16*)"״")[0],16,0);
    //LCD_ShowAEChar(Xstatus+16,y,((u16*)"̬")[0],16,0);

    //LCD_ShowAEChar(Xtime,   y,((u16*)"ʱ")[0],16,0);
    //LCD_ShowAEChar(Xtime+16,y,((u16*)"��")[0],16,0);


    for (i =0 ; i<Portnum; i++)
    {
        //   y +=20;
        //LCD_ShowxNum(Xport,y,i,2,16,0);

        /*����*/
        //LCD_ShowxNum(Xpower,y,port->st[i].pow,4,16,0);

        /*ʱ��*/
//LCD_ShowxNum(Xtime,y,port->st[i].tl,4,16,0);

        /*״̬*/
        if(port->st[i].sst == 0x01)
        {
            //LCD_ShowAEChar(Xstatus,   y,((u16*)"��")[0],16,0);
        }
        else if(port->st[i].sst == 0x02)
        {
            //LCD_ShowAEChar(Xstatus,   y,((u16*)"��")[0],16,0);
        }
        else
        {
            //  LCD_ShowAEChar(Xstatus,   y,((u16*)"��")[0],16,0);
            //LCD_ShowAEChar(Xstatus+16,y,((u16*)"��")[0],16,0);
        }


    }

}
void Display_RxFrameData(u8 *data,u8 len)
{
    u8 i = 0;
    u16 x = 5;
    u16 y = 240;
    u8 w = 25;
    u8 buf[]= "                                                  ";
    //L/CD_ShowString(x,y,240,20,16,buf);
    //LCD_ShowString(x,y+20,240,20,16,buf);
    //LCD_ShowString(x,y+40,240,20,16,buf);
    //LCD_ShowString(x,y+60,240,20,16,buf);

    for (i =0 ; i<len; i++)
    {
        sprintf(buf,"%02X ",*(data+i));
        //LCD_ShowString(x,y,w,20,16,buf);
        x += w;
        if(x > 240-w)
        {
            x =5;
            y+=20;
        }
    }
}
void Dianchuan_Test_TX(void)
{
    static int cmdindex =0;
    u8 sessionid[] = {0x00,0x00,0x00,0x00,0x00,0x01};
    u8 cmd[] = {
        DIANCHUAN_CMD_GetAllPortStatus     //��ȡ�豸ÿ���˿ڵ�״̬      ����-->�紨
        , DIANCHUAN_CMD_StartPower          //����ɹ���ʼ���     ����-->�紨
        //, DIANCHUAN_CMD_CoinReport          //Ͷ���ϱ�             �紨-->����
        //, DIANCHUAN_CMD_PowerComplete          //�ύ������״̬     �紨-->����
        , DIANCHUAN_CMD_GetPortStatus       //��ѯ�˿ڵ�ǰ�ĳ��״̬      ����-->�紨
        , DIANCHUAN_CMD_GetTotalConsumption       //��ѯ�����ܶ�����      ����-->�紨
        //, DIANCHUAN_CMD_SetICEnable       //����IC����Ͷ�����Ƿ����     ����-->�紨
        , DIANCHUAN_CMD_SetMaxPower       //IC����Ͷ�ҡ����������     ����-->�紨
        , DIANCHUAN_CMD_ClosePower       //Զ��ֹͣĳ���˿ڵĳ��     ����-->�紨
        //, DIANCHUAN_CMD_ReadICCoinPower       //��ȡ�豸IC����Ͷ�ҡ���������á�ˢ���ͳ�����ͣʹ��     ?
        //, DIANCHUAN_CMD_ReportError          //�ϴ��豸����             �紨-->����
        //, DIANCHUAN_CMD_ReportData          //ˢ�����˷ѳɹ���������ʼ�����ģ�鷢�Ϳ��š��۷ѽ������͡��˿�            �紨-->����
        , DIANCHUAN_CMD_SetAutoFinish        //���ó��վ������ͣ��ˢ���Ƿ��˷�   ����-->�紨
        , DIANCHUAN_CMD_GetAllPortPowerStatus        //��ѯ�������г��˿ڵĳ��״̬   ����-->�紨
        , DIANCHUAN_CMD_GetVersonNumber        //��ȡ�豸�İ汾��   ����-->�紨
        , DIANCHUAN_CMD_SetUpdateVersonNumber        //����汾����   ����-->�紨
        , DIANCHUAN_CMD_SetFreemodeVolume        //������ѳ��ģʽ����������  ����-->�紨
        , DIANCHUAN_CMD_SetMinPowerTime        //������͸��书�ʡ�����ʱ��   ����-->�紨
        , DIANCHUAN_CMD_SetOnceICPowerTime        //ˢ�����ʱ��   ����-->�紨
        , DIANCHUAN_CMD_ReadPowrModeVolumeTime        //��ȡ�豸ˢ�����ʱ�䡢���ģʽ���������������书�ʡ�����ʱ��   ����-->�紨

    };

    switch(cmd[cmdindex])
    {
    case DIANCHUAN_CMD_GetAllPortStatus:
    {
        u8 data[] = {0x00};

        DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetAllPortStatus,sessionid,data,sizeof(data));
        DianChuan_SendDataToTxLink(&DianChuanTxFrame);

    }
    break;

    case DIANCHUAN_CMD_StartPower:
    {
        u8 data[] = {0x01,0x00,0x00,0x00,0x0064};
        DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_StartPower,sessionid,data,sizeof(data));
        DianChuan_SendDataToTxLink(&DianChuanTxFrame);

    }
    break;
    case  DIANCHUAN_CMD_CoinReport:        //Ͷ���ϱ�             �紨-->����
    {
        u8 data[] = {0x01,0x2};
        DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_CoinReport,sessionid,data,sizeof(data));
        DianChuan_SendDataToTxLink(&DianChuanTxFrame);
    }
    break;
    case  DIANCHUAN_CMD_PowerComplete:         //�ύ������״̬     �紨-->����
    {   //������
    }
    break;

    case  DIANCHUAN_CMD_GetPortStatus:      //��ѯ�˿ڵ�ǰ�ĳ��״̬      ����-->�紨
    {
        u8 data[] = {0x01};
        DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetPortStatus,sessionid,data,sizeof(data));
        DianChuan_SendDataToTxLink(&DianChuanTxFrame);
    }
    break;
    case  DIANCHUAN_CMD_GetTotalConsumption:      //��ѯ�����ܶ�����      ����-->�紨
    {
        u8 data[] = {0x00};
        DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetTotalConsumption,sessionid,data,sizeof(data));
        DianChuan_SendDataToTxLink(&DianChuanTxFrame);

    }
    break;
    case  DIANCHUAN_CMD_SetMaxPower:      //IC����Ͷ�ҡ����������     ����-->�紨
    {
        u8 sessionid[] = {0x00,0x00,0x00,0x00,0x00,0x01};
        u8 data[] = {0x01,0xF4,0x01,0x00,0xff,0x00,0xff,0x00,0xff};

        DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_SetMaxPower,sessionid,data,sizeof(data));
        DianChuan_SendDataToTxLink(&DianChuanTxFrame);

    }
    break;
    case  DIANCHUAN_CMD_SetICEnable:      //����IC����Ͷ�����Ƿ����     ����-->�紨
    {
        u8 sessionid[] = {0x00,0x00,0x00,0x00,0x00,0x01};
        u8 data[] = {0x01,0x01};

        DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_SetICEnable,sessionid,data,sizeof(data));
        DianChuan_SendDataToTxLink(&DianChuanTxFrame);

    }
    break;
    case  DIANCHUAN_CMD_ClosePower:      //Զ��ֹͣĳ���˿ڵĳ��     ����-->�紨
    {
        u8 sessionid[] = {0x00,0x00,0x00,0x00,0x00,0x01};
        u8 data[] = {0x01};

        DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_ClosePower,sessionid,data,sizeof(data));
        DianChuan_SendDataToTxLink(&DianChuanTxFrame);

    }
    break;
    case  DIANCHUAN_CMD_ReadICCoinPower:      //��ȡ�豸IC����Ͷ�ҡ���������á�ˢ���ͳ�����ͣʹ��     ?
    {
        u8 data[] = {0x00};
        DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_ReadICCoinPower,sessionid,data,sizeof(data));
        DianChuan_SendDataToTxLink(&DianChuanTxFrame);
    }
    break;
    case  DIANCHUAN_CMD_ReportError:         //�ϴ��豸����             �紨-->����
    {
        //������
        //u8 data[] = {0x00};
        // DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetTotalConsumption,sessionid,data,sizeof(data));
        //DianChuan_SendDataToTxLink(&DianChuanTxFrame);

    }
    break;
    case  DIANCHUAN_CMD_SetAutoFinish:       //���ó��վ������ͣ��ˢ���Ƿ��˷�   ����-->�紨
    {
        u8 data[] = {0x01,0x00};
        DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_SetAutoFinish,sessionid,data,sizeof(data));
        DianChuan_SendDataToTxLink(&DianChuanTxFrame);

    }
    break;

    case  DIANCHUAN_CMD_GetAllPortPowerStatus:       //��ѯ�������г��˿ڵĳ��״̬   ����-->�紨
    {
        u8 data[] = {0x00};
        DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetAllPortPowerStatus,sessionid,data,sizeof(data));
        DianChuan_SendDataToTxLink(&DianChuanTxFrame);
    }
    break;

    case  DIANCHUAN_CMD_GetVersonNumber:       //��ȡ�豸�İ汾��   ����-->�紨
    {
        u8 data[] = {0x00};
        DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetVersonNumber,sessionid,data,sizeof(data));
        DianChuan_SendDataToTxLink(&DianChuanTxFrame);

    }
    break;

    case  DIANCHUAN_CMD_SetUpdateVersonNumber:       //����汾����   ����-->�紨
    {
        u8 data[] = {0x01};
        DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_SetUpdateVersonNumber,sessionid,data,sizeof(data));
        DianChuan_SendDataToTxLink(&DianChuanTxFrame);
    }
    break;

    case  DIANCHUAN_CMD_SetFreemodeVolume:       //������ѳ��ģʽ����������   ����-->�紨
    {
        u8 data[] = {0x00,0x01};
        DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_SetFreemodeVolume,sessionid,data,sizeof(data));
        DianChuan_SendDataToTxLink(&DianChuanTxFrame);
    }
    break;

    case  DIANCHUAN_CMD_SetMinPowerTime:       //������͸��书�ʡ�����ʱ��   ����-->�紨
    {
        u8 data[] = {0x00,0x10,0x00,0x80};
        DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_SetMinPowerTime,sessionid,data,sizeof(data));
        DianChuan_SendDataToTxLink(&DianChuanTxFrame);

    }
    break;

    case  DIANCHUAN_CMD_SetOnceICPowerTime:       //ˢ�����ʱ��   ����-->�紨
    {
        u8 data[] = {0x00,0x10,0x00,0x10,0x00,0x10};
        DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_SetOnceICPowerTime,sessionid,data,sizeof(data));
        DianChuan_SendDataToTxLink(&DianChuanTxFrame);

    }
    break;

    case  DIANCHUAN_CMD_ReadPowrModeVolumeTime:       //��ȡ�豸ˢ�����ʱ�䡢���ģʽ���������������书�ʡ�����ʱ��   ����-->�紨
    {
        u8 data[] = {0x00};
        DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_ReadPowrModeVolumeTime,sessionid,data,sizeof(data));
        DianChuan_SendDataToTxLink(&DianChuanTxFrame);
    }
    break;
    }

    cmdindex = (cmdindex+1)%sizeof(cmd);
}
#endif
void uart3_rec_cmd_response(uint8_t *rcv_buf, uint16_t* rcv_len)
{
    const char*uart_buf = (const char*)RxBuffer3;
    uint16_t uart_buf_len = RxCounter3;
    if(uart_buf ==NULL)
    {
        printf("uart_buf ==NULL\r\n");
        goto out;
    }
    if(uart_buf_len ==0)
    {
        printf("uart_buf_len ==0\r\n");
        goto out;
    }
    else
    {
        //uart2_cpu_printf("uart2 uart_buf_len =%d =%s\r\n",uart_buf_len,uart_buf);
        if(rcv_buf && (uart_buf_len >0))
        {
            //if(RxCounter2 > (*rcv_len))
            //	RxCounter2 = *rcv_len;
            memcpy(rcv_buf,uart_buf,uart_buf_len);
            *rcv_len  = uart_buf_len;
        }
        else
            printf("uart3 uart_buf_len =%d =%s\r\n",uart_buf_len,uart_buf);



    }

out:
    reset_uart3_rx_buffer();
#if (UART3_RX_DMA ==1)
    DMA_Enable(UART3_RX_DMA_CHANNEL,UART3_RX_BUFFER_LEN);//������һ��DMA����
#endif

}
void uart4_rec_cmd_response(uint8_t *rcv_buf, uint16_t* rcv_len)
{
    const char*uart_buf = (const char*)RxBuffer4;
    uint16_t uart_buf_len = RxCounter4;
    if(uart_buf ==NULL)
    {
        printf("uart_buf ==NULL\r\n");
        goto out;
    }
    if(uart_buf_len ==0)
    {
        printf("uart_buf_len ==0\r\n");
        goto out;
    }
    else
    {
        //uart2_cpu_printf("uart2 uart_buf_len =%d =%s\r\n",uart_buf_len,uart_buf);
        if(rcv_buf && (uart_buf_len >0))
        {
            //if(RxCounter2 > (*rcv_len))
            //	RxCounter2 = *rcv_len;
            memcpy(rcv_buf,uart_buf,uart_buf_len);
            *rcv_len  = uart_buf_len;
        }
        else
            printf("uart4 uart_buf_len =%d =%s\r\n",uart_buf_len,uart_buf);



    }

out:
    reset_uart4_rx_buffer();
#if (UART4_RX_DMA ==1)
    DMA_Enable(UART4_RX_DMA_CHANNEL,UART4_RX_BUFFER_LEN);//������һ��DMA����
#endif

}
uint8_t data_temp[100]= {0};

void dc_tx_send_data(uint8_t dc1_2,DianChuan_Frame* pFrame)
{
    /*
    uint8_t sop;
    uint8_t len;
    uint8_t cmd;
    uint8_t session_id[6];
    uint8_t data[247];
    uint8_t sum;
    */
    int i =0;

    uint16_t len = 0;
    if(pFrame)
    {
#if 0
        printf("pFrame->sop=0x%02x\r\n",pFrame->sop);
        printf("pFrame->len=0x%02x\r\n",pFrame->len);
        printf("pFrame->cmd=0x%02x\r\n",pFrame->cmd);
        printf("pFrame->session_id:");
        for(i=0; i<6; i++)
            printf("[%d]=0x%02x,",i,pFrame->session_id[i]);
        printf("\r\n");
        printf("pFrame->data:");
        for(i=0; i<(pFrame->len-8); i++)
            printf("[%d]=0x%02x.",i,pFrame->data[i]);
        printf("\r\n");
        printf("pFrame->sum=0x%02x\r\n",pFrame->sum);
#endif
        data_temp[0] = pFrame->sop;
        data_temp[1] = pFrame->len;
        data_temp[2] = pFrame->cmd;
        memcpy(&data_temp[3],pFrame->session_id,6);
        memcpy(&data_temp[9],pFrame->data,(pFrame->len-8));
        data_temp[9+(pFrame->len-8)]=pFrame->sum;

        len = pFrame->len+2;
    }
    else
    {
        printf("\r\n pFrame is null\r\n");
        return;
    }

    for(i=0; i<len; i++)
    {
        printf("%02x ",data_temp[i]);
    }
    if(dc1_2 ==1)
    {

#if(UART3_TX_DMA == 1)
        printf("\r\n uart3_dma_send_data =%d,%d\r\n",len,get_curtime2());
        uart3_dma_send_data(data_temp,len);
#else
        printf("\r\n uart3_cpu_send_data =%d,%d\r\n",len,get_curtime2());
        for(i=0; i<len; i++)
        {
            uart3_sendchar(data_temp[i]);
        }

#endif
    }
    else if(dc1_2 ==2)
    {
#if(UART4_TX_DMA == 1)
        printf("\r\n uart4_dma_send_data =%d,%d\r\n",len,get_curtime2());
        uart4_dma_send_data(data_temp,len);
#else
        printf("\r\n uart4_cpu_send_data =%d,%d\r\n",len,get_curtime2());
        for(i=0; i<len; i++)
        {
            uart4_sendchar(data_temp[i]);
        }
#endif
    }
    else
    {
        printf("\r\n dc1_2 error\r\n");
        return;
    }


}
void dc_rx_rcv_data(uint8_t dc1_2,DianChuan_Frame* pFrame)
{
    int i =0;
    //uint8_t data_temp[100]={0};
    uint16_t len = 0;
    if(pFrame == NULL)
    {
        printf("\r\n pFrame is null\r\n");
        return;
    }

    if(dc1_2 ==1)
    {

        uart3_rec_cmd_response(data_temp,&len);
        printf("\r\n dc_rx_rcv_data 3 len=%d\r\n",len);
        for(i=0; i<len; i++)
        {
            printf("%02x ",data_temp[i]);
        }
        printf("\r\n");


    }
    else if(dc1_2 ==2)
    {

        uart4_rec_cmd_response(data_temp,&len);
        printf("\r\n dc_rx_rcv_data 4 len=%d\r\n",len);
        for(i=0; i<len; i++)
        {
            printf("%02x ",data_temp[i]);
        }
        printf("\r\n");
    }
    else
    {
        printf("\r\n dc1_2 error\r\n");
        return;
    }
    if(len<11)
        return;
    pFrame->sop = data_temp[0];
    pFrame->len = data_temp[1];
    pFrame->cmd = data_temp[2];
    memcpy(pFrame->session_id,&data_temp[3],6);
    memcpy(pFrame->data,&data_temp[9],(pFrame->len-8));
    pFrame->sum = data_temp[9+(pFrame->len-8)];




}
void gpio_dc_detect_init(void)
{

    GPIO_InitTypeDef GPIO_InitStructure;
    // uint8_t gpio_value = 0;
    //dc2
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;						//????
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //dc1
#if(HW_BOARD_TYPE  ==  HW_BOARD_TYPE_IS_STM32PRODUCT1)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
#elif(HW_BOARD_TYPE  ==  HW_BOARD_TYPE_IS_STM32PRODUCT_V2)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
#endif
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;						//????
    GPIO_Init(GPIOE, &GPIO_InitStructure);
}
void gpio_dc_detect_exti_config(void)
{
    EXTI_InitTypeDef  EXTI_InitStruct;
#if(HW_BOARD_TYPE  ==  HW_BOARD_TYPE_IS_STM32PRODUCT1)
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource15);
    //ѡ��PA�˿�0�� PA0
    EXTI_InitStruct.EXTI_Line = EXTI_Line15;
    //ѡ��Line0
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    //�ж�ģʽ
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    //�����ش���
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    //ʹ���ж������¼��Ĵ���
    EXTI_Init(&EXTI_InitStruct);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource15);
//ѡ��PA�˿�0�� PA0
    EXTI_InitStruct.EXTI_Line = EXTI_Line15;
//ѡ��Line0
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
//�ж�ģʽ
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
//�����ش���
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
//ʹ���ж������¼��Ĵ���
    EXTI_Init(&EXTI_InitStruct);


#elif(HW_BOARD_TYPE  ==  HW_BOARD_TYPE_IS_STM32PRODUCT_V2)

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource15);
    //ѡ��PA�˿�0�� PA0
    EXTI_InitStruct.EXTI_Line = EXTI_Line15;
    //ѡ��Line0
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    //�ж�ģʽ
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    //�����ش���
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    //ʹ���ж������¼��Ĵ���
    EXTI_Init(&EXTI_InitStruct);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource13);
//ѡ��PA�˿�0�� PA0
    EXTI_InitStruct.EXTI_Line = EXTI_Line13;
//ѡ��Line0
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
//�ж�ģʽ
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
//�����ش���
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
//ʹ���ж������¼��Ĵ���
    EXTI_Init(&EXTI_InitStruct);

#endif
}
volatile uint8_t dc1_exist =0;
volatile uint8_t dc2_exist =0;

void gpio_dc_detect_value(void)
{
    uint8_t gpio_value = 0;
#if(HW_BOARD_TYPE  ==  HW_BOARD_TYPE_IS_STM32PRODUCT1)
    gpio_value = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_15);
    if(gpio_value == Bit_SET)
    {
        //printf("DC1 GPIO_Pin_15 is h\r\n");
        dc1_exist = 1;
    }
    else
    {
        //("DC1 GPIO_Pin_15 is l\r\n");
        dc1_exist = 0;
    }


    gpio_value = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15);
    if(gpio_value == Bit_SET)
    {
        //printf("DC2 GPIO_Pin_15 is h\r\n");
        dc2_exist = 1;
    }
    else
    {
        // printf("DC2 GPIO_Pin_15 is l\r\n");
        dc2_exist = 0;
    }

#elif(HW_BOARD_TYPE  ==  HW_BOARD_TYPE_IS_STM32PRODUCT_V2)
    gpio_value = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_13);
    if(gpio_value == Bit_SET)
    {
        // printf("DC1 GPIO_Pin_13 is h\r\n");
        dc1_exist = 1;
    }
    else
    {
        //printf("DC1 GPIO_Pin_13 is l\r\n");
        dc1_exist = 0;
    }


    gpio_value = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15);
    if(gpio_value == Bit_SET)
    {
        //printf("DC2 GPIO_Pin_15 is h\r\n");
        dc2_exist = 1;
    }
    else
    {
        // printf("DC2 GPIO_Pin_15 is l\r\n");
        dc2_exist = 0;
    }


#endif


}
void dc_tx_rx_data_test(void)
{

//    uint32_t flags=0;
    uint8_t data[100] = {0x00};
    uint8_t data_len =1;
    uint8_t cmd =DIANCHUAN_CMD_GetAllPortStatus;
    uint8_t sessionid[] = {0x00,0x00,0x00,0x00,0x00,0x00};
    static uint8_t choise =0;
    uint8_t dc1_2 =2;
    for (;;)
    {
        /*
        uart3_dma_printf("uart3_dma_printf test\r\n");
        uart3_cpu_printf("uart3_cpu_printf test\r\n");
        uart4_cpu_printf("uart4_cpu_printf RxBuffer4=%s RxCounter4=%d\r\n",RxBuffer5,RxCounter5);
        uart4_dma_printf("uart4_dma_printf RxBuffer4=%s RxCounter4=%d\r\n",RxBuffer5,RxCounter5);

        */
        //dc1--uart3 ,dc2--uart4(debug)
        printf("dc %d %d\r\n",dc1_exist,dc2_exist);
        if(dc1_exist == 0)
            dc1_2 =1;
        else if(dc2_exist == 0)
            dc1_2 =2;
        else
            dc1_2 =1;
#if 0
        if(dc1_exist == 0)
            dc1_2 = 1;
        else if(dc2_exist == 0)
            dc1_2 = 2;
        else
            continue;
#endif
        printf("dc test DIANCHUAN_CMD_GetAllPortStatus\r\n");
        if(choise == 0)
        {
            choise = 1;
            DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,cmd,sessionid,data,data_len);
        }
        else
        {
            DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,cmd,NULL,data,data_len);
            choise = 0;
        }

#if 0
        //0xee 0x09 0x01 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0xc7

        pFrame->sop=0xee
                    pFrame->len=0x09
                                pFrame->cmd=0x01
                                    pFrame->session_id:
                                            [0]=0x00,[1]=0x00,[2]=0x00,[3]=0x00,[4]=0x00,[5]=0x00,
                                            pFrame->data:
                                                    [0]=0x00.
                                                            pFrame->sum=0xc7
#endif
                                                                    dc_tx_send_data(dc1_2,&DianChuanTxFrame);

        printf("uart%d rx start=%d from dc1\r\n",(dc1_2==1)?3:4,get_curtime2());
        extern void Delay1ms(__IO uint16_t nTime);
        Delay1ms(100*1);
        printf("uart%d rx end=%d from dc1\r\n",(dc1_2==1)?3:4,get_curtime2());
        dc_rx_rcv_data(dc1_2,&DianChuanRxFrame);
        Delay1ms(100*1);

    }
}
int dc_tx_rx_data(uint8_t dc1_2,uint8_t cmd)
{

    uint32_t flags=0;
    uint8_t data[100] = {0x00};
    uint8_t data_len = 0;
    //uint8_t sessionid[] = {0x00,0x00,0x00,0x00,0x00,0x00};
//	uint8_t dc1_2 =2;
    osEventFlagsId_t ef_id =NULL;
    //dc1--uart3 ,dc2--uart4(debug)
    printf("dc %d %d %d\r\n",dc1_2,dc1_exist,dc2_exist);
    if(dc1_2 == 1)
    {
        if(dc1_exist != 0)
        {
            printf("dc1 not exist\r\n");
            return -1;
        }

    }
    else if(dc1_2 == 2)
    {
        if(dc2_exist != 0)
        {
            printf("dc2 not exist\r\n");
            return -2;
        }

    }
    else
    {
        printf("dc1_2=%d invalid\r\n",dc1_2);
        return -3;

    }

    printf("dc test DIANCHUAN_CMD_GetAllPortStatus\r\n");
    switch(cmd)
    {
    case DIANCHUAN_CMD_GetAllPortStatus:
        data[0] = 0x00;
        data_len =1;

        //ʾ����ģ�鷢�ͣ�EE 09 01 00 00 00 00 00 00 00 08
        //			66 13 01 00 00 00 00 00 00 0a 01 01 01 01 01 01 01 01 01 01 18  (�˿ڿ��� ��
        //	  ģ����գ�66 13 01 00 00 00 00 00 00 0A 02 01 01 01 01 01 01 01 01 01 1B ��1�˿����ڳ�磬�����˿ڿ��� ��
        break;
    case DIANCHUAN_CMD_StartPower:
        data[0] = 0x00;//port num
        data[1] = 0x00;
        data[2] = 0x00;
        data[3] = 0x01;//TIME/POWER
        data[4] = 0x64;//TIME/POWER
        data_len =5;
        //0xee 0x0d 0x02 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x01 0x64 0x6a
        //0x66 0x0a 0x02 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x01 0x09
        break;
    case DIANCHUAN_CMD_GetPortStatus:
        data[0] = 0x00;//port num
        data_len =1;
        //0xee 0x09 0x06 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x0f
        //0xee 0x0c 0x05 0x30 0x30 0x30 0x30 0x30 0x30 0x0a 0x00 0x00 0x00 0x03
        break;
    case DIANCHUAN_CMD_GetTotalConsumption:
        data[0] = 0x00;//port num
        data_len =1;
        //0xee 0x09 0x07 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x0e
        //0x66 0x0c 0x07 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x0b
        break;
    case DIANCHUAN_CMD_GetVersonNumber:
        data[0] = 0x00;//port num
        data_len =1;
        //0xee 0x09 0x07 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x0e
        //0x66 0x0c 0x07 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x0b
        break;


    default:
        printf("cmd=0x%02x invalid\r\n",cmd);
        return -3;

    }
    DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,cmd,NULL,data,data_len);

    printf("uart3/4 tx start=%d\r\n",get_curtime2());

    dc_tx_send_data(dc1_2,&DianChuanTxFrame);

    if(dc1_2 == 1)
    {
#if(UART3_TX_DMA == 1)
        flags = EVENT_FLAGS_UART3_TX_COMPLETE|EVENT_FLAGS_UART3;
#else
        flags = EVENT_FLAGS_UART3;
        printf("uart3 tx end=%d\r\n",get_curtime2());
#endif
        ef_id = evt_id_uart3;
    }
    else
    {
#if(UART4_TX_DMA == 1)
        flags = EVENT_FLAGS_UART4_TX_COMPLETE|EVENT_FLAGS_UART4;
#else
        flags = EVENT_FLAGS_UART4;
        printf("uart4 tx end=%d\r\n",get_curtime2());
#endif
        ef_id = evt_id_uart4;
    }
    printf("*****dc_tx_rx_data wait flags =0x%08x %d\r\n",flags,get_curtime2());
    flags = osEventFlagsWait(ef_id, flags, osFlagsWaitAll, UART3_RX_COMMAND_TIMEOUT);

    if((int32_t)flags == osError)
    {
        printf("%s osError\r\n",__func__);
        return -4;
    }
    else if((int32_t)flags == osErrorTimeout)
    {
        printf("%s osErrorTimeout=%d\r\n",__func__,get_curtime2());
        return -5;
    }

#if(UART3_TX_DMA == 1)
    if((flags&EVENT_FLAGS_UART3_TX_COMPLETE)  == EVENT_FLAGS_UART3_TX_COMPLETE)
    {
        printf("uart3 tx end\r\n");

    }
#endif
#if(UART4_TX_DMA == 1)
    if((flags&EVENT_FLAGS_UART4_TX_COMPLETE)  == EVENT_FLAGS_UART4_TX_COMPLETE)
    {
        printf("uart4 tx end\r\n");

    }
#endif
#if 0

    if(dc1_2 == 1)
    {
        flags = EVENT_FLAGS_UART3;

    }
    else
    {
        flags = EVENT_FLAGS_UART4;

    }


    flags = osEventFlagsWait(ef_id, flags, osFlagsWaitAll, UART3_RX_COMMAND_TIMEOUT);
    printf("*****%s osEventFlagsWait flags =0x%08x\r\n",__func__,flags);

    if((int32_t)flags == osError)
    {
        printf("%s osError\r\n",__func__);
        return -4;
    }
    else if((int32_t)flags == osErrorTimeout)
    {
        printf("%s osErrorTimeout\r\n",__func__);
        return -5;
    }
#endif
    // else
    //a {
    printf("uart%d rx start=%d from dc\r\n",(dc1_2==1)?3:4,get_curtime2());

    if((flags&EVENT_FLAGS_UART3)  == EVENT_FLAGS_UART3)
    {
        printf("uart3 rx end\r\n");
        dc_rx_rcv_data(dc1_2,&DianChuanRxFrame);

    }
    else if((flags&EVENT_FLAGS_UART4)  == EVENT_FLAGS_UART4)
    {
        printf("uart4 rx end\r\n");
        dc_rx_rcv_data(dc1_2,&DianChuanRxFrame);

    }
    else
    {
        printf("dc1_2=%d invalid flags=0x%08x\r\n",dc1_2,flags);
        return -6;

    }
    // }
    return 0;



}
int dc_com_data(uint8_t dc1_2,uint8_t cmd,uint8_t* pdata,uint8_t data_len)
{

    uint32_t flags=0;
    //uint8_t data[100] = {0x00};
    // uint8_t data_len = 0;
    //uint8_t sessionid[] = {0x00,0x00,0x00,0x00,0x00,0x00};
//	uint8_t dc1_2 =2;
    osEventFlagsId_t ef_id =NULL;
    //dc1--uart3 ,dc2--uart4(debug)
    printf("dc %d %d %d\r\n",dc1_2,dc1_exist,dc2_exist);
    if(dc1_2 == 1)
    {
        if(dc1_exist != 0)
        {
            printf("dc1 not exist\r\n");
            return -1;
        }

    }
    else if(dc1_2 == 2)
    {
        if(dc2_exist != 0)
        {
            printf("dc2 not exist\r\n");
            return -2;
        }

    }
    else
    {
        printf("dc1_2=%d invalid\r\n",dc1_2);
        return -3;

    }


    DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,cmd,NULL,pdata,data_len);

    printf("uart3/4 tx start=%d\r\n",get_curtime2());

    dc_tx_send_data(dc1_2,&DianChuanTxFrame);

    if(dc1_2 == 1)
    {
#if(UART3_TX_DMA == 1)
        flags = EVENT_FLAGS_UART3_TX_COMPLETE|EVENT_FLAGS_UART3;
#else
        flags = EVENT_FLAGS_UART3;
        printf("uart3 tx end=%d\r\n",get_curtime2());
#endif
        ef_id = evt_id_uart3;
    }
    else
    {
#if(UART4_TX_DMA == 1)
        flags = EVENT_FLAGS_UART4_TX_COMPLETE|EVENT_FLAGS_UART4;
#else
        flags = EVENT_FLAGS_UART4;
        printf("uart4 tx end=%d\r\n",get_curtime2());
#endif
        ef_id = evt_id_uart4;
    }
    printf("*****dc_tx_rx_data wait flags =0x%08x %d\r\n",flags,get_curtime2());
    flags = osEventFlagsWait(ef_id, flags, osFlagsWaitAll, UART3_RX_COMMAND_TIMEOUT);

    if((int32_t)flags == osError)
    {
        printf("%s osError\r\n",__func__);
        return -4;
    }
    else if((int32_t)flags == osErrorTimeout)
    {
        printf("%s osErrorTimeout=%d\r\n",__func__,get_curtime2());
        return -5;
    }

#if(UART3_TX_DMA == 1)
    if((flags&EVENT_FLAGS_UART3_TX_COMPLETE)  == EVENT_FLAGS_UART3_TX_COMPLETE)
    {
        printf("uart3 tx end\r\n");

    }
#endif
#if(UART4_TX_DMA == 1)
    if((flags&EVENT_FLAGS_UART4_TX_COMPLETE)  == EVENT_FLAGS_UART4_TX_COMPLETE)
    {
        printf("uart4 tx end\r\n");

    }
#endif
#if 0

    if(dc1_2 == 1)
    {
        flags = EVENT_FLAGS_UART3;

    }
    else
    {
        flags = EVENT_FLAGS_UART4;

    }


    flags = osEventFlagsWait(ef_id, flags, osFlagsWaitAll, UART3_RX_COMMAND_TIMEOUT);
    printf("*****%s osEventFlagsWait flags =0x%08x\r\n",__func__,flags);

    if((int32_t)flags == osError)
    {
        printf("%s osError\r\n",__func__);
        return -4;
    }
    else if((int32_t)flags == osErrorTimeout)
    {
        printf("%s osErrorTimeout\r\n",__func__);
        return -5;
    }
#endif
    // else
    //a {
    printf("uart%d rx start=%d from dc\r\n",(dc1_2==1)?3:4,get_curtime2());

    if((flags&EVENT_FLAGS_UART3)  == EVENT_FLAGS_UART3)
    {
        printf("uart3 rx end\r\n");
        dc_rx_rcv_data(dc1_2,&DianChuanRxFrame);


    }
    else if((flags&EVENT_FLAGS_UART4)  == EVENT_FLAGS_UART4)
    {
        printf("uart4 rx end\r\n");
        dc_rx_rcv_data(dc1_2,&DianChuanRxFrame);

    }
    else
    {
        printf("dc1_2=%d invalid flags=0x%08x\r\n",dc1_2,flags);
        return -6;

    }
    // }
    return 0;



}
void dc_port_status(void)
{
    uint8_t data[2]= {0};
    uint8_t data_len =0;
    uint8_t dc1_2 = 1;
    int ret =0;
    int i =0;
    DianChuan_Frame *pFrame = &DianChuanRxFrame;

    data[0] = 0x00;
    data_len =1;

    ret = dc_com_data( dc1_2,DIANCHUAN_CMD_GetAllPortStatus,data, data_len);
    if(ret ==0)
    {


        printf("pFrame->sop=0x%02x\r\n",pFrame->sop);
        printf("pFrame->len=0x%02x\r\n",pFrame->len);
        printf("pFrame->cmd=0x%02x\r\n",pFrame->cmd);
        printf("pFrame->session_id:");
        for(i=0; i<6; i++)
            printf("session_id[%d]=0x%02x ",i,pFrame->session_id[i]);
        printf("\r\n");
        printf("pFrame->data:");
        for(i=0; i<(pFrame->len-8); i++)
            printf("data[%d]=0x%02x ",i,pFrame->data[i]);
        printf("\r\n");
        printf("pFrame->sum=0x%02x\r\n",pFrame->sum);

        printf("port num=%d",pFrame->data[0]);


        for(i=0; i<pFrame->data[0]; i++)
        {
            printf("data[%d]=0x%02x ",i+1,pFrame->data[i+1]);
            charge_Info.dc_port_status[i]= (Dc_Port_Status)pFrame->data[i+1];
        }
        printf("\r\n");
    }
    else
    {
        for(i=0; i<CHARGE_NUM; i++)
        {

            charge_Info.dc_port_status[i]=PORT_FREE;
        }

    }
}
//0xee 0x0d 0x02 0x00 0x00 0x00 0x00 0x00 0x00 0x01 0x00 0x00 0x00 0x01 0x0f
//0x66 0x0a 0x02 0x00 0x00 0x00 0x00 0x00 0x00 0x01 0x01 0x08
int dc_start_power(uint8_t port_num,uint16_t time_minute)
{
    uint8_t data[5]= {0};
    uint8_t data_len =0;
    uint8_t dc1_2 = 0;

    int ret =0;

    data[0] = port_num;//port num
    data[1] = 0x00;
    data[2] = 0x00;

    data[3] = (time_minute>>8) &0xff;//TIME/POWER
    data[4] = time_minute &0xff;//TIME/POWER
    data_len =5;
    if(((int)port_num >= 0) && (port_num<=9))
        dc1_2 =1;
    else if((port_num >=10) && (port_num<=19))
        dc1_2 = 2;
    else
        return -1;
    ret = dc_com_data( dc1_2,DIANCHUAN_CMD_StartPower,data, data_len);
    return ret;

}
int dc_stop_power(uint8_t port_num)
{
    uint8_t data[2]= {0};
    uint8_t data_len =0;
    uint8_t dc1_2 = 0;
    int ret =0;
    data[0] = port_num;//port num
    data_len =1;
    if(((int)port_num >= 0) && (port_num<=9))
        dc1_2 =1;
    else if((port_num >=10) && (port_num<=19))
        dc1_2 = 2;
    else
        return -1;


    ret = dc_com_data( dc1_2,DIANCHUAN_CMD_ClosePower,data, data_len);

    return ret;

}

int dc_port_x_status(uint8_t port_num,uint16_t* pport_status)
{
    uint8_t data[2]= {0};
    uint8_t data_len =0;
    uint8_t dc1_2 = 1;
    int ret =0;
//    int i =0;
    DianChuan_Frame *pFrame = &DianChuanRxFrame;
    uint16_t port_status =0;

    if(((int)port_num >= 0) && (port_num<=9))
        dc1_2 =1;
    else if((port_num >=10) && (port_num<=19))
        dc1_2 = 2;
    else
        return -1;

    data[0] = port_num;
    data_len =1;

    ret = dc_com_data( dc1_2,DIANCHUAN_CMD_GetPortStatus,data, data_len);
    if(ret ==0)
    {
#if 0
        printf("pFrame->sop=0x%02x\r\n",pFrame->sop);
        printf("pFrame->len=0x%02x\r\n",pFrame->len);
        printf("pFrame->cmd=0x%02x\r\n",pFrame->cmd);
        printf("pFrame->session_id:");
        for(i=0; i<6; i++)
            printf("session_id[%d]=0x%02x ",i,pFrame->session_id[i]);
        printf("\r\n");
        printf("pFrame->data:");
        for(i=0; i<(pFrame->len-8); i++)
            printf("data[%d]=0x%02x ",i,pFrame->data[i]);
        printf("\r\n");
        printf("pFrame->sum=0x%02x\r\n",pFrame->sum);
#endif
        if(port_num ==pFrame->data[0] )
        {

            port_status= (pFrame->data[1]<<8)|pFrame->data[2];
            printf("left time=%d\r\n",port_status);
            if(port_status >0)
            {
                //charge_Info.charge_left_tm[port_num-1] = port_status;
            }
            printf("INSTANT  power(*0.1W)=%d\r\n",(pFrame->data[3]<<8)|pFrame->data[4]);
            charge_Info.charge_cur_pow[port_num-1] = (pFrame->data[3]<<8)|pFrame->data[4];
            if(pport_status)
            {
                if((charge_Info.charge_cur_pow[port_num-1]*0.1) <5)
                    //if(charge_Info.charge_cur_pow[port_num-1] == 0)
                    *pport_status = 0;
                else
                    *pport_status = 1;
            }
            return 0;

        }
        else
        {
            //printf("rec port num error=%d =%d\r\n",ch,pFrame->data[0]);
            printf("%s rec port num error=%d %d\r\n",__func__,port_num,pFrame->data[0]);
            return 1;
        }

//        printf("\r\n");
    }
    return 1;
}
//0xee 0x0d 0x02 0x00 0x00 0x00 0x00 0x00 0x00 0x01 0x00 0x00 0x00 0x01 0x0f
//0x66 0x0a 0x02 0x00 0x00 0x00 0x00 0x00 0x00 0x01 0x01 0x08
int dc_set_max_power(uint8_t dc1_2,uint16_t max_power)
{
    uint8_t data[9]= {0};
    uint8_t data_len =0;
    //uint8_t dc1_2 = 0;

    int ret =0;
    //    u8 sessionid[] = {0x00,0x00,0x00,0x00,0x00,0x01};
    //   u8 data[] = {0x01,0xF4,0x01,0x00,0xff,0x00,0xff,0x00,0xff};
    DianChuan_Frame *pFrame = &DianChuanRxFrame;

    data[0] = (max_power>>8) &0xff;//TIME/POWER
    data[1] = max_power &0xff;//TIME/POWER
    data[2] = 0x00;//IC MONEY
    data[3] = 0;
    data[4] = 0xff;
    data[5] = 0;
    data[6] = 0xff;
    data[7] = 0;
    data[8] = 0xff;


    data_len =9;

    ret = dc_com_data( dc1_2,DIANCHUAN_CMD_SetMaxPower,data, data_len);
    if(0x01 ==pFrame->data[0] )
    {
        printf("dc_set_max_power ok\r\n");
    }
    else
    {
        printf("dc_set_max_power fail=0x%02x\r\n",pFrame->data[0]);
    }
    return ret;

}
int dc_get_total_consumption(uint8_t dc1_2)
{
    uint8_t data[2]= {0};
    uint8_t data_len =0;
    //uint8_t dc1_2 = 0;
    int ret =0;

    data[0] = 0x00;//port num
    data_len =1;



    ret = dc_com_data( dc1_2,DIANCHUAN_CMD_GetTotalConsumption,data, data_len);
//0x66 0x0c 0x07 0x00 0x00 0x00 0x00 0x00  0x00  0x00 0x00 0x00 0x00 0x0b
    return ret;

}
int dc_get_maxpower_charge_finish_stopen(uint8_t dc1_2)
{
    uint8_t data[2]= {0};
    uint8_t data_len =0;
    //uint8_t dc1_2 = 0;
    int ret =0;
    DianChuan_Frame *pFrame = &DianChuanRxFrame;

    uint16_t max_power=0;
    uint8_t ic_money=0;
    uint16_t time1=0;
    uint16_t time2=0;
    uint16_t time3=0;
    uint8_t ic_money_en=0;
    uint8_t charge_finish_stopen=0;

    data[0] = 0x00;//port num
    data_len =1;

    ret = dc_com_data( dc1_2,DIANCHUAN_CMD_ReadICCoinPower,data, data_len);
    if(ret ==0)
    {
        max_power = (pFrame->data[0]<<8)|pFrame->data[1];
        ic_money = pFrame->data[2];
        time1 = (pFrame->data[3]<<8)|pFrame->data[4];
        time2 = (pFrame->data[5]<<8)|pFrame->data[6];
        time3 = (pFrame->data[7]<<8)|pFrame->data[8];
        ic_money_en = pFrame->data[9];
        charge_finish_stopen = pFrame->data[10];
        printf("max_power=%d\r\n",max_power);
        printf("ic_money=%d\r\n",ic_money);
        printf("time1=%d\r\n",time1);
        printf("time2=%d\r\n",time2);
        printf("time3=%d\r\n",time3);
        printf("ic_money_en=%d\r\n",ic_money_en);
        printf("charge_finish_stopen=%d\r\n",charge_finish_stopen);
    }
    return ret;

}
int dc_get_port_charge_status(uint8_t dc1_2)
{
    uint8_t data[2]= {0};
    uint8_t data_len =0;
    //uint8_t dc1_2 = 0;
    int ret =0;
    DianChuan_Frame *pFrame = &DianChuanRxFrame;
    int i =0;
    uint16_t total_current=0;
    uint8_t dc_temp=0;
    uint16_t loop_status=0;

    uint16_t power_w[10]= {0};


    data[0] = 0x00;
    data_len =1;
//DIANCHUAN_CMD_GetAllPortPowerStatus --no response
    ret = dc_com_data( dc1_2,DIANCHUAN_CMD_GetAllPortPowerStatus,data, data_len);
    if(ret ==0)
    {
        total_current = (pFrame->data[0]<<8)|pFrame->data[1];
        dc_temp = pFrame->data[2];
        loop_status = (pFrame->data[3]<<8)|pFrame->data[4];
        printf("total_current=%d\r\n",total_current);
        printf("dc_temp=%d\r\n",dc_temp);
        printf("loop_status=0x%04x\r\n",loop_status);
        for(i=0; i<10; i++)
        {
            power_w[i] = (pFrame->data[5+i]<<8)|pFrame->data[6+i];
            printf("power_w[i]=%d\r\n",power_w[i]);
        }

    }
    return ret;

}

void dc_construct_ack_frame(DianChuan_Frame *frame,u8 cmd,u8 *session_id,u8 *data,u8 datalen)
{
    u8 i =0;


    if(frame == NULL)
    {
        return;
    }
    else
        memset(frame,0,sizeof(DianChuan_Frame));

    frame->sop = DIANCHUAN_RX_SOP;
    frame->len = 0;
    frame->cmd = cmd;
    frame->len++;
    frame->sum = cmd;

    if(session_id == NULL)
    {
        memset(frame->session_id,0,6);
    }
    else
    {
        memcpy(frame->session_id,session_id,6);
    }
    frame->len += 6;
    frame->sum ^= frame->session_id[0]^frame->session_id[1]^frame->session_id[2]^ \
                  frame->session_id[3]^frame->session_id[4]^frame->session_id[5];
    //printf("frame->sum=0x%02x\r\n",frame->sum);
    if(data == NULL)
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
volatile uint8_t dc_port_complete =0;
volatile uint8_t dc_port_complete_reason =0;

uint8_t get_dc_port_time_complete(void)
{
    return dc_port_complete;
}
void reset_dc_port_time_complete(void)
{

    dc_port_complete = 0;
    dc_port_complete_reason =0xff;
}

uint8_t dc_active_send_mcu_response(uint8_t dc1_2,uint8_t*pdata,uint8_t data_len)
{
#if 1
    uint8_t* pRxBuffer = pdata;
    if(pRxBuffer[0]== DIANCHUAN_TX_SOP)
    {
        if(pRxBuffer[2]== DIANCHUAN_CMD_PowerComplete)
        {
            uint8_t port_ch = pRxBuffer[9];
            uint16_t port_left_time = (pRxBuffer[10]<<8)|pRxBuffer[11];

            uint8_t reason =  pRxBuffer[12];
            printf("PowerComplete port_ch=%d,port_left_time=0x%04x,reason=0x%x\r\n",
                   port_ch,port_left_time,reason);
            //66 09 05 00 00 00 00 00 00 01 0d
            switch(reason)
            {
            case CHARGE_COMPLETE_REASON_IS_TIME_POWER_COMPLETE:
                dc_port_complete_reason = CHARGE_COMPLETE_REASON_IS_TIME_POWER_COMPLETE;
                dc_port_complete = port_ch ;
                break;
            case CHARGE_COMPLETE_REASON_IS_USER_MANU_STOP:
                dc_port_complete_reason = CHARGE_COMPLETE_REASON_IS_USER_MANU_STOP;
                dc_port_complete = port_ch ;
                break;
            case CHARGE_COMPLETE_REASON_IS_CHARGE_FULL_AUTOSTOP:
                dc_port_complete_reason = CHARGE_COMPLETE_REASON_IS_USER_MANU_STOP;
                dc_port_complete = port_ch ;
#if 1
#else
                if(port_ch>0 &&  port_ch < 20 )
                {
                    if(charge_Info.dc_port_status[port_ch] == PORT_USEING)
                    {


                        //SET_EVENT(MCU_EVENT_AUDIO_NUM_1+port_ch);
                        SET_EVENT(port_ch);
                        SET_EVENT(MCU_EVENT_AUDIO_CHARGE_END);

                        if(evt_id_app)
                        {

                            osEventFlagsSet(evt_id_app, EVENT_FLAGS_APP_DC_COMPLETE);
                        }
                    }
                    else
                    {
                        printf("REASON_IS_TIME_POWER_COMPLETE %d=0x%02x\r\n",port_ch,charge_Info.dc_port_status[port_ch]);

                    }
                }
#endif

                break;
            //case CHARGE_COMPLETE_REASON_IS_USER_MANU_STOP:
            //case CHARGE_COMPLETE_REASON_IS_CHARGE_FULL_AUTOSTOP:
            //	break;
            case CHARGE_COMPLETE_REASON_IS_DEVICE_PORT_BADSTOP:
            case CHARGE_COMPLETE_REASON_IS_POWER_TOO_MAX_CUTSTOP:
            default:
                SET_EVENT(MCU_EVENT_AUDIO_CHARGE_AO);
                SET_EVENT(MCU_EVENT_AUDIO_CHARGE_BUG);
                break;

            }

        }
        else if(pRxBuffer[2] /*DianChuanRxFrame.cmd*/== DIANCHUAN_CMD_ReportError)
        {
            uint8_t port_ch = pRxBuffer[9];
            uint8_t error_code = pRxBuffer[10];
            printf("ReportError port_ch=%d,error_code=0x%x\r\n",port_ch,error_code);
            if(evt_id_app)
            {

                osEventFlagsSet(evt_id_app, EVENT_FLAGS_APP_DC_REPORT_ERROR);
            }


        }
        else
            printf("cmd pRxBuffer[2]=0x%x\r\n",pRxBuffer[2]);
        //ack
        uint8_t cmd = pRxBuffer[2];
        uint8_t data = 1;
        uint8_t data_len = 1;
        dc_construct_ack_frame(&DianChuanTxFrame,cmd,NULL,&data,data_len);

        printf("dc1 ack...=%d\r\n",get_curtime2());
        //ee 09 05 00 00 00 00 00 00 01 0d
        dc_tx_send_data(dc1_2,&DianChuanTxFrame);
        return 1;
///		reset_uart3_rx_buffer();
//#if (UART3_RX_DMA ==1)
//		DMA_Enable(UART3_RX_DMA_CHANNEL,UART3_RX_BUFFER_LEN);//������һ��DMA����
//#endif

    }
    else
    {
        return 0;
    }

#endif

}
