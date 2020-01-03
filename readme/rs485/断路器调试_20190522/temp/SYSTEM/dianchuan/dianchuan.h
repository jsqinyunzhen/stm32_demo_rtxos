#ifndef __DIANCHUAN_H
#define __DIANCHUAN_H
#include "stdio.h"	
#include "sys.h" 



#define DIANCHUAN_RX_SOP (0x66)
#define DIANCHUAN_TX_SOP (0xEE)
#define DIANCHUAN_CMD_GetAllPortStatus (0x01)    //��ȡ�豸ÿ���˿ڵ�״̬      ����-->�紨
#define DIANCHUAN_CMD_StartPower (0x02)         //����ɹ���ʼ���     ����-->�紨
#define DIANCHUAN_CMD_CoinReport (0x03)         //Ͷ���ϱ�             �紨-->����
#define DIANCHUAN_CMD_PowerComplete (0x05)         //�ύ������״̬     �紨-->����
#define DIANCHUAN_CMD_GetPortStatus (0x06)      //��ѯ�˿ڵ�ǰ�ĳ��״̬      ����-->�紨
#define DIANCHUAN_CMD_GetTotalConsumption (0x07)      //��ѯ�����ܶ�����      ����-->�紨
#define DIANCHUAN_CMD_SetMaxPower (0x08)      //IC����Ͷ�ҡ����������     ����-->�紨
#define DIANCHUAN_CMD_SetICEnable (0x09)      //����IC����Ͷ�����Ƿ����     ����-->�紨
#define DIANCHUAN_CMD_ClosePower (0x0b)      //Զ��ֹͣĳ���˿ڵĳ��     ����-->�紨
#define DIANCHUAN_CMD_ReadICCoinPower (0x0c)      //��ȡ�豸IC����Ͷ�ҡ���������á�ˢ���ͳ�����ͣʹ��     ?
#define DIANCHUAN_CMD_ReportError (0x0d)         //�ϴ��豸����             �紨-->����
#define DIANCHUAN_CMD_ReportData (0x11)         //ˢ�����˷ѳɹ���������ʼ�����ģ�鷢�Ϳ��š��۷ѽ������͡��˿�            �紨-->����
#define DIANCHUAN_CMD_ReportOnlineData22 (0x22)   // ���Ϳ��š�����ģ��                     �紨-->����
#define DIANCHUAN_CMD_ReportOnlineData23 (0x23)   //��ģ���ٻظ��յ����ݣ�ȷ�ο�ʼ�۷�ָ��    �紨-->����
#define DIANCHUAN_CMD_ReportSyncData12 (0x12)       //���̨�������ͬ��        �紨-->����
#define DIANCHUAN_CMD_ReportSyncData16 (0x16)       //ͬ���ɹ��豸�ڷ���ָ��    �紨-->����
#define DIANCHUAN_CMD_SetAutoFinish (0x13)       //���ó��վ������ͣ��ˢ���Ƿ��˷�   ����-->�紨
#define DIANCHUAN_CMD_Set5Power (0x14)       //���ó��վ5���Ʒѹ��ʡ�����   ����-->�紨
#define DIANCHUAN_CMD_Read5Power (0x15)       //��ȡ�豸5���Ʒѹ��ʡ�����  �紨-->����
#define DIANCHUAN_CMD_GetAllPortPowerStatus (0x24)       //��ѯ�������г��˿ڵĳ��״̬   ����-->�紨
#define DIANCHUAN_CMD_GetVersonNumber (0x25)       //��ȡ�豸�İ汾��   ����-->�紨
#define DIANCHUAN_CMD_SetUpdateVersonNumber (0x26)       //����汾����   ����-->�紨
#define DIANCHUAN_CMD_SetFreemodeVolume (0x27)       //������ѳ��ģʽ����������  ����-->�紨
#define DIANCHUAN_CMD_SetMinPowerTime (0x28)       //������͸��书�ʡ�����ʱ��   ����-->�紨
#define DIANCHUAN_CMD_SetOnceICPowerTime (0x29)       //ˢ�����ʱ��   ����-->�紨
#define DIANCHUAN_CMD_ReadPowrModeVolumeTime (0x2a)       //��ȡ�豸ˢ�����ʱ�䡢���ģʽ���������������书�ʡ�����ʱ��   ����-->�紨


#define TXCMDBUFCNT     20   //���͵紨����֡�Ķ��г��ȣ���ʱ�趨���20
#define DIANCHUANPORTNUM (10) //�紨���ӵĳ��˿��� 10��



typedef struct DianChuan_UART_TX_node
{
	u8 USART_TX_BUF[USART_REC_LEN];
	u16 USART_TX_Data_Len;
}DianChuan_UART_TX;  //�紨��������֡

typedef struct DianChuan_UART_TX_link
{
    DianChuan_UART_TX Cmd[TXCMDBUFCNT];
    u8 Head;
    u8 Tail;
    u8 Count;
}DianChuan_TX_link;   //�紨��������֡����

typedef struct DianChuan_TX_Frame_Type
{
    u8 sop;
    u8 len;
    u8 cmd;
    u8 session_id[6];
    u8 data[247];
    u8 sum;
}DianChuan_Frame;  //�紨����֡��ʽ


typedef struct DianChuan_Port_Status_Type
{
    u8 cn; //������ţ�����
    u8 sst; //����״̬   0x01Ϊ�� 0x02Ϊ��
    u16 apow;//����ʣ����֣���λw
    u8 ipow;//��С���ʣ����֣���λw
    u8 tck; //��������䣩���ʱ�䣬���֣���λ����
    u8 opt; //����ʣ��ʱ�䣬���֣���λ����
    u16 en;//���γ�����ʱ�����ĵĵ��ܣ����֣���λkwh
    u8 type;/*״̬��  -3-�趨����״̬ʧ�ܣ�0-�����رգ�1-С�����޹�����������رգ�2-�������޹��ʣ�3-�趨����״̬�ɹ�,4-ͨ���һ��ʱ��û�в����ͷ�Զ��رգ�5-��ͷ���γ������ã���ֳ�6��7����6-���������γ���7-��ͷ�������γ�*/
    u16 pow; //����
    u16 cur;//����
    u16 tl;//ʣ��ʱ��
}DianChuan_Port_Status; //�˿�״̬

typedef struct DianChuan_Board_Type
{
    char did[20]; //���վid��ţ��ַ���uuid
    u32 t;  //ʵʱʱ�䣬���֣�������
    DianChuan_Port_Status st[DIANCHUANPORTNUM]; //״̬
    u8 warning; //    �������ͣ����֣�1-�¶ȳ��ޣ�2-�����ű��򿪣�3-ͣ�磬4-�̸�
    u16 temp;  //���վ�¶ȣ����֣���λ���϶�
    u32 v;//�汾��
    
}DianChuan_Board; //�紨����


extern DianChuan_Frame DianChuanTxFrame;
extern DianChuan_TX_link DianChuanTxLink;
extern void Display_RxFrameData(u8 *data,u8 len);
extern u8 DianChuan_RxDataProsess(u8 * buf,u8 len);
extern void Display_UpdatePortStatus(DianChuan_Board *port,u8 portnum);
extern void DianChuan_USART_SendData(u8 *buf, u16 len);


void DianChuan_ConstructTxCmdFrame(DianChuan_Frame *frame,u8 cmd,u8 *session_id,u8 *data,u8 datalen);
u8 DianChuan_SendDataToTxLink(DianChuan_Frame *frame);
void DianChuan_RxCmd_GetAllPortStatus(DianChuan_Frame *frame);//��ȡ�豸ÿ���˿ڵ�״̬      ����-->�紨
void DianChuan_RxCmd_StartPower(DianChuan_Frame *frame);//����ɹ���ʼ���     ����-->�紨
void DianChuan_RxCmd_CoinReport (DianChuan_Frame *frame);         //Ͷ���ϱ�             �紨-->����
void DianChuan_RxCmd_PowerComplete (DianChuan_Frame *frame);         //�ύ������״̬     �紨-->����
void DianChuan_RxCmd_GetPortStatus (DianChuan_Frame *frame);      //��ѯ�˿ڵ�ǰ�ĳ��״̬      ����-->�紨
void DianChuan_RxCmd_GetTotalConsumption (DianChuan_Frame *frame);      //��ѯ�����ܶ�����      ����-->�紨
void DianChuan_RxCmd_SetMaxPower (DianChuan_Frame *frame);      //IC����Ͷ�ҡ����������     ����-->�紨
void DianChuan_RxCmd_SetICEnable (DianChuan_Frame *frame);      //����IC����Ͷ�����Ƿ����     ����-->�紨
void DianChuan_RxCmd_ClosePower (DianChuan_Frame *frame);      //Զ��ֹͣĳ���˿ڵĳ��     ����-->�紨
void DianChuan_RxCmd_ReadICCoinPower (DianChuan_Frame *frame);      //��ȡ�豸IC����Ͷ�ҡ���������á�ˢ���ͳ�����ͣʹ��     ?
void DianChuan_RxCmd_ReportError (DianChuan_Frame *frame);         //�ϴ��豸����             �紨-->����
void DianChuan_RxCmd_ReportData (DianChuan_Frame *frame);         //ˢ�����˷ѳɹ���������ʼ�����ģ�鷢�Ϳ��š��۷ѽ������͡��˿�            �紨-->����
void DianChuan_RxCmd_ReportOnlineData22 (DianChuan_Frame *frame);   // ���Ϳ��š�����ģ��                     �紨-->����
void DianChuan_RxCmd_ReportOnlineData23 (DianChuan_Frame *frame);   //��ģ���ٻظ��յ����ݣ�ȷ�ο�ʼ�۷�ָ��    �紨-->����
void DianChuan_RxCmd_ReportSyncData12 (DianChuan_Frame *frame);       //���̨�������ͬ��        �紨-->����
void DianChuan_RxCmd_ReportSyncData16 (DianChuan_Frame *frame);       //ͬ���ɹ��豸�ڷ���ָ��    �紨-->����
void DianChuan_RxCmd_SetAutoFinish (DianChuan_Frame *frame);       //���ó��վ������ͣ��ˢ���Ƿ��˷�   ����-->�紨
void DianChuan_RxCmd_Set5Power (DianChuan_Frame *frame);       //���ó��վ5���Ʒѹ��ʡ�����   ����-->�紨
void DianChuan_RxCmd_Read5Power (DianChuan_Frame *frame);       //��ȡ�豸5���Ʒѹ��ʡ�����  �紨-->����
void DianChuan_RxCmd_GetAllPortPowerStatus (DianChuan_Frame *frame);       //��ѯ�������г��˿ڵĳ��״̬   ����-->�紨
void DianChuan_RxCmd_GetVersonNumber (DianChuan_Frame *frame);       //��ȡ�豸�İ汾��   ����-->�紨
void DianChuan_RxCmd_SetUpdateVersonNumber (DianChuan_Frame *frame);       //����汾����   ����-->�紨
void DianChuan_RxCmd_SetFreemodeVolume (DianChuan_Frame *frame);       //������ѳ��ģʽ����������   ����-->�紨
void DianChuan_RxCmd_SetMinPowerTime (DianChuan_Frame *frame);       //������͸��书�ʡ�����ʱ��   ����-->�紨
void DianChuan_RxCmd_SetOnceICPowerTime (DianChuan_Frame *frame);       //ˢ�����ʱ��   ����-->�紨
void DianChuan_RxCmd_ReadPowrModeVolumeTime (DianChuan_Frame *frame);       //��ȡ�豸ˢ�����ʱ�䡢���ģʽ���������������书�ʡ�����ʱ��   ����-->�紨

#endif


