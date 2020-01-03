#ifndef __DIANCHUAN_H
#define __DIANCHUAN_H
#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_it.h"

//#include "sys.h" 



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


#define USART_REC_LEN 20
typedef struct DianChuan_UART_TX_node
{
	uint8_t USART_TX_BUF[USART_REC_LEN];
	uint16_t USART_TX_Data_Len;
}DianChuan_UART_TX;  //�紨��������֡

typedef struct DianChuan_UART_TX_link
{
    DianChuan_UART_TX Cmd[TXCMDBUFCNT];
    uint8_t Head;
    uint8_t Tail;
    uint8_t Count;
}DianChuan_TX_link;   //�紨��������֡����

typedef struct DianChuan_TX_Frame_Type
{
    uint8_t sop;
    uint8_t len;
    uint8_t cmd;
    uint8_t session_id[6];
    //uint8_t data[247];
	uint8_t data[100];
    uint8_t sum;
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

#if 0
ֹͣ��ԭ��
0x00������ĳ��ʱ�䡢����������
0x01���û��ֶ�ֹͣ���β�ͷ�����ǰ���ֹͣ��ť��
0x02��������ˣ��Զ�ֹͣ
0x03���豸���Ƕ˿ڳ������⣬����ֹͣ
0x04�����������ʳ������վ�ĵ�·���������ʣ��ж����
0x05��ˢ���˷ѽ���	
0x06����ʼ���δ�ӳ����

#endif

#define CHARGE_COMPLETE_REASON_IS_TIME_POWER_COMPLETE  0X00
#define CHARGE_COMPLETE_REASON_IS_USER_MANU_STOP  0X01
#define CHARGE_COMPLETE_REASON_IS_CHARGE_FULL_AUTOSTOP  0X02
#define CHARGE_COMPLETE_REASON_IS_DEVICE_PORT_BADSTOP  0X03
#define CHARGE_COMPLETE_REASON_IS_POWER_TOO_MAX_CUTSTOP  0X04

extern DianChuan_Frame DianChuanTxFrame;
extern DianChuan_Frame DianChuanRxFrame;
void DianChuan_ConstructTxCmdFrame(DianChuan_Frame *frame,u8 cmd,u8 *session_id,u8 *data,u8 datalen);

#if 0
extern DianChuan_TX_link DianChuanTxLink;
extern void Display_RxFrameData(u8 *data,u8 len);
extern u8 DianChuan_RxDataProsess(u8 * buf,u8 len);
extern void Display_UpdatePortStatus(DianChuan_Board *port,u8 portnum);
extern void DianChuan_USART_SendData(u8 *buf, u16 len);



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
void uart3_rec_cmd_response(uint8_t *rcv_buf, uint16_t* rcv_len);
void uart4_rec_cmd_response(uint8_t *rcv_buf, uint16_t* rcv_len);
void dc_tx_send_data(uint8_t dc1_2,DianChuan_Frame* pFrame);
void dc_rx_rcv_data(uint8_t dc1_2,DianChuan_Frame* pFrame);
extern volatile uint8_t dc1_exist; 
extern volatile uint8_t dc2_exist; 

void gpio_dc_detect_init(void);
void gpio_dc_detect_value(void);
void gpio_dc_detect_exti_config(void);
void dc_tx_rx_data_test(void);
int dc_tx_rx_data(uint8_t dc1_2,uint8_t cmd);

int dc_com_data(uint8_t dc1_2,uint8_t cmd,uint8_t* pdata,uint8_t data_len);

void dc_port_status(void);
int dc_start_power(uint8_t port_num,uint16_t time_minute);
int dc_stop_power(uint8_t port_num);
//int dc_port_x_status(uint8_t ch);
int dc_port_x_status(uint8_t port_num,uint16_t* pport_status);

int dc_set_max_power(uint8_t dc1_2,uint16_t max_power);
int dc_get_total_consumption(uint8_t dc1_2);
int dc_get_maxpower_charge_finish_stopen(uint8_t dc1_2);
int dc_get_port_charge_status(uint8_t dc1_2);//no response
uint8_t dc_active_send_mcu_response(uint8_t dc1_2,uint8_t*pdata,uint8_t data_len);
uint8_t get_dc_port_time_complete(void);
void reset_dc_port_time_complete(void);
extern volatile uint8_t dc_port_complete_reason;
#endif


