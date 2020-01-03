#ifndef __DIANCHUAN_H
#define __DIANCHUAN_H
#include "stdio.h"	
#include "sys.h" 



#define DIANCHUAN_RX_SOP (0x66)
#define DIANCHUAN_TX_SOP (0xEE)
#define DIANCHUAN_CMD_GetAllPortStatus (0x01)    //读取设备每个端口的状态      网关-->电川
#define DIANCHUAN_CMD_StartPower (0x02)         //付款成功开始充电     网关-->电川
#define DIANCHUAN_CMD_CoinReport (0x03)         //投币上报             电川-->网关
#define DIANCHUAN_CMD_PowerComplete (0x05)         //提交充电结束状态     电川-->网关
#define DIANCHUAN_CMD_GetPortStatus (0x06)      //查询端口当前的充电状态      网关-->电川
#define DIANCHUAN_CMD_GetTotalConsumption (0x07)      //查询消费总额数据      网关-->电川
#define DIANCHUAN_CMD_SetMaxPower (0x08)      //IC卡、投币、最大功率设置     网关-->电川
#define DIANCHUAN_CMD_SetICEnable (0x09)      //设置IC卡、投币器是否可用     网关-->电川
#define DIANCHUAN_CMD_ClosePower (0x0b)      //远程停止某个端口的充电     网关-->电川
#define DIANCHUAN_CMD_ReadICCoinPower (0x0c)      //读取设备IC卡、投币、最大功率设置、刷卡和充满自停使能     ?
#define DIANCHUAN_CMD_ReportError (0x0d)         //上传设备故障             电川-->网关
#define DIANCHUAN_CMD_ReportData (0x11)         //刷卡、退费成功按按键开始充电向模块发送卡号、扣费金额、卡类型、端口            电川-->网关
#define DIANCHUAN_CMD_ReportOnlineData22 (0x22)   // 发送卡号、余额给模块                     电川-->网关
#define DIANCHUAN_CMD_ReportOnlineData23 (0x23)   //给模块再回复收到数据，确任开始扣费指令    电川-->网关
#define DIANCHUAN_CMD_ReportSyncData12 (0x12)       //向后台发送余额同步        电川-->网关
#define DIANCHUAN_CMD_ReportSyncData16 (0x16)       //同步成功设备在返回指令    电川-->网关
#define DIANCHUAN_CMD_SetAutoFinish (0x13)       //设置充电站充满自停、刷卡是否退费   网关-->电川
#define DIANCHUAN_CMD_Set5Power (0x14)       //设置充电站5档计费功率、比例   网关-->电川
#define DIANCHUAN_CMD_Read5Power (0x15)       //读取设备5档计费功率、比例  电川-->网关
#define DIANCHUAN_CMD_GetAllPortPowerStatus (0x24)       //查询整机所有充电端口的充电状态   网关-->电川
#define DIANCHUAN_CMD_GetVersonNumber (0x25)       //读取设备的版本号   网关-->电川
#define DIANCHUAN_CMD_SetUpdateVersonNumber (0x26)       //软件版本更新   网关-->电川
#define DIANCHUAN_CMD_SetFreemodeVolume (0x27)       //设置免费充电模式、音量调节  网关-->电川
#define DIANCHUAN_CMD_SetMinPowerTime (0x28)       //设置最低浮充功率、浮充时间   网关-->电川
#define DIANCHUAN_CMD_SetOnceICPowerTime (0x29)       //刷卡充电时间   网关-->电川
#define DIANCHUAN_CMD_ReadPowrModeVolumeTime (0x2a)       //读取设备刷卡充电时间、充电模式、语音音量、浮充功率、浮充时间   网关-->电川


#define TXCMDBUFCNT     20   //发送电川数据帧的队列长度，暂时设定最大20
#define DIANCHUANPORTNUM (10) //电川板子的充电端口数 10个



typedef struct DianChuan_UART_TX_node
{
	u8 USART_TX_BUF[USART_REC_LEN];
	u16 USART_TX_Data_Len;
}DianChuan_UART_TX;  //电川发送数据帧

typedef struct DianChuan_UART_TX_link
{
    DianChuan_UART_TX Cmd[TXCMDBUFCNT];
    u8 Head;
    u8 Tail;
    u8 Count;
}DianChuan_TX_link;   //电川发送数据帧队列

typedef struct DianChuan_TX_Frame_Type
{
    u8 sop;
    u8 len;
    u8 cmd;
    u8 session_id[6];
    u8 data[247];
    u8 sum;
}DianChuan_Frame;  //电川数据帧格式


typedef struct DianChuan_Port_Status_Type
{
    u8 cn; //插座编号，数字
    u8 sst; //开关状态   0x01为关 0x02为开
    u16 apow;//最大功率，数字，单位w
    u8 ipow;//最小功率，数字，单位w
    u8 tck; //涓流（浮充）充电时间，数字，单位分钟
    u8 opt; //插座剩余时间，数字，单位分钟
    u16 en;//本次充电完成时所消耗的电能，数字，单位kwh
    u8 type;/*状态码  -3-设定开关状态失败，0-正常关闭，1-小于下限功率涓流结束关闭，2-大于上限功率，3-设定开关状态成功,4-通电后一定时间没有插入插头自动关闭，5-插头被拔出（弃用，拆分成6和7），6-涓流情况被拔出，7-插头正常被拔出*/
    u16 pow; //功率
    u16 cur;//电流
    u16 tl;//剩余时间
}DianChuan_Port_Status; //端口状态

typedef struct DianChuan_Board_Type
{
    char did[20]; //充电站id编号，字符串uuid
    u32 t;  //实时时间，数字，世纪秒
    DianChuan_Port_Status st[DIANCHUANPORTNUM]; //状态
    u8 warning; //    报警类型，数字，1-温度超限，2-机箱门被打开，3-停电，4-烟感
    u16 temp;  //充电站温度，数字，单位摄氏度
    u32 v;//版本号
    
}DianChuan_Board; //电川充电板


extern DianChuan_Frame DianChuanTxFrame;
extern DianChuan_TX_link DianChuanTxLink;
extern void Display_RxFrameData(u8 *data,u8 len);
extern u8 DianChuan_RxDataProsess(u8 * buf,u8 len);
extern void Display_UpdatePortStatus(DianChuan_Board *port,u8 portnum);
extern void DianChuan_USART_SendData(u8 *buf, u16 len);


void DianChuan_ConstructTxCmdFrame(DianChuan_Frame *frame,u8 cmd,u8 *session_id,u8 *data,u8 datalen);
u8 DianChuan_SendDataToTxLink(DianChuan_Frame *frame);
void DianChuan_RxCmd_GetAllPortStatus(DianChuan_Frame *frame);//读取设备每个端口的状态      网关-->电川
void DianChuan_RxCmd_StartPower(DianChuan_Frame *frame);//付款成功开始充电     网关-->电川
void DianChuan_RxCmd_CoinReport (DianChuan_Frame *frame);         //投币上报             电川-->网关
void DianChuan_RxCmd_PowerComplete (DianChuan_Frame *frame);         //提交充电结束状态     电川-->网关
void DianChuan_RxCmd_GetPortStatus (DianChuan_Frame *frame);      //查询端口当前的充电状态      网关-->电川
void DianChuan_RxCmd_GetTotalConsumption (DianChuan_Frame *frame);      //查询消费总额数据      网关-->电川
void DianChuan_RxCmd_SetMaxPower (DianChuan_Frame *frame);      //IC卡、投币、最大功率设置     网关-->电川
void DianChuan_RxCmd_SetICEnable (DianChuan_Frame *frame);      //设置IC卡、投币器是否可用     网关-->电川
void DianChuan_RxCmd_ClosePower (DianChuan_Frame *frame);      //远程停止某个端口的充电     网关-->电川
void DianChuan_RxCmd_ReadICCoinPower (DianChuan_Frame *frame);      //读取设备IC卡、投币、最大功率设置、刷卡和充满自停使能     ?
void DianChuan_RxCmd_ReportError (DianChuan_Frame *frame);         //上传设备故障             电川-->网关
void DianChuan_RxCmd_ReportData (DianChuan_Frame *frame);         //刷卡、退费成功按按键开始充电向模块发送卡号、扣费金额、卡类型、端口            电川-->网关
void DianChuan_RxCmd_ReportOnlineData22 (DianChuan_Frame *frame);   // 发送卡号、余额给模块                     电川-->网关
void DianChuan_RxCmd_ReportOnlineData23 (DianChuan_Frame *frame);   //给模块再回复收到数据，确任开始扣费指令    电川-->网关
void DianChuan_RxCmd_ReportSyncData12 (DianChuan_Frame *frame);       //向后台发送余额同步        电川-->网关
void DianChuan_RxCmd_ReportSyncData16 (DianChuan_Frame *frame);       //同步成功设备在返回指令    电川-->网关
void DianChuan_RxCmd_SetAutoFinish (DianChuan_Frame *frame);       //设置充电站充满自停、刷卡是否退费   网关-->电川
void DianChuan_RxCmd_Set5Power (DianChuan_Frame *frame);       //设置充电站5档计费功率、比例   网关-->电川
void DianChuan_RxCmd_Read5Power (DianChuan_Frame *frame);       //读取设备5档计费功率、比例  电川-->网关
void DianChuan_RxCmd_GetAllPortPowerStatus (DianChuan_Frame *frame);       //查询整机所有充电端口的充电状态   网关-->电川
void DianChuan_RxCmd_GetVersonNumber (DianChuan_Frame *frame);       //读取设备的版本号   网关-->电川
void DianChuan_RxCmd_SetUpdateVersonNumber (DianChuan_Frame *frame);       //软件版本更新   网关-->电川
void DianChuan_RxCmd_SetFreemodeVolume (DianChuan_Frame *frame);       //设置免费充电模式、音量调节   网关-->电川
void DianChuan_RxCmd_SetMinPowerTime (DianChuan_Frame *frame);       //设置最低浮充功率、浮充时间   网关-->电川
void DianChuan_RxCmd_SetOnceICPowerTime (DianChuan_Frame *frame);       //刷卡充电时间   网关-->电川
void DianChuan_RxCmd_ReadPowrModeVolumeTime (DianChuan_Frame *frame);       //读取设备刷卡充电时间、充电模式、语音音量、浮充功率、浮充时间   网关-->电川

#endif


