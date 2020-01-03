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
读取设备每个端口的状态      网关-->电川
PORT_NUM	PORT1_STATUS	PORT2_STATUS	…
0x0A	0x02	0x01	…
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
付款成功开始充电     网关-->电川
PORT	RESULT
0x03	0x01

PORT	是	用户选择的充电端口号，0x02表示2号端口。
RESULT	是	0x01，成功 0x02，充电失败

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
//投币上报             电川-->网关
模块不处理
*/
void DianChuan_RxCmd_CoinReport (DianChuan_Frame *frame){}

/*
提交充电结束状态     电川-->网关
PORT	TIME/POWER	REASON	卡号	 退费金额	卡类型
0x03	0x01E0	    0x01   0x01020304	0x01	0x0055

PORT	是	充电端口号，0x02表示2号端口。
TIME/POWER	是	用户剩余的充电时间或者剩余电量：1.如果是充电时间则以分钟为单位，如 0x0064表示100分钟。2.如果是充电电量则以0.01度（千瓦时）为单位，如0x0064表示100个单位=1度电量，以分钟0.01度
当该值为0xFFFF的时候，系统会全额退款，表示该次交易不成功（一般为设备损坏，或是用户无法正常充电）。
REASON	是	停止的原因：
0x00：购买的充电时间、电量用完了
0x01：用户手动停止（拔插头，或是按了停止按钮）
0x02：充电满了，自动停止
0x03：设备或是端口出现问题，被迫停止
0x04：因充电器功率超过充电站的单路最大输出功率，切断输出
0x05：刷卡退费结束	
0x06：开始充电未接充电器
卡号	是	0x010 0x02 0x03 0x04 （只有在刷卡充电且需要退费的情况下发送卡号），如果此端口无刷卡不发送此4个字节卡号与退费金额
退费金额	是	0x01表示  0.1元  （只有在刷卡充电且需要退费的情况下发送金额），如果此端口无刷卡不发送此1个字节退费金额
卡类型	是	两个字节的卡类型


需要回复数据给电川
RESULT
0x01
RESULT	是	0x01表示模块接收成功

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
查询端口当前的充电状态      网关-->电川
PORT	TIME\POWER	INSTANT POWER
0x03	0x0001	0x0001

PORT	是	0x03表示端口3
TIME\POWER	是	表示这一路充电端口的剩余充电时间或者充电电量：1.如果是充电时间则以分钟为单位，如 0x0064表示100分钟。2.如果是充电电量则以0.01度（千瓦时）为单位，如0x0064表示100个单位=1度电量，以分钟0.01度
，0x00表示不在充电（包括空闲、故障）
INSTANT POWER	是	这一路当前充电的瞬时功率，单位为0.1W。如0x0001表示0.1W。

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
#if 1 //调试
        DianChuanBoard.st[port-1].sst = time?0x02:0x01;
        DianChuanBoard.st[port-1].tl = time;
        DianChuanBoard.st[port-1].pow = power;
#endif

}

/*查询消费总额数据      网关-->电川
服务器会去查询机器存储投币、刷卡的消费总额数据。
CARD_MONEY	COIN_MONEY
0x0001	0x01E0
*/
void DianChuan_RxCmd_GetTotalConsumption (DianChuan_Frame *frame){}

/*IC卡、投币、最大功率设置     网关-->电川
NULL
0x01
NULL	是	固定为0x01,表示成功，0x00表示失败
*/
void DianChuan_RxCmd_SetMaxPower (DianChuan_Frame *frame){}

/*设置IC卡、投币器是否可用     网关-->电川
NULL
0x01
*/
void DianChuan_RxCmd_SetICEnable (DianChuan_Frame *frame){}

/*远程停止某个端口的充电     网关-->电川
PORT	TIME/POWER
0x03	0x01E0
PORT	是	充电端口号，0x02表示2号端口。
TIME\POWER	是	用户剩余的充电时间或者充电电量，单位参考第4

*/
void DianChuan_RxCmd_ClosePower (DianChuan_Frame *frame)      //
{
    u8 port =0;
    u8 time = 0;
    
    port = frame->data[0];
    time = ((u16)frame->data[1]<<8)+frame->data[2];
    
    printf("port .... %02X,time = %04X \r\n",port,time);
}

/*读取设备IC卡、投币、最大功率设置、刷卡和充满自停使能  ?
MAX_POWER	IC_MONEY	TIME1/POWER1	TIME2/POWER2	TIME3/POWER3	刷卡退费使能	充满自停使能
0x01F4	0x01	0x0001	0x0001	0x0001	0x00	0x00

MAX_POWER	是	最大输出功率，以W（瓦）为单位。
IC_MONEY	是	用户每次刷IC卡需要消耗的金额，单位为角。如0x01表示0.1元
TIME1/POWER1	是	第一个币的充电时间(0-999分钟）或者充电电量（0-9.99度）
TIME2/POWER2	是	第二个币的充电时间(0-999分钟）或者充电电量（0-9.99度）
TIME3/POWER3	是	第三个币的充电时间(0-999分钟）或者充电电量（0-9.99度）
刷卡退费使能	是	0x00关闭，0x01开启
充满自停使能	是	0x00关闭，0x01开启

*/
void DianChuan_RxCmd_ReadICCoinPower (DianChuan_Frame *frame){}

/*上传设备故障             电川-->网关
PORT	ERROR_CODE
0x03	0x01

PORT	是	充电端口号，0x02表示2号端口。
        若错误的地方不包括端口号，则填写0xFF。0xff代表整机
ERROR_CODE	是	错误码
错误码列表：

0x01	端口输出故障
0x02	机器整体充电功率过大
0x03	电源故障
0x04	预留

回复数据 固定
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
        //整机错误
    }    
    
    printf("port .... %02X,error_code = %02X \r\n",port,error_code);
    DianChuan_ConstructTxCmdFrame(&ack,frame->cmd,frame->session_id,data,sizeof(data));
    DianChuan_SendDataToTxLink(&ack);
}

/*刷卡、退费成功按按键开始充电向模块发送卡号、扣费金额、卡类型、端口 电川-->网关
设备向模块发送IC卡号和扣费金额、卡类型、端口。
*/
void DianChuan_RxCmd_ReportData (DianChuan_Frame *frame){}

/*在线卡类型是0x0055的刷卡向后台发送卡号、扣费金额等                    
电川-->网关
*/
void DianChuan_RxCmd_ReportOnlineData22 (DianChuan_Frame *frame){}
/*给模块再回复收到数据，确任开始扣费指令    电川-->网关*/
void DianChuan_RxCmd_ReportOnlineData23 (DianChuan_Frame *frame){}

/*向后台发送余额同步        电川-->网关
设备向模块发送IC卡号、余额、同步指令
*/
void DianChuan_RxCmd_ReportSyncData12 (DianChuan_Frame *frame){}
/*同步成功设备在返回指令    电川-->网关*/
void DianChuan_RxCmd_ReportSyncData16 (DianChuan_Frame *frame){}

/*设置充电站充满自停、刷卡是否退费   网关-->电川
NULL
0x01
NULL	是	固定为0x00
*/
void DianChuan_RxCmd_SetAutoFinish (DianChuan_Frame *frame)       //
{
    //DianChuan_Frame ack = {0};
    //u8 data[1]={0x00};
    u8 data =0 ;
    data = frame->data[0];
    
    printf("data .... %d,\r\n",data);
}


/*设置充电站5档计费功率、比例   网关-->电川
null	是	0x01设置成功， 0x00 设置失败
*/
void DianChuan_RxCmd_Set5Power (DianChuan_Frame *frame) {}

/*读取设备5档计费功率、比例  电川-->网关*/
void DianChuan_RxCmd_Read5Power (DianChuan_Frame *frame){}


/*查询整机所有充电端口的充电状态   网关-->电川
回复数据（上面表示数据含义，下面表示示例数据）：
总电流[2B]+箱体温度[1B]+输出回路继电器状态[2B]+输出回路功率[20B]+剩余充电时间[20B]

总电流	箱体温度	输出回路继电器状态	第1路充电功率	。。。。。。	第10路充电功率	第1路充电剩余时间	。。。。。。	第10路充电剩余时间
0x0001	0xff	0x0001	0x0001		0x0001	0x0001		0x0001

总电流	是	0x0001表示 0.1度  表示整机充电的总电流
箱体温度	是	表示现在机箱的温度，如果没有接传感器，回传0xff
输出回路继电器状态	是	Bit位 0000000001（表示第一路在充电）0表示无充电，1表示充电。
第1路充电功率	是	这一路当前充电的瞬时功率，单位为0.1W。如0x0001表示0.1W。
。。。。	。。。。	。。。。
第10路充电功率	是	这一路当前充电的瞬时功率，单位为0.1W。如0x0001表示0.1W。
第1路剩余充电时间	是	表示这一路充电端口的剩余充电时间，无充电为0
0x0001表示1分钟
。。。。	。。。。	。。。。。
第10路剩余充电时间	是	表示这一路充电端口的剩余充电时间，无充电为0
0x0001表示1分钟
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
#if 1 //调试
        DianChuanBoard.st[i].sst = relaystatus&(1<<i)?0x02:0x01;
        DianChuanBoard.st[i].pow = portpower[i];
        DianChuanBoard.st[i].tl = lasttime[i];
#endif
    }


}

/*读取设备的版本号   网关-->电川
版本号
0x0001

版本号	是	程序软件版本号

*/
void DianChuan_RxCmd_GetVersonNumber (DianChuan_Frame *frame)       //
{
    u16 version = 0;
    version = ((u16)frame->data[0]<<8)+frame->data[1];
    printf("version .... %d\r\n",version);
}

/*
软件版本更新   网关-->电川

版本号	状态
0x0001	0x01

版本号	是	程序软件版本号
状态	是	0x01,进入更新状态

在线更新协议：Xmodem-1k  波特率9600
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
设置免费充电模式、音量调节
NULL
0x01
NULL	是	固定为0x01

*/
void DianChuan_RxCmd_SetFreemodeVolume (DianChuan_Frame *frame)       
{
    u16 data= 0;
    data = frame->data[0];
    
    printf("data .... %d,\r\n",data);
}

/*
设置最低浮充功率、浮充时间   网关-->电川

NULL
0x01
NULL	是	固定为0x01

*/
void DianChuan_RxCmd_SetMinPowerTime (DianChuan_Frame *frame)       
{
    u16 data= 0;
    data = frame->data[0];
    
    printf("data .... %d,\r\n",data);
}


/*
刷卡充电时间   网关-->电川
*/
void DianChuan_RxCmd_SetOnceICPowerTime (DianChuan_Frame *frame){}

/* 
读取设备刷卡充电时间、充电模式、语音音量、浮充功率、浮充时间   网关-->电川
TIME1/POWER1	TIME2/POWER2	TIME3/POWER3	免费充电模式	语音音量	Floating powr	Floating Time
0x0001	0x0001	0x0001	0x00	0x01	0x0001	0x0001
TIME1/PWOER1	是	刷第一次卡的充电时间(0-999分钟）或者充电电量（0-9.99度）
TIME2/POWER2	是	刷第二次卡的充电时间(0-999分钟）或者充电电量（0-9.99度）
TIME3/POWER3	是	刷第三次卡的充电时间(0-999分钟）或者充电电量（0-9.99度）
免费充电模式	是	0x00关闭，0x01打开
语音音量	是	范围是1-8；8最高
Floating powr	是	最低的浮充功率，低于这个值关闭输出，范围0-99.9   ，0x0001=0.1W
Floating Time	是	到达最低浮充功率进行检测的时间：120秒-999秒 0x0001=1秒

*/
void DianChuan_RxCmd_ReadPowrModeVolumeTime (DianChuan_Frame *frame){}
/**/

u8 DianChuan_RXFrameProsess(u8 *FrameBuf , u8 len)
{
    u8 ret = 0;
    
#if 1  //调试使用
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
        case  DIANCHUAN_CMD_CoinReport:        //投币上报             电川-->网关
        {
             DianChuan_RxCmd_CoinReport (&DianChuanRxFrame);         //投币上报             电川-->网关
           
        }
        break;
        case  DIANCHUAN_CMD_PowerComplete:         //提交充电结束状态     电川-->网关
        {
             DianChuan_RxCmd_PowerComplete (&DianChuanRxFrame);         //提交充电结束状态     电川-->网关
            //Display_RxFrameData(FrameBuf,len);
        }
        break;

        case  DIANCHUAN_CMD_GetPortStatus:      //查询端口当前的充电状态      网关-->电川
        {
            DianChuan_RxCmd_GetPortStatus (&DianChuanRxFrame);      //查询端口当前的充电状态      网关-->电川
            //Display_RxFrameData(FrameBuf,len);
        }
        break;
        case  DIANCHUAN_CMD_GetTotalConsumption:      //查询消费总额数据      网关-->电川
        {
             DianChuan_RxCmd_GetTotalConsumption (&DianChuanRxFrame);      //查询消费总额数据      网关-->电川
            //Display_RxFrameData(FrameBuf,len);
        }
        break;
        case  DIANCHUAN_CMD_SetMaxPower:      //IC卡、投币、最大功率设置     网关-->电川
        {
             DianChuan_RxCmd_SetMaxPower (&DianChuanRxFrame);      //IC卡、投币、最大功率设置     网关-->电川
            //Display_RxFrameData(FrameBuf,len);
        }
        break;
        case  DIANCHUAN_CMD_SetICEnable:      //设置IC卡、投币器是否可用     网关-->电川
        {
             DianChuan_RxCmd_SetICEnable (&DianChuanRxFrame);      //设置IC卡、投币器是否可用     网关-->电川
            //Display_RxFrameData(FrameBuf,len);
        }
        break;
        case  DIANCHUAN_CMD_ClosePower:      //远程停止某个端口的充电     网关-->电川
        {
             DianChuan_RxCmd_ClosePower (&DianChuanRxFrame);      //远程停止某个端口的充电     网关-->电川
            //Display_RxFrameData(FrameBuf,len);
        }
        break;
        case  DIANCHUAN_CMD_ReadICCoinPower:      //读取设备IC卡、投币、最大功率设置、刷卡和充满自停使能     ?
        {
             DianChuan_RxCmd_ReadICCoinPower (&DianChuanRxFrame);      //读取设备IC卡、投币、最大功率设置、刷卡和充满自停使能     ?
            //Display_RxFrameData(FrameBuf,len);
        }
        break;
        case  DIANCHUAN_CMD_ReportError:         //上传设备故障             电川-->网关
        {
             DianChuan_RxCmd_ReportError (&DianChuanRxFrame);         //上传设备故障             电川-->网关
            //Display_RxFrameData(FrameBuf,len);
        }
        break;
        case  DIANCHUAN_CMD_ReportData:         //刷卡、退费成功按按键开始充电向模块发送卡号、扣费金额、卡类型、端口            电川-->网关
        {
             DianChuan_RxCmd_ReportData (&DianChuanRxFrame);         //刷卡、退费成功按按键开始充电向模块发送卡号、扣费金额、卡类型、端口            电川-->网关
            //Display_RxFrameData(FrameBuf,len);
        }
        break;

        case  DIANCHUAN_CMD_ReportOnlineData22:   // 发送卡号、余额给模块                     电川-->网关
        {
             DianChuan_RxCmd_ReportOnlineData22 (&DianChuanRxFrame);   // 发送卡号、余额给模块                     电川-->网关
            //Display_RxFrameData(FrameBuf,len);
        }
        break;

        case  DIANCHUAN_CMD_ReportOnlineData23:   //给模块再回复收到数据，确任开始扣费指令    电川-->网关
        {
             DianChuan_RxCmd_ReportOnlineData23 (&DianChuanRxFrame);   //给模块再回复收到数据，确任开始扣费指令    电川-->网关
            //Display_RxFrameData(FrameBuf,len);
        }
        break;

        case  DIANCHUAN_CMD_ReportSyncData12:       //向后台发送余额同步        电川-->网关
        {
             DianChuan_RxCmd_ReportSyncData12 (&DianChuanRxFrame);       //向后台发送余额同步        电川-->网关
           // Display_RxFrameData(FrameBuf,len);
        }
        break;

        case  DIANCHUAN_CMD_ReportSyncData16:       //同步成功设备在返回指令    电川-->网关
        {
             DianChuan_RxCmd_ReportSyncData16 (&DianChuanRxFrame);       //同步成功设备在返回指令    电川-->网关
            //Display_RxFrameData(FrameBuf,len);
        }
        break;

        case  DIANCHUAN_CMD_SetAutoFinish:       //设置充电站充满自停、刷卡是否退费   网关-->电川
        {
             DianChuan_RxCmd_SetAutoFinish (&DianChuanRxFrame);       //设置充电站充满自停、刷卡是否退费   网关-->电川
            //Display_RxFrameData(FrameBuf,len);
        }
        break;

        case  DIANCHUAN_CMD_Set5Power:       //设置充电站5档计费功率、比例   网关-->电川
        {
             DianChuan_RxCmd_Set5Power (&DianChuanRxFrame);       //设置充电站5档计费功率、比例   网关-->电川
            //Display_RxFrameData(FrameBuf,len);
        }
        break;

        case  DIANCHUAN_CMD_Read5Power:       //读取设备5档计费功率、比例  电川-->网关
        {
             DianChuan_RxCmd_Read5Power (&DianChuanRxFrame);       //读取设备5档计费功率、比例  电川-->网关
            //Display_RxFrameData(FrameBuf,len);
        }
        break;

        case  DIANCHUAN_CMD_GetAllPortPowerStatus:       //查询整机所有充电端口的充电状态   网关-->电川
        {
             DianChuan_RxCmd_GetAllPortPowerStatus (&DianChuanRxFrame);       //查询整机所有充电端口的充电状态   网关-->电川
            //Display_RxFrameData(FrameBuf,len);
        }
        break;

        case  DIANCHUAN_CMD_GetVersonNumber:       //读取设备的版本号   网关-->电川
        {
             DianChuan_RxCmd_GetVersonNumber (&DianChuanRxFrame);       //读取设备的版本号   网关-->电川
            //Display_RxFrameData(FrameBuf,len);
        }
        break;

        case  DIANCHUAN_CMD_SetUpdateVersonNumber:       //软件版本更新   网关-->电川
        {
             DianChuan_RxCmd_SetUpdateVersonNumber (&DianChuanRxFrame);       //软件版本更新   网关-->电川
            //Display_RxFrameData(FrameBuf,len);
        }
        break;

        case  DIANCHUAN_CMD_SetFreemodeVolume:       //设置免费充电模式、音量调节   网关-->电川
        {
             DianChuan_RxCmd_SetFreemodeVolume (&DianChuanRxFrame);       //设置免费充电模式、音量调节   网关-->电川
            //Display_RxFrameData(FrameBuf,len);
        }
        break;

        case  DIANCHUAN_CMD_SetMinPowerTime:       //设置最低浮充功率、浮充时间   网关-->电川
        {
             DianChuan_RxCmd_SetMinPowerTime (&DianChuanRxFrame);       //设置最低浮充功率、浮充时间   网关-->电川
           // Display_RxFrameData(FrameBuf,len);
        }
        break;

        case  DIANCHUAN_CMD_SetOnceICPowerTime:       //刷卡充电时间   网关-->电川
        {
             DianChuan_RxCmd_SetOnceICPowerTime (&DianChuanRxFrame);       //刷卡充电时间   网关-->电川
           // Display_RxFrameData(FrameBuf,len);
        }
        break;

        case  DIANCHUAN_CMD_ReadPowrModeVolumeTime:       //读取设备刷卡充电时间、充电模式、语音音量、浮充功率、浮充时间   网关-->电川
        {
            
             DianChuan_RxCmd_ReadPowrModeVolumeTime (&DianChuanRxFrame);       //读取设备刷卡充电时间、充电模式、语音音量、浮充功率、浮充时间   网关-->电川
            //Display_RxFrameData(FrameBuf,len);
        }
        break;
    }
    return 0;
}

/*
SOP	    LEN	    CMD	    SESSION_ID	    DATA	SUM
1Byte   1Byte   1Byte   6Bytes          n       1Byte  

返回 0 代表数据被处理完，
返回 非零代表有数据没处理
*/
u8 DianChuan_RxDataProsess(u8 * buf,u8 len)
{
    if(buf[0] == DIANCHUAN_RX_SOP || buf[0] == DIANCHUAN_TX_SOP)
    {
        if(buf[1] > 7 && buf[1] + 2 == len)//判断数据帧是否完整
        {
            DianChuan_RXFrameProsess(buf,len);
            len = 0;
        }
    }
    else
    {
        len = 0; //帧头不对清空帧buf
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
#if 1  //调试使用
        Display_TxFrameData(buf,len);
#endif 
    for(i=0; i < len; i++)
    {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
        USART_SendData(USART1,*(buf+i));
    }
}

