#include "stm32f10x.h"
#include "delay.h"
#include <stdio.h>
#include "usart.h"	
#include "led.h"
#include "lcd.h"
#include "adc.h"
#include "key.h"
#include "dma.h"
#include "rtc.h"
#include "string.h"
#include "dianchuan.h"
#include "breaker.h"

void Main_timer_Init_1second(void);


__IO u16 adc_array[10][CH_NUM]={0}; //violate
heating_time_t heating_time[HEAT_COUNT] ={{0,0},{0,0},{0,0}};
extern DianChuan_Board DianChuanBoard;

/******* 
init() 
*********/
 void init()
 {
    //RCC_DeInit(); //打开系统会卡
	LED_Init();
	delay_init();
	uart_init(4800);
    //RS485_Config(9600);
	LCD_Init();
	Adc_Init();
	KEY_Init();
	Main_timer_Init_1second();

	while(RTC_Init())		//RTC初始化	，一定要初始化成功
	{ 
		LCD_ShowString(60,130,200,16,16,"RTC ERROR!   ");	
		delay_ms(800);
		LCD_ShowString(60,130,200,16,16,"RTC Trying...");	
	}
 }


void LCD_ShowAEChar(u16 x,u16 y,u16 num,u8 size,u8 mode)
{  							  
	u8 *AECode = 0;
	u8 t = 0;
	u8 t1 = 0;
	u8 temp1 =0;
	u8 temp2 =0;
	u16 y0=y;

	AECode = get_AEChar_bmp(num);
	
	if(AECode == 0)
	{
		return;
	}
	for(t = 0; t <size; t++)
	{   
		temp1 = *(AECode + t);
		temp2 = *(AECode + t + size);
		for(t1=0;t1<8;t1++)
		{			    
			if(temp1&0x1)LCD_Fast_DrawPoint(x,y0,POINT_COLOR);
			else if(mode==0)LCD_Fast_DrawPoint(x,y0,BACK_COLOR);


			if(temp2&0x1)LCD_Fast_DrawPoint(x,y0+8,POINT_COLOR);
			else if(mode==0)LCD_Fast_DrawPoint(x,y0+8,BACK_COLOR);
			temp1>>=1;
			temp2>>=1;
			y0++;
		} 
		x++;
		y0 = y;
	}  	    	   	 	  
}   
static int cmdindex =0;
u8 g_cmd[] = {
    
        //DIANCHUAN_CMD_SetMaxPower,       //IC卡、投币、最大功率设置     网关-->电川
        //DIANCHUAN_CMD_GetAllPortStatus     //读取设备每个端口的状态      网关-->电川
        //DIANCHUAN_CMD_StartPower          //付款成功开始充电     网关-->电川
        //, DIANCHUAN_CMD_CoinReport          //投币上报             电川-->网关
        //, DIANCHUAN_CMD_PowerComplete          //提交充电结束状态     电川-->网关
        //, DIANCHUAN_CMD_GetPortStatus       //查询端口当前的充电状态      网关-->电川
        //, DIANCHUAN_CMD_GetTotalConsumption       //查询消费总额数据      网关-->电川
        //, DIANCHUAN_CMD_SetICEnable       //设置IC卡、投币器是否可用     网关-->电川
       // , DIANCHUAN_CMD_ReadICCoinPower       //读取设备IC卡、投币、最大功率设置、刷卡和充满自停使能     ?
       // , DIANCHUAN_CMD_ClosePower       //远程停止某个端口的充电     网关-->电川
        //, DIANCHUAN_CMD_ReportError          //上传设备故障             电川-->网关
        //, DIANCHUAN_CMD_ReportData          //刷卡、退费成功按按键开始充电向模块发送卡号、扣费金额、卡类型、端口            电川-->网关
        //, DIANCHUAN_CMD_SetAutoFinish        //设置充电站充满自停、刷卡是否退费   网关-->电川
        //, DIANCHUAN_CMD_GetAllPortPowerStatus        //查询整机所有充电端口的充电状态   网关-->电川
        //, DIANCHUAN_CMD_GetVersonNumber        //读取设备的版本号   网关-->电川
       // , DIANCHUAN_CMD_SetUpdateVersonNumber        //软件版本更新   网关-->电川
        //, DIANCHUAN_CMD_SetFreemodeVolume        //设置免费充电模式、音量调节  网关-->电川
        //, DIANCHUAN_CMD_SetMinPowerTime        //设置最低浮充功率、浮充时间   网关-->电川
       // , DIANCHUAN_CMD_SetOnceICPowerTime        //刷卡充电时间   网关-->电川
       // , DIANCHUAN_CMD_ReadPowrModeVolumeTime        //读取设备刷卡充电时间、充电模式、语音音量、浮充功率、浮充时间   网关-->电川
/////////////////////////

        //BREAKER_CMD_ReportIDACK ,   //回复 A0
        BREAKER_CMD_OpenBreaker ,         //网关发送 【 打开电路器 】 命令
        BREAKER_CMD_ReadBreaker ,      //网关 发送 【 查询 】 命令
        BREAKER_CMD_CloseBreaker ,      // 网关 发送 【 关闭断路 】 器命令
        BREAKER_CMD_Calibrate       //发送校准命令

        };

u8 port_focus_HIGH = 18;
u8 port_number = 11;
u8 BreakerCmd_Y = 210;
u8 BreakerRXCmd = 0;
void Display_UpdatePortStatus(DianChuan_Board *port,u8 Portnum)
{
    u8 x,y = 0;
    u8 Xport = 10;
    u8 Xpower = 100;
    u8 Xstatus = 50;
    u8 i =0;
    u8 Xtime = 160;
    u8 buf[16]= {0};

	//LCD_ShowString(100,50,200,16,16,"AB");	
	
	//printf("Display_TemperatureTime .... \r\n");
	LCD_ShowAEChar(Xport,  y,((u16*)"端")[0],16,0);
	LCD_ShowAEChar(Xport+16,  y,((u16*)"口")[0],16,0);
	LCD_ShowAEChar(Xpower,   y,((u16*)"功")[0],16,0);
	LCD_ShowAEChar(Xpower+16,y,((u16*)"率")[0],16,0);
	LCD_ShowAEChar(Xstatus,   y,((u16*)"状")[0],16,0);
	LCD_ShowAEChar(Xstatus+16,y,((u16*)"态")[0],16,0);
    
	LCD_ShowAEChar(Xtime,   y,((u16*)"时")[0],16,0);
	LCD_ShowAEChar(Xtime+16,y,((u16*)"间")[0],16,0);

    
    LCD_ShowString(0,port_focus_HIGH,16,port_focus_HIGH*10,16,"                      "); 
    if(port_number == 11)
    {
        LCD_ShowString(0,port_focus_HIGH,16,port_focus_HIGH*10,16,"->->->->->->->->->->->"); 
    }
    else
    {
        LCD_ShowString(0,port_number*port_focus_HIGH,16,16,16,"->"); 
    }

	LCD_ShowAEChar(0,   BreakerCmd_Y,((u16*)"发")[0],16,0);
	LCD_ShowAEChar(16,BreakerCmd_Y,((u16*)"送")[0],16,0);
    
    sprintf(buf,": %02X    ", g_cmd[cmdindex]);
    LCD_ShowString(32,BreakerCmd_Y,64,20,16,buf);
    if(g_cmd[cmdindex] == BREAKER_CMD_OpenBreaker)
    {
        LCD_ShowAEChar(64,BreakerCmd_Y,((u16*)"打")[0],16,0);
        LCD_ShowAEChar(80,BreakerCmd_Y,((u16*)"开")[0],16,0);
    }
    else if(g_cmd[cmdindex] == BREAKER_CMD_ReadBreaker)
    {
        LCD_ShowAEChar(64,BreakerCmd_Y,((u16*)"查")[0],16,0);
        LCD_ShowAEChar(80,BreakerCmd_Y,((u16*)"询")[0],16,0);
    }
    else if(g_cmd[cmdindex] == BREAKER_CMD_CloseBreaker)
    {
        LCD_ShowAEChar(64,BreakerCmd_Y,((u16*)"关")[0],16,0);
        LCD_ShowAEChar(80,BreakerCmd_Y,((u16*)"闭")[0],16,0);
    }
    

	LCD_ShowAEChar(120,   BreakerCmd_Y,((u16*)"接")[0],16,0);
	LCD_ShowAEChar(136,BreakerCmd_Y,((u16*)"收")[0],16,0);
    
    strcpy(buf,"    ");
    if(BreakerRXCmd != 0)
    {
        sprintf(buf,": %02X", BreakerRXCmd);
    }
    LCD_ShowString(152,BreakerCmd_Y,32,16,16,buf);

    
    for (i =0 ;i<Portnum;i++)
    {
        y +=port_focus_HIGH;
        LCD_ShowxNum(Xport+10,y,i+1,2,16,0);

/*功率*/
        LCD_ShowxNum(Xpower,y,port->st[i].pow,4,16,0);

/*时间*/
        LCD_ShowxNum(Xtime,y,port->st[i].tl,4,16,0);

/*状态*/
        if(port->st[i].sst == 0x01)
        {
            LCD_ShowAEChar(Xstatus,   y,((u16*)"关")[0],16,0);
        }
        else if(port->st[i].sst == 0x02)
        {
            LCD_ShowAEChar(Xstatus,   y,((u16*)"开")[0],16,0);
        }
        else
        {
            LCD_ShowAEChar(Xstatus,   y,((u16*)"错")[0],16,0);
    	    //LCD_ShowAEChar(Xstatus+16,y,((u16*)"误")[0],16,0);
        }


    }
    
}
 void Display_RxFrameData(u8 *data,u8 len)
 {
    u8 i = 0;
    u16 x = 5;
    u16 y = 280;
    u8 w = 25;
    u8 buf[]= "                              ";
    //LCD_ShowString(x,y,240,20,16,buf);
    //LCD_ShowString(x,y+20,240,20,16,buf);
    LCD_ShowString(x,y,240,20,16,buf);
    LCD_ShowString(x,y+20,240,20,16,buf);
    strcpy(buf,"    ");
    if(BreakerRXCmd != 0)
    {
        sprintf(buf,": %02X", BreakerRXCmd);
    }
    LCD_ShowString(152,BreakerCmd_Y,40,16,16,buf);

    for (i =0 ;i<len;i++)
    {
        sprintf(buf,"%02X ",*(data+i));
        LCD_ShowString(x,y,w,20,16,buf);
        x += w;
        if(x > 240-w)
        {
            x =5;
            y+=20;
        }
    }
 }
 void Display_TxFrameData(u8 *data,u8 len)
 {
    u8 i = 0;
    u16 x = 5;
    u16 y = 230;
    u8 w = 25;
    u8 buf[]= "                              ";
    LCD_ShowString(x,y,240,20,16,buf);
    LCD_ShowString(x,y+20,240,20,16,buf);
    //LCD_ShowString(x,y+40,240,20,16,buf);
    //LCD_ShowString(x,y+60,240,20,16,buf);
    
    BreakerRXCmd = 0;
    LCD_ShowString(152,BreakerCmd_Y,40,16,16,"    ");

    for (i =0 ;i<len;i++)
    {
        sprintf(buf,"%02X ",*(data+i));
        LCD_ShowString(x,y,w,20,16,buf);
        x += w;
        if(x > 240-w)
        {
            x =5;
            y+=20;
        }
    }
}
 

extern void DianChuan_USART_SendData(u8 *buf, u16 len);

void Dianchuan_Test_TX(void)
{
	u8 sessionid[] = {0x00,0x00,0x00,0x00,0x00,0x21};


    switch(g_cmd[cmdindex])
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
            u8 data[] = {0x01,0x00,0x00,0x00,0x05};
            DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_StartPower,sessionid,data,sizeof(data));
            DianChuan_SendDataToTxLink(&DianChuanTxFrame);
        }
        break;
        case  DIANCHUAN_CMD_CoinReport:        //投币上报             电川-->网关
        {
            u8 data[] = {0x01,0x2};
            DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_CoinReport,sessionid,data,sizeof(data));
            DianChuan_SendDataToTxLink(&DianChuanTxFrame);
        }
        break;
        case  DIANCHUAN_CMD_PowerComplete:         //提交充电结束状态     电川-->网关
        {//待测试
        }
        break;

        case  DIANCHUAN_CMD_GetPortStatus:      //查询端口当前的充电状态      网关-->电川
        {
            u8 data[] = {0x01};
            DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetPortStatus,sessionid,data,sizeof(data));
            DianChuan_SendDataToTxLink(&DianChuanTxFrame);
        }
        break;
        case  DIANCHUAN_CMD_GetTotalConsumption:      //查询消费总额数据      网关-->电川
        {
            u8 data[] = {0x00};
            DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetTotalConsumption,sessionid,data,sizeof(data));
            DianChuan_SendDataToTxLink(&DianChuanTxFrame);

        }
        break;
        case  DIANCHUAN_CMD_SetMaxPower:      //IC卡、投币、最大功率设置     网关-->电川
        {
            u8 sessionid[] = {0x00,0x00,0x00,0x00,0x00,0x01};
            u8 data[] = {0x03,0x21,0x01,0x00,0xff,0x00,0xff,0x00,0xff};
            
            DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_SetMaxPower,sessionid,data,sizeof(data));
            DianChuan_SendDataToTxLink(&DianChuanTxFrame);

        }
        break;
        case  DIANCHUAN_CMD_SetICEnable:      //设置IC卡、投币器是否可用     网关-->电川
        {
            u8 sessionid[] = {0x00,0x00,0x00,0x00,0x00,0x01};
            u8 data[] = {0x01,0x01};
            
            DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_SetICEnable,sessionid,data,sizeof(data));
            DianChuan_SendDataToTxLink(&DianChuanTxFrame);

        }
        break;
        case  DIANCHUAN_CMD_ClosePower:      //远程停止某个端口的充电     网关-->电川
        {
            u8 sessionid[] = {0x00,0x00,0x00,0x00,0x00,0x01};
            u8 data[] = {0x01};
            
            DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_ClosePower,sessionid,data,sizeof(data));
            DianChuan_SendDataToTxLink(&DianChuanTxFrame);

        }
        break;
        case  DIANCHUAN_CMD_ReadICCoinPower:      //读取设备IC卡、投币、最大功率设置、刷卡和充满自停使能     ?
        {
            u8 data[] = {0x00};
            DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_ReadICCoinPower,sessionid,data,sizeof(data));
            DianChuan_SendDataToTxLink(&DianChuanTxFrame);
        }
        break;
        case  DIANCHUAN_CMD_ReportError:         //上传设备故障             电川-->网关
        {
            //待测试
            //u8 data[] = {0x00};
           // DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetTotalConsumption,sessionid,data,sizeof(data));
            //DianChuan_SendDataToTxLink(&DianChuanTxFrame);

        }
        break;
        case  DIANCHUAN_CMD_SetAutoFinish:       //设置充电站充满自停、刷卡是否退费   网关-->电川
        {
            u8 data[] = {0x01,0x00};
            DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_SetAutoFinish,sessionid,data,sizeof(data));
            DianChuan_SendDataToTxLink(&DianChuanTxFrame);

        }
        break;

        case  DIANCHUAN_CMD_GetAllPortPowerStatus:       //查询整机所有充电端口的充电状态   网关-->电川
        {
            u8 data[] = {0x00};
            DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetAllPortPowerStatus,sessionid,data,sizeof(data));
            DianChuan_SendDataToTxLink(&DianChuanTxFrame);
        }
        break;

        case  DIANCHUAN_CMD_GetVersonNumber:       //读取设备的版本号   网关-->电川
        {
            u8 data[] = {0x00};
            DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetVersonNumber,sessionid,data,sizeof(data));
            DianChuan_SendDataToTxLink(&DianChuanTxFrame);

        }
        break;

        case  DIANCHUAN_CMD_SetUpdateVersonNumber:       //软件版本更新   网关-->电川
        {
            u8 data[] = {0x01};
            DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_SetUpdateVersonNumber,sessionid,data,sizeof(data));
            DianChuan_SendDataToTxLink(&DianChuanTxFrame);
        }
        break;

        case  DIANCHUAN_CMD_SetFreemodeVolume:       //设置免费充电模式、音量调节   网关-->电川
        {
            u8 data[] = {0x00,0x01};
            DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_SetFreemodeVolume,sessionid,data,sizeof(data));
            DianChuan_SendDataToTxLink(&DianChuanTxFrame);
        }
        break;

        case  DIANCHUAN_CMD_SetMinPowerTime:       //设置最低浮充功率、浮充时间   网关-->电川
        {
            u8 data[] = {0x01,0xF4,0x00,0xB4};
            DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_SetMinPowerTime,sessionid,data,sizeof(data));
            DianChuan_SendDataToTxLink(&DianChuanTxFrame);

        }
        break;

        case  DIANCHUAN_CMD_SetOnceICPowerTime:       //刷卡充电时间   网关-->电川
        {
            u8 data[] = {0x00,0x10,0x00,0x10,0x00,0x10};
            DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_SetOnceICPowerTime,sessionid,data,sizeof(data));
            DianChuan_SendDataToTxLink(&DianChuanTxFrame);

        }
        break;

        case  DIANCHUAN_CMD_ReadPowrModeVolumeTime:       //读取设备刷卡充电时间、充电模式、语音音量、浮充功率、浮充时间   网关-->电川
        {
            u8 data[] = {0x00};
            DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_ReadPowrModeVolumeTime,sessionid,data,sizeof(data));
            DianChuan_SendDataToTxLink(&DianChuanTxFrame);
        }
        break;
    }
    
    cmdindex = (cmdindex+1)%sizeof(g_cmd);
}
 void breaker_Test_TX(void)
 {
     u8 sessionid[] = {0x00,0x00,0x00,0x00,0x00,0x21};
 
 
     switch(g_cmd[cmdindex])
     {

         case BREAKER_CMD_ReportIDACK:   //回复 A0
         case BREAKER_CMD_OpenBreaker:
         case BREAKER_CMD_ReadBreaker:      //网关 发送 【 查询 】 命令
         case BREAKER_CMD_CloseBreaker:      // 网关 发送 【 关闭断路 】 器命令
         case BREAKER_CMD_Calibrate:      //发送校准命令
         {
             //u8 data[] = {0x5A,0x0F,0xA2,0x00,0x00,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0};
             u8 id[] = {0x00,0x00,0x00,0x00};
             u8 data[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00};
             if(port_number > 10)
             {
                u8 i = 0;
                for(i = 1; i < port_number;i++)
                {
                    id[3] = i;
                    Breaker_ConstructTxCmdFrame(&BreakerTxFrame,g_cmd[cmdindex],id,data,sizeof(data));
                    Breaker_SendDataToTxLink(&BreakerTxFrame);
                }
             }
             else
             {
                 id[3] = port_number;
                 Breaker_ConstructTxCmdFrame(&BreakerTxFrame,g_cmd[cmdindex],id,data,sizeof(data));
                 Breaker_SendDataToTxLink(&BreakerTxFrame);
             }
         }
         break;
     }
     
     //cmdindex = (cmdindex+1)%sizeof(g_cmd);
 }

 /******* main() 
 *********/
 int main(void)
 {	
	int time = 0;
	u8 sessionid[] = {0x00,0x00,0x00,0x00,0x00,0x01};
	init();
    POINT_COLOR=RED;
#ifdef USE_DMA_READ_ADC
    DMA1_Config_ADC1(DMA1_Channel1,(u32)&ADC1->DR,(u32)&adc_array,CH_NUM*10);
    DMA_ReadADC_Enable(DMA1_Channel1);
    
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能	
#endif

	while(1)
	{
		switch(KEY_Scan(0))//
		{
			case KEY0_PRES://循环测试命令
			{
				//printf("KEY_Scan .... %d\r\n",KEY0_PRES);
				//Dianchuan_Test_TX();
				breaker_Test_TX();
			}
			break;
			case KEY1_PRES: 
			{
				u8 buf[16] = {0};
				//printf("KEY_Scan .... %d\r\n",KEY1_PRES);
				cmdindex = (cmdindex+1)%sizeof(g_cmd);
                    sprintf(buf,": %02X    ", g_cmd[cmdindex]);
                LCD_ShowString(32,BreakerCmd_Y,64,16,16,buf);
                if(g_cmd[cmdindex] == BREAKER_CMD_OpenBreaker)
                {
                    LCD_ShowAEChar(64,BreakerCmd_Y,((u16*)"打")[0],16,0);
                    LCD_ShowAEChar(80,BreakerCmd_Y,((u16*)"开")[0],16,0);
                }
                else if(g_cmd[cmdindex] == BREAKER_CMD_ReadBreaker)
                {
                    LCD_ShowAEChar(64,BreakerCmd_Y,((u16*)"查")[0],16,0);
                    LCD_ShowAEChar(80,BreakerCmd_Y,((u16*)"询")[0],16,0);
                }
                else if(g_cmd[cmdindex] == BREAKER_CMD_CloseBreaker)
                {
                    LCD_ShowAEChar(64,BreakerCmd_Y,((u16*)"关")[0],16,0);
                    LCD_ShowAEChar(80,BreakerCmd_Y,((u16*)"闭")[0],16,0);
                }
				#if 0
                DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetPortStatus,sessionid,data,1);data[0]++;
                DianChuan_SendDataToTxLink(&DianChuanTxFrame);
                DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetPortStatus,sessionid,data,1);data[0]++;
                DianChuan_SendDataToTxLink(&DianChuanTxFrame);
                DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetPortStatus,sessionid,data,1);data[0]++;
                DianChuan_SendDataToTxLink(&DianChuanTxFrame);
                DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetPortStatus,sessionid,data,1);data[0]++;
                DianChuan_SendDataToTxLink(&DianChuanTxFrame);
                DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetPortStatus,sessionid,data,1);data[0]++;
                DianChuan_SendDataToTxLink(&DianChuanTxFrame);
                
                DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetPortStatus,sessionid,data,1);data[0]++;
                DianChuan_SendDataToTxLink(&DianChuanTxFrame);
                DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetPortStatus,sessionid,data,1);data[0]++;
                DianChuan_SendDataToTxLink(&DianChuanTxFrame);
                DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetPortStatus,sessionid,data,1);data[0]++;
                DianChuan_SendDataToTxLink(&DianChuanTxFrame);
                DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetPortStatus,sessionid,data,1);data[0]++;
                DianChuan_SendDataToTxLink(&DianChuanTxFrame);
                DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetPortStatus,sessionid,data,1);data[0]++;
                DianChuan_SendDataToTxLink(&DianChuanTxFrame);
                #endif
			}
			break;
			case WKUP_PRES:
			{

			}
			break;
		}
		delay_ms(10);        
        ++time;
#if 1 //调试
        if(time%51 == 0) //500ms分钟读一次
        {
            Display_UpdatePortStatus(&DianChuanBoard,DIANCHUANPORTNUM);
	    }
#endif
	}
 }

 //通用定时器中断初始化
//这里使用的是定时器3!
void Main_timer_Init_1second(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能

	TIM_TimeBaseStructure.TIM_Period = 5000; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =SystemCoreClock/10000; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(  //使能或者失能指定的TIM中断
		TIM3, //TIM2
		TIM_IT_Update ,
		ENABLE  //使能
		);
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	TIM_Cmd(TIM3, ENABLE);  //使能TIMx外设
							 
}
 
extern void Breaker_SendDataFromTxLink (void);

/***********Main_timer_Init_1second************/
void TIM3_IRQHandler(void)   //TIM3中断
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源 
        //DianChuan_SendDataFromTxLink ();
        Breaker_SendDataFromTxLink();
	}
}
    

void EXTI0_IRQHandler(void)
{
    delay_ms(10);
    if(EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        u8 sessionid[] = {0x00,0x00,0x00,0x00,0x00,0x21};
        //printf("KEY_Scan ........\r\n");
        #if 0
        //u8 buf[]={0x66,0x13,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x0A,0x02, 0x01,0x01,0x01,0x01, 0x01,0x01,0x01,0x01,0x01,0x1B};
        //USART_DianChuan_SendData(buf,sizeof(buf));
        u8 data[] = {0x01,0x00,0x00,0x00,0x64};
        DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_StartPower,sessionid,data,1);data[0]++;
        DianChuan_SendDataToTxLink(&DianChuanTxFrame);
        DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_StartPower,sessionid,data,1);data[0]++;
        DianChuan_SendDataToTxLink(&DianChuanTxFrame);
        DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_StartPower,sessionid,data,1);data[0]++;
        DianChuan_SendDataToTxLink(&DianChuanTxFrame);
        DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_StartPower,sessionid,data,1);data[0]++;
        DianChuan_SendDataToTxLink(&DianChuanTxFrame);
        DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_StartPower,sessionid,data,1);data[0]++;
        DianChuan_SendDataToTxLink(&DianChuanTxFrame);
        
        DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_StartPower,sessionid,data,1);data[0]++;
        DianChuan_SendDataToTxLink(&DianChuanTxFrame);
        DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_StartPower,sessionid,data,1);data[0]++;
        DianChuan_SendDataToTxLink(&DianChuanTxFrame);
        DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_StartPower,sessionid,data,1);data[0]++;
        DianChuan_SendDataToTxLink(&DianChuanTxFrame);
        DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_StartPower,sessionid,data,1);data[0]++;
        DianChuan_SendDataToTxLink(&DianChuanTxFrame);
        DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_StartPower,sessionid,data,1);data[0]++;
        DianChuan_SendDataToTxLink(&DianChuanTxFrame); 
#endif
#if 0
    u8 data[] = {0x01};
    //printf("KEY_Scan .... %d\r\n",KEY1_PRES);
    DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetPortStatus,sessionid,data,1);data[0]++;
    DianChuan_SendDataToTxLink(&DianChuanTxFrame);
    DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetPortStatus,sessionid,data,1);data[0]++;
    DianChuan_SendDataToTxLink(&DianChuanTxFrame);
    DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetPortStatus,sessionid,data,1);data[0]++;
    DianChuan_SendDataToTxLink(&DianChuanTxFrame);
    DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetPortStatus,sessionid,data,1);data[0]++;
    DianChuan_SendDataToTxLink(&DianChuanTxFrame);
    DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetPortStatus,sessionid,data,1);data[0]++;
    DianChuan_SendDataToTxLink(&DianChuanTxFrame);
    
    DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetPortStatus,sessionid,data,1);data[0]++;
    DianChuan_SendDataToTxLink(&DianChuanTxFrame);
    DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetPortStatus,sessionid,data,1);data[0]++;
    DianChuan_SendDataToTxLink(&DianChuanTxFrame);
    DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetPortStatus,sessionid,data,1);data[0]++;
    DianChuan_SendDataToTxLink(&DianChuanTxFrame);
    DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetPortStatus,sessionid,data,1);data[0]++;
    DianChuan_SendDataToTxLink(&DianChuanTxFrame);
    DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_GetPortStatus,sessionid,data,1);data[0]++;
    DianChuan_SendDataToTxLink(&DianChuanTxFrame);
        #endif
        LCD_ShowString(0,port_focus_HIGH,16,port_focus_HIGH*10,16,"                      "); 
        port_number ++;
        if (port_number > 11)
        {
            port_number = 1;
        }
        
        if(port_number == 11)
        {
            LCD_ShowString(0,port_focus_HIGH,16,port_focus_HIGH*10,16,"->->->->->->->->->->->"); 
        }
        else
        {
            LCD_ShowString(0,port_number*port_focus_HIGH,16,16,16,"->"); 
        }

    }
 
    EXTI_ClearITPendingBit(EXTI_Line0);
}

void DMA1_Channel1_IRQHandler(void)//DMA1_Channel1_IRQHandler
 {
	if((DMA1->ISR)&(1<<1))        //判断传输完成中断位是否置位
	{
		DMA1->IFCR |= 1<<1; //清除完成传输中断
		LED1=!LED1; 
	} 
}


