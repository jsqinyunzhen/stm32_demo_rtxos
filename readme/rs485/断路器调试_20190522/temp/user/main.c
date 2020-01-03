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
    //RCC_DeInit(); //��ϵͳ�Ῠ
	LED_Init();
	delay_init();
	uart_init(4800);
    //RS485_Config(9600);
	LCD_Init();
	Adc_Init();
	KEY_Init();
	Main_timer_Init_1second();

	while(RTC_Init())		//RTC��ʼ��	��һ��Ҫ��ʼ���ɹ�
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
    
        //DIANCHUAN_CMD_SetMaxPower,       //IC����Ͷ�ҡ����������     ����-->�紨
        //DIANCHUAN_CMD_GetAllPortStatus     //��ȡ�豸ÿ���˿ڵ�״̬      ����-->�紨
        //DIANCHUAN_CMD_StartPower          //����ɹ���ʼ���     ����-->�紨
        //, DIANCHUAN_CMD_CoinReport          //Ͷ���ϱ�             �紨-->����
        //, DIANCHUAN_CMD_PowerComplete          //�ύ������״̬     �紨-->����
        //, DIANCHUAN_CMD_GetPortStatus       //��ѯ�˿ڵ�ǰ�ĳ��״̬      ����-->�紨
        //, DIANCHUAN_CMD_GetTotalConsumption       //��ѯ�����ܶ�����      ����-->�紨
        //, DIANCHUAN_CMD_SetICEnable       //����IC����Ͷ�����Ƿ����     ����-->�紨
       // , DIANCHUAN_CMD_ReadICCoinPower       //��ȡ�豸IC����Ͷ�ҡ���������á�ˢ���ͳ�����ͣʹ��     ?
       // , DIANCHUAN_CMD_ClosePower       //Զ��ֹͣĳ���˿ڵĳ��     ����-->�紨
        //, DIANCHUAN_CMD_ReportError          //�ϴ��豸����             �紨-->����
        //, DIANCHUAN_CMD_ReportData          //ˢ�����˷ѳɹ���������ʼ�����ģ�鷢�Ϳ��š��۷ѽ������͡��˿�            �紨-->����
        //, DIANCHUAN_CMD_SetAutoFinish        //���ó��վ������ͣ��ˢ���Ƿ��˷�   ����-->�紨
        //, DIANCHUAN_CMD_GetAllPortPowerStatus        //��ѯ�������г��˿ڵĳ��״̬   ����-->�紨
        //, DIANCHUAN_CMD_GetVersonNumber        //��ȡ�豸�İ汾��   ����-->�紨
       // , DIANCHUAN_CMD_SetUpdateVersonNumber        //����汾����   ����-->�紨
        //, DIANCHUAN_CMD_SetFreemodeVolume        //������ѳ��ģʽ����������  ����-->�紨
        //, DIANCHUAN_CMD_SetMinPowerTime        //������͸��书�ʡ�����ʱ��   ����-->�紨
       // , DIANCHUAN_CMD_SetOnceICPowerTime        //ˢ�����ʱ��   ����-->�紨
       // , DIANCHUAN_CMD_ReadPowrModeVolumeTime        //��ȡ�豸ˢ�����ʱ�䡢���ģʽ���������������书�ʡ�����ʱ��   ����-->�紨
/////////////////////////

        //BREAKER_CMD_ReportIDACK ,   //�ظ� A0
        BREAKER_CMD_OpenBreaker ,         //���ط��� �� �򿪵�·�� �� ����
        BREAKER_CMD_ReadBreaker ,      //���� ���� �� ��ѯ �� ����
        BREAKER_CMD_CloseBreaker ,      // ���� ���� �� �رն�· �� ������
        BREAKER_CMD_Calibrate       //����У׼����

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
	LCD_ShowAEChar(Xport,  y,((u16*)"��")[0],16,0);
	LCD_ShowAEChar(Xport+16,  y,((u16*)"��")[0],16,0);
	LCD_ShowAEChar(Xpower,   y,((u16*)"��")[0],16,0);
	LCD_ShowAEChar(Xpower+16,y,((u16*)"��")[0],16,0);
	LCD_ShowAEChar(Xstatus,   y,((u16*)"״")[0],16,0);
	LCD_ShowAEChar(Xstatus+16,y,((u16*)"̬")[0],16,0);
    
	LCD_ShowAEChar(Xtime,   y,((u16*)"ʱ")[0],16,0);
	LCD_ShowAEChar(Xtime+16,y,((u16*)"��")[0],16,0);

    
    LCD_ShowString(0,port_focus_HIGH,16,port_focus_HIGH*10,16,"                      "); 
    if(port_number == 11)
    {
        LCD_ShowString(0,port_focus_HIGH,16,port_focus_HIGH*10,16,"->->->->->->->->->->->"); 
    }
    else
    {
        LCD_ShowString(0,port_number*port_focus_HIGH,16,16,16,"->"); 
    }

	LCD_ShowAEChar(0,   BreakerCmd_Y,((u16*)"��")[0],16,0);
	LCD_ShowAEChar(16,BreakerCmd_Y,((u16*)"��")[0],16,0);
    
    sprintf(buf,": %02X    ", g_cmd[cmdindex]);
    LCD_ShowString(32,BreakerCmd_Y,64,20,16,buf);
    if(g_cmd[cmdindex] == BREAKER_CMD_OpenBreaker)
    {
        LCD_ShowAEChar(64,BreakerCmd_Y,((u16*)"��")[0],16,0);
        LCD_ShowAEChar(80,BreakerCmd_Y,((u16*)"��")[0],16,0);
    }
    else if(g_cmd[cmdindex] == BREAKER_CMD_ReadBreaker)
    {
        LCD_ShowAEChar(64,BreakerCmd_Y,((u16*)"��")[0],16,0);
        LCD_ShowAEChar(80,BreakerCmd_Y,((u16*)"ѯ")[0],16,0);
    }
    else if(g_cmd[cmdindex] == BREAKER_CMD_CloseBreaker)
    {
        LCD_ShowAEChar(64,BreakerCmd_Y,((u16*)"��")[0],16,0);
        LCD_ShowAEChar(80,BreakerCmd_Y,((u16*)"��")[0],16,0);
    }
    

	LCD_ShowAEChar(120,   BreakerCmd_Y,((u16*)"��")[0],16,0);
	LCD_ShowAEChar(136,BreakerCmd_Y,((u16*)"��")[0],16,0);
    
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

/*����*/
        LCD_ShowxNum(Xpower,y,port->st[i].pow,4,16,0);

/*ʱ��*/
        LCD_ShowxNum(Xtime,y,port->st[i].tl,4,16,0);

/*״̬*/
        if(port->st[i].sst == 0x01)
        {
            LCD_ShowAEChar(Xstatus,   y,((u16*)"��")[0],16,0);
        }
        else if(port->st[i].sst == 0x02)
        {
            LCD_ShowAEChar(Xstatus,   y,((u16*)"��")[0],16,0);
        }
        else
        {
            LCD_ShowAEChar(Xstatus,   y,((u16*)"��")[0],16,0);
    	    //LCD_ShowAEChar(Xstatus+16,y,((u16*)"��")[0],16,0);
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
        case  DIANCHUAN_CMD_CoinReport:        //Ͷ���ϱ�             �紨-->����
        {
            u8 data[] = {0x01,0x2};
            DianChuan_ConstructTxCmdFrame(&DianChuanTxFrame,DIANCHUAN_CMD_CoinReport,sessionid,data,sizeof(data));
            DianChuan_SendDataToTxLink(&DianChuanTxFrame);
        }
        break;
        case  DIANCHUAN_CMD_PowerComplete:         //�ύ������״̬     �紨-->����
        {//������
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
            u8 data[] = {0x03,0x21,0x01,0x00,0xff,0x00,0xff,0x00,0xff};
            
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
            u8 data[] = {0x01,0xF4,0x00,0xB4};
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
    
    cmdindex = (cmdindex+1)%sizeof(g_cmd);
}
 void breaker_Test_TX(void)
 {
     u8 sessionid[] = {0x00,0x00,0x00,0x00,0x00,0x21};
 
 
     switch(g_cmd[cmdindex])
     {

         case BREAKER_CMD_ReportIDACK:   //�ظ� A0
         case BREAKER_CMD_OpenBreaker:
         case BREAKER_CMD_ReadBreaker:      //���� ���� �� ��ѯ �� ����
         case BREAKER_CMD_CloseBreaker:      // ���� ���� �� �رն�· �� ������
         case BREAKER_CMD_Calibrate:      //����У׼����
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
    
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������	
#endif

	while(1)
	{
		switch(KEY_Scan(0))//
		{
			case KEY0_PRES://ѭ����������
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
                    LCD_ShowAEChar(64,BreakerCmd_Y,((u16*)"��")[0],16,0);
                    LCD_ShowAEChar(80,BreakerCmd_Y,((u16*)"��")[0],16,0);
                }
                else if(g_cmd[cmdindex] == BREAKER_CMD_ReadBreaker)
                {
                    LCD_ShowAEChar(64,BreakerCmd_Y,((u16*)"��")[0],16,0);
                    LCD_ShowAEChar(80,BreakerCmd_Y,((u16*)"ѯ")[0],16,0);
                }
                else if(g_cmd[cmdindex] == BREAKER_CMD_CloseBreaker)
                {
                    LCD_ShowAEChar(64,BreakerCmd_Y,((u16*)"��")[0],16,0);
                    LCD_ShowAEChar(80,BreakerCmd_Y,((u16*)"��")[0],16,0);
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
#if 1 //����
        if(time%51 == 0) //500ms���Ӷ�һ��
        {
            Display_UpdatePortStatus(&DianChuanBoard,DIANCHUANPORTNUM);
	    }
#endif
	}
 }

 //ͨ�ö�ʱ���жϳ�ʼ��
//����ʹ�õ��Ƕ�ʱ��3!
void Main_timer_Init_1second(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��

	TIM_TimeBaseStructure.TIM_Period = 5000; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM_TimeBaseStructure.TIM_Prescaler =SystemCoreClock/10000; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(  //ʹ�ܻ���ʧ��ָ����TIM�ж�
		TIM3, //TIM2
		TIM_IT_Update ,
		ENABLE  //ʹ��
		);
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx����
							 
}
 
extern void Breaker_SendDataFromTxLink (void);

/***********Main_timer_Init_1second************/
void TIM3_IRQHandler(void)   //TIM3�ж�
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ 
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
	if((DMA1->ISR)&(1<<1))        //�жϴ�������ж�λ�Ƿ���λ
	{
		DMA1->IFCR |= 1<<1; //�����ɴ����ж�
		LED1=!LED1; 
	} 
}


