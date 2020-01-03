
一.20190805
  1.rtx os port
  #include CMSIS_device_header
	#define CMSIS_device_header "stm32f10x.h"
  2.uart1 ->uart_printf与printf
  printf需要keil选择上microlib
  printf  打印末尾需要 r\n，否则securt显示换行异常
  3.thread api demo
  https://blog.csdn.net/wylwws/article/details/71191507
  http://www.keil.com/pack/doc/CMSIS/RTOS2/html/group__CMSIS__RTOS__KernelCtrl.html 
二.20190806 
  rtx api demo https://www.cnblogs.com/shangdawei/p/3854191.html
  
  cmsis pack 更新到ARM.CMSIS.5.5.1.pack，下载地址可以是csdn或者 http://www.keil.com/dd2/pack/
  
三.20190812
  rtx os  header file
  C:\Keil_v5\ARM\Pack\ARM\CMSIS\5.5.1\CMSIS\RTOS2\RTX\Include1;
  C:\Keil_v5\ARM\Pack\ARM\CMSIS\5.5.1\CMSIS\RTOS2\RTX\Source;//rtx os
  C:\Keil_v5\ARM\Pack\ARM\CMSIS\5.5.1\CMSIS\Core\Include;//cmsis
  .\RTE;.\RTE\Compiler;.\middleware;.\middleware\config
  
  event record not success 
  C:\Keil_v5\ARM\PACK\Keil\ARM_Compiler\1.4.0\Doc\EventRecorder\html\er_use.html 
四.20190813
  为什么中断处理函数中不能使用printf语句
  https://blog.csdn.net/chenyefei/article/details/82495129
  
  nvic 
  gpio exti
5.20190814
  rtc alarm int
  PC13 BKP_RTCOutputConfig(BKP_RTCOutputSource_CalibClock);
  
  PA.8 MCO
  
 6.20190819
    d.spi1 
	//spi flash USE_STM3210B_EVAL
	  /* 配置 SPI1 引脚: SCK, MISO 和 MOSI */
    PA GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
 
 	PIN30 ---PA5---SCK
	PIN31 ---PA6---MISO
	PIN32 ---PA7---MOSI 
	PIN29 ---PA4---SPI CS/
	
	j:pwm--TIM4 CH1 AUDIO
	PIN92 ---PB6 --
7.20180829
	Astyle https://blog.csdn.net/u010160335/article/details/78587411
	
	dc1--uart3
	dc2--uart4
	C:\mqtt\apache-activemq-5.15.9\bin\win64   activemq.bat start
	
	dc
DIANCHUAN_CMD_GetAllPortStatus (0x01)   
	T:EE 09 01 00 00 00 00 00 00 00 08
	R:66 13 01 00 00 00 00 00 00 0A 01 01 01 01 01 01 01 01 01 01 18 
DIANCHUAN_CMD_GetTotalConsumption (0x07)  	
	T:ee 09 07 00 00 00 00 00 00 00 0e
	R:66 0C 07 00 00 00 00 00 00 00 00 00 00 0B 
DIANCHUAN_CMD_ReadICCoinPower (0x0c)    
	T:ee 09 0c 00 00 00 00 00 00 00 05 
	R:66 11 0C 00 00 00 00 00 00 01 F4 00 00 FF 00 FF 00 FF 17 
DIANCHUAN_CMD_GetPortStatus (0x06)  	
	T:ee 09 06 00 00 00 00 00 00 01 0e 0 2a
	R:66 0D 06 00 00 00 00 00 00 01 00 00 00 00 0A 

DIANCHUAN_CMD_GetAllPortPowerStatus (0x24)  
	T:ee 09 24 00 00 00 00 00 00 00 2d
	R:no response

DIANCHUAN_CMD_SetMaxPower (0x08)
	T:ee 11 08 00 00 00 00 00 00 01 f4 00 00 ff 00 ff 00 ff 13
	R:66 09 08 00 00 00 00 00 00 01 00 

DIANCHUAN_CMD_StartPower (0x02)    	//charge_mg_set_on(1,3,200,100,30);
	T:ee 0d 02 00 00 00 00 00 00 01 00 00 00 03 0d
	R:66 0A 02 00 00 00 00 00 00 01 01 08 
	
	
	
	--------------------------
 DIANCHUAN_CMD_PowerComplete (0x05)   
 R:EE 0C 05 30 30 30 30 30 30 01 00 01 01 08 
   EE 0C 05 30 30 30 30 30 30 01 00 01 01 08
 t:66 09 05 00 00 00 00 00 00 01 08
 
 SIM 卡与4G天线确保
 AT+CPIN?  AT+CSQ
 http://common-download.oss-cn-hangzhou.aliyuncs.com/jindoo/charge-1.0.0.4-ENCODE.bin