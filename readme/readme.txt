
һ.20190805
  1.rtx os port
  #include CMSIS_device_header
	#define CMSIS_device_header "stm32f10x.h"
  2.uart1 ->uart_printf��printf
  printf��Ҫkeilѡ����microlib
  printf  ��ӡĩβ��Ҫ r\n������securt��ʾ�����쳣
  3.thread api demo
  https://blog.csdn.net/wylwws/article/details/71191507
  http://www.keil.com/pack/doc/CMSIS/RTOS2/html/group__CMSIS__RTOS__KernelCtrl.html 
��.20190806 
  rtx api demo https://www.cnblogs.com/shangdawei/p/3854191.html
  
  cmsis pack ���µ�ARM.CMSIS.5.5.1.pack�����ص�ַ������csdn���� http://www.keil.com/dd2/pack/
  
��.20190812
  rtx os  header file
  C:\Keil_v5\ARM\Pack\ARM\CMSIS\5.5.1\CMSIS\RTOS2\RTX\Include1;
  C:\Keil_v5\ARM\Pack\ARM\CMSIS\5.5.1\CMSIS\RTOS2\RTX\Source;//rtx os
  C:\Keil_v5\ARM\Pack\ARM\CMSIS\5.5.1\CMSIS\Core\Include;//cmsis
  .\RTE;.\RTE\Compiler;.\middleware;.\middleware\config
  
  event record not success 
  C:\Keil_v5\ARM\PACK\Keil\ARM_Compiler\1.4.0\Doc\EventRecorder\html\er_use.html 
��.20190813
  Ϊʲô�жϴ������в���ʹ��printf���
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
	  /* ���� SPI1 ����: SCK, MISO �� MOSI */
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
 
 1.SIM ����4G����ȷ��
 AT+CPIN?  AT+CSQ
 
 2.cregע�����ˣ�����cgregû��ע���ϣ�Ƿ�ѵ�����

set uart2 rx complete:29=AT+CREG?
+CREG: 0,1

OK

cmd:AT+CGREG?
 result:AT+CGREG?
+CGREG: 0,2

OK


upgrade file:
 http://common-download.oss-cn-hangzhou.aliyuncs.com/jindoo/charge-1.0.0.4-ENCODE.bin
 
 
 http:
 https://www.baidu.com/img/baidu_logo.gif
 
 
 138 6700 0927
QQ1055265937


485 id:
5A 0F A1 00 00 00 04 00 00 00 00 00 00 00 F0 
5A 0F A1 00 00 00 05 00 00 00 00 00 00 00 F1 


uart_buf_len =22 =app_rs485_broker_test
send:
5a 0f a0 00 00 00 04 00 00 00 00 00 00 00 f1 

send:
5a 0f a1 00 00 00 04 00 00 00 00 00 00 00 f0 
send:
//open
5a 0f a2 00 00 00 04 00 00 00 00 00 00 00 f3 
//open ack
5a 0f a3 00 00 00 04 00 00 00 00 00 00 00 f2 
5A 0F A3 00 00 00 04 00 00 00 00 00 00 01 F3--
//lookup
5a 0f a4 00 00 00 04 00 00 00 00 00 00 00 f5 
//lookup ack
5a 0f a5 00 00 00 04 00 00 00 00 00 00 00 f4 
5A 0F A5 00 00 00 04 09 23 00 00 00 00 01 DF --
 
//close
5a 0f a6 00 00 00 04 00 00 00 00 00 00 00 f7 
//close ack
5a 0f a7 00 00 00 04 00 00 00 00 00 00 00 f6 
5A 0F A7 00 00 00 04 00 00 00 00 00 00 00 F6--

app_rs485_broker_test