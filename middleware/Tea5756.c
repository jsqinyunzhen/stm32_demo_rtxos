/****************************************************************************
* Copyright (C), 2011 �ܶ�Ƕ��ʽ������ www.ourstm.net
*
* �������� �ܶ���STM32������V2,2.1,V3,MINI�ϵ���ͨ��           
* QQ: 9191274, ������sun68, Email: sun68@163.com 
* �Ա����̣�ourstm.taobao.com  
*
* �ļ���: tea5767.c
* ���ݼ���:
*       �����������TEA5767�ĵײ���������
*
* �ļ���ʷ:
* �汾��  ����       ����    ˵��
* v0.2    2011-7-06 sun68  �������ļ�
*
*/


/* Includes ------------------------------------------------------------------*/

#include "stm32f10x.h"
#include "project_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* I2C�����ߵĶ��� */

#define TEA_SCL_IN  {GPIOB->CRH&=0XFFFFF0FF;GPIOB->CRH|=8<<8;}
#define TEA_SCL_OUT {GPIOB->CRH&=0XFFFFF0FF;GPIOB->CRH|=3<<8;}

#define TEA_SDA_IN  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}
#define TEA_SDA_OUT {GPIOB->CRH&=0XFFFF0fFF;GPIOB->CRH|=3<<12;}

#define SCL_H       GPIOB->BSRR = GPIO_Pin_10			   
#define SCL_L       GPIOB->BRR  = GPIO_Pin_10 
   
#define SDA_H        GPIOB->BSRR = GPIO_Pin_11 
#define SDA_L         GPIOB->BRR  = GPIO_Pin_11 

#define SCL_read    GPIOB->IDR  & GPIO_Pin_10
#define SDA_read     GPIOB->IDR  & GPIO_Pin_11




#define Tea5767_WriteAddress1    0xc0
#define Tea5767_ReadAddress1     0xc1

/* Private variables ---------------------------------------------------------*/
uint8_t Tx1_Buffer[] = {0XF0,0X2C,0XD0,0X12,0X40};
uint8_t Rx1_Buffer[] = {0XF0,0X2C,0XD0,0X12,0X40};
uint8_t Tx2_Buffer[] = {0X00,0Xc0,0xe0,0X41,0X6e,0X7e};
//uint8_t Rx1_Buffer[BufferSize1], Rx2_Buffer[BufferSize2];	  

unsigned long   FM_FREQ=105300000;		  //Ĭ��������ͨ�㲥98.8MHz
//unsigned long   FM_FREQ=91600000;
unsigned long FM_PLL;
unsigned char PLL_HIGH=0;
unsigned char PLL_LOW=0;
unsigned char len=0; 
static unsigned char rec_f=2;
unsigned char ch2=0;

void I2C_GPIO_Config(void);


void I2C_delay(void);
bool I2C_Start(void);

void I2C_Stop(void);

void I2C_Ack(void);

void I2C_NoAck(void);


bool I2C_WaitAck(void);
void I2C_SendByte(u8 SendByte);
void I2C_FM_Init(void);
bool I2C_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite);
//extern void Delay(__IO uint32_t nCount);
static void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}
void FM_Configuration(void);

/****************************************************************************
* ��    �ƣ�void FM_Configuration(void)
* ��    �ܣ�I2C FM������ģ��TEA5767�����ߵĳ�ʼ�� 
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
void FM_Configuration(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;    
  /* ����PB10,PB11ΪI2C�� SCL SDL */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;		    
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;//GPIO_Mode_IPU;                                               
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}


/****************************************************************************
* ��    �ƣ�void I2C_EE_Init()
* ��    �ܣ�FM ��ʼ��
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
void I2C_FM_Init(void)
{
  /* I2C���������� */
  FM_Configuration();

  /* I2C ��ʼ�� */
  //I2C_Configuration();  
}

/****************************************************************************
* ��    �ƣ�bool I2C_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite)
* ��    �ܣ�I2Cд
* ��ڲ�����uint8_t* pBuffer --��д�������  uint8_t WriteAddr--������ַ  uint8_t NumByteToWrite--д����ֽ���
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
bool I2C_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite)
{
  	if(!I2C_Start())return FALSE;
    I2C_SendByte(WriteAddr);                         //������ַ 
    if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}	 //�ȴ�Ӧ��
		while(NumByteToWrite--)
		{
		  I2C_SendByte(* pBuffer);
		  I2C_WaitAck();
          pBuffer++;
		}
	  I2C_Stop(); 	
	  return TRUE;
}

/****************************************************************************
* ��    �ƣ�void I2C_delay(void)
* ��    �ܣ�I2C ������ʱ����
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
void I2C_delay(void)
{
/*	
   u8 i=100*10; 
   while(i) 
   { 
     i--; 
   } 
   */
   Delay1ms(1);
}
/****************************************************************************
* ��    �ƣ�bool I2C_Start(void)
* ��    �ܣ�I2C��ʼ״̬
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
bool I2C_Start(void)
{
	TEA_SDA_OUT
	SDA_H;						//SDA�ø�
	
	TEA_SCL_OUT
	SCL_H;						//SCL�ø�
	
	I2C_delay();
	
	TEA_SDA_IN
	if(!SDA_read)return FALSE;	//SDA��Ϊ�͵�ƽ������æ,�˳�
	
	TEA_SDA_OUT
	SDA_L;
	
	I2C_delay();
	TEA_SDA_IN
	if(SDA_read) return FALSE;	//SDA��Ϊ�ߵ�ƽ�����߳���,�˳�
	
	TEA_SDA_OUT
	SDA_L;						//SDA�õ�
	I2C_delay();
	return TRUE;
}
/****************************************************************************
* ��    �ƣ�void I2C_Stop(void)
* ��    �ܣ�I2Cֹͣ״̬
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
void I2C_Stop(void)
{
	TEA_SCL_OUT
	SCL_L;				  
	I2C_delay();
	TEA_SDA_OUT
	SDA_L;
	I2C_delay();
	TEA_SCL_OUT
	SCL_H;
	I2C_delay();
	TEA_SDA_OUT
	SDA_H;
	I2C_delay();
}
/****************************************************************************
* ��    �ƣ�void I2C_Ack(void)
* ��    �ܣ�I2C ACKӦ��
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
void I2C_Ack(void)
{	
	TEA_SCL_OUT
	SCL_L;
	I2C_delay();
	
	TEA_SDA_OUT
	SDA_L;
	I2C_delay();
	
	TEA_SCL_OUT
	SCL_H;
	I2C_delay();
	
	TEA_SCL_OUT
	SCL_L;
	I2C_delay();
}
/****************************************************************************
* ��    �ƣ�void I2C_NoAck(void)
* ��    �ܣ�I2C ��Ӧ��
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
void I2C_NoAck(void)
{	
	TEA_SCL_OUT
	SCL_L;
	I2C_delay();
	
	TEA_SDA_OUT
	SDA_H;
	I2C_delay();
	
	TEA_SCL_OUT
	SCL_H;
	I2C_delay();
	
	TEA_SCL_OUT
	SCL_L;
	I2C_delay();
}
/****************************************************************************
* ��    �ƣ�bool I2C_WaitAck(void)
* ��    �ܣ�I2C�ȴ�Ӧ��
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
bool I2C_WaitAck(void) 	 //����Ϊ:=1��ACK,=0��ACK
{
	TEA_SCL_OUT
	SCL_L;
	I2C_delay();
	
	TEA_SDA_OUT
	SDA_H;			
	I2C_delay();
	
	TEA_SCL_OUT
	SCL_H;
	I2C_delay();
	
	TEA_SDA_IN
	if(SDA_read)
	{
	  TEA_SCL_OUT
      SCL_L;
      return FALSE;
	}
	TEA_SCL_OUT
	SCL_L;
	return TRUE;
}
/****************************************************************************
* ��    �ƣ�void I2C_SendByte(u8 SendByte)
* ��    �ܣ�
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
void I2C_SendByte(u8 SendByte) //���ݴӸ�λ����λ//
{
    u8 i=8;
    while(i--)
    {
		TEA_SCL_OUT
        SCL_L;
        I2C_delay();
      if(SendByte&0x80)
	  {
		  TEA_SDA_OUT
        SDA_H;  
	  }
      else 
		 {
		  TEA_SDA_OUT
        SDA_L; 
		 }			 
        SendByte<<=1;
        I2C_delay();
		 TEA_SCL_OUT
		SCL_H;
        I2C_delay();
    }
	TEA_SCL_OUT
    SCL_L;
}
/****************************************************************************
* ��    �ƣ�u8 I2C_ReceiveByte(void)
* ��    �ܣ�I2C�����ֽ�
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
u8 I2C_ReceiveByte(void)  //���ݴӸ�λ����λ//
{ 
    u8 i=8;
    u8 ReceiveByte=0;
	TEA_SDA_OUT
    SDA_H;				
    while(i--)
    {
      ReceiveByte<<=1;   
		TEA_SCL_OUT		
      SCL_L;
      I2C_delay();
		TEA_SCL_OUT
	  SCL_H;
      I2C_delay();	
		
		TEA_SDA_IN
      if(SDA_read)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L;
    return ReceiveByte;
}

/****************************************************************************
* ��    �ƣ�bool I2C_ReadByte(u8* pBuffer,   u8 length,   u8 DeviceAddress)
* ��    �ܣ�I2C ��
* ��ڲ�����u8* pBuffer-- ����     u8 length--�������ֽ���  u8 DeviceAddress--������ַ
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/	
bool I2C_ReadByte(u8* pBuffer,   u8 length,   u8 DeviceAddress)
{		
    if(!I2C_Start())return FALSE;
    I2C_SendByte(DeviceAddress);                            //������ַ 
    if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}
	    
		while(length--)
		{
		  *pBuffer = I2C_ReceiveByte();
     	  if(length == 1)I2C_NoAck();
     	  else I2C_Ack(); 
          pBuffer++;
        
		}
	  I2C_Stop(); 	
	  return TRUE;
}


/****************************************************************************
* ��    �ƣ�void SetPLL(void)
* ��    �ܣ�tea5767����FMƵ��
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
void SetPLL(void)
{	     
   FM_PLL=(unsigned long)((4000*(FM_FREQ/1000+225))/32768); 	         //����PLLֵ
   if(rec_f==2) 
	   PLL_HIGH=(unsigned char)(((FM_PLL >> 8)&0X3f)|0xc0);	 //PLL���ֽ�ֵ--����ģʽ����
   else 
	   PLL_HIGH=(unsigned char)((FM_PLL >> 8)&0X3f);	                 //PLL���ֽ�ֵ
   Tx1_Buffer[0]=PLL_HIGH;		 						                 //I2C��һ�ֽ�ֵ
   PLL_LOW=(unsigned char)FM_PLL;			      		                 //PLL���ֽ�ֵ
   Tx1_Buffer[1]= PLL_LOW;						                         //I2C�ڶ��ֽ�ֵ
   I2C_Write(Tx1_Buffer, Tea5767_WriteAddress1, 5); 					 //д��tea767
}
int main_fm(unsigned char rec_f)
{ float a=0;
  unsigned long fm_ch[50];
  unsigned char ch1=0;
	uint8_t TxBuffer1[] = "�ܶ���STM32������TEA5767����ʾDEMO";
  //RCC_Configuration();
  //NVIC_Configuration();
  FM_Configuration();  
  //Usart1_Init();  	  
  printf("\r\n �ܶ���STM32������TEA5767����ʾDEMO \n");
  printf("\r\n");
  printf("\r\n");
  printf("\r\n H(h)---����                    S(s)---������Ŀ  \n");
  printf("\r\n D(d)---��ʾ��Ч��Ŀ            xxP(xxp)---����ѡ���Ľ�Ŀ(��12P) \n");
  printf("\r\n xx.xM(xx.xm)---ֱ��ѡ��Ƶ��(��98.8M)      \n");
  printf("\r\n");
  printf("\r\n");
  I2C_Write(Tx1_Buffer, Tea5767_WriteAddress1, 5);   	//����TEA5767Ĭ�ϵ�5���ֽ�����
  SetPLL();												//���ó�ʼĬ��Ƶ��
  
 // while (1)
  {	
  	if(rec_f!=0){
		
		if(rec_f==1){                   //ֱ������Ƶ��
			a=atof(TxBuffer1);
			FM_FREQ=a*1000000;
			SetPLL();
			//printf(&TxBuffer1[0],len+2);
			printf("\r ��ǰFMƵ����:   %g\n MHz \n",a); 
		}
		else if(rec_f==2){                   //����
			//printf("Search......",15+2);
			printf("\n ����FM��Ŀ! \n");
			Tx1_Buffer[0] = 0XF0; 
			//I2C_Write(Tx1_Buffer, Tea5767_WriteAddress1, 5); 
			//Delay(0xffffff);
			FM_FREQ=87500000;
			FM_FREQ=FM_FREQ+100000;				
		  	SetPLL(); 
			Delay(0x6fffff);
			I2C_ReadByte(Rx1_Buffer,5,Tea5767_ReadAddress1);
			ch2=0;
		 	while(1){
		    //IF<51  IF>55  LEVEL<9
fm_pub:
				FM_FREQ=FM_FREQ+100000;			
		  		if(FM_FREQ>108000000)		
				{
					FM_FREQ=105300000; 
					break;
				}  
				SetPLL(); 
				Delay(0x0fffff);
				I2C_ReadByte(Rx1_Buffer,5,Tea5767_ReadAddress1);				
				a=FM_FREQ;
				a=a/1000000;				
				if((Rx1_Buffer[0]&0x3f)!=(Tx1_Buffer[0]&0x3f)||(Rx1_Buffer[1]!=Tx1_Buffer[1])||(Rx1_Buffer[1]&0x80!=0x80)||Rx1_Buffer[2]<50||Rx1_Buffer[2]>=56||(Rx1_Buffer[3]>>4)<7||(Rx1_Buffer[3]>>4)>14){
				    
					printf("\n ��ǰFMƵ����:   %g MHz \r\n",a); 
								
				}
				else {					
					
					printf("\n ��ǰFMƵ����:   %g MHz     ���ź�!!!    %u  %u\r\n",a, Rx1_Buffer[2],Rx1_Buffer[3]>>4);
					fm_ch[ch2]= FM_FREQ; 					
					ch2++;
					goto fm_pub;
				}
				
				
			}
			if(FM_FREQ!=105300000) goto fm_pub;
			printf("\r\n ��Ч��FMƵ���ܹ���:   %u �� \n",ch2); 
			ch1=ch2;
			while(ch1--){
				a=fm_ch[ch1];
				a=a/1000000;	
				printf("\r\n %u  FMƵ��: %g  MHz \n",ch1,a); 
			}				
		}
		else if(rec_f==3){                   //��ʾ��ЧƵ��
			printf("\r\n ��Ч��FMƵ���ܹ���:   %u �� \n",ch2); 
			ch1=ch2;
			while(ch1--){
				a=fm_ch[ch1];
				a=a/1000000;	
				printf("\r\n %u  FMƵ��: %g  MHz \n",ch1,a); 
			}				
		}
		else if(rec_f==4){                   //��ʾ��ЧƵ��
			ch1=atoi(TxBuffer1);
			FM_FREQ=fm_ch[ch1];
			a=fm_ch[ch1];
			a=a/1000000;
			printf("\r\n ��ǰFMƵ����:   %g\n MHz \n",a); 
		}
	    else if(rec_f==5){                   //��ʾ��ЧƵ��
			printf("\r\n �ܶ���STM32������TEA5767����ʾDEMO \n");
  			printf("\r\n");
  			printf("\r\n");
  			printf("\r\n H(h)---����                    S(s)---������Ŀ  \n");
  			printf("\r\n D(d)---��ʾ��Ч��Ŀ            xxP(xxp)---����ѡ���Ľ�Ŀ(��12P) \n");
  			printf("\r\n xx.xM(xx.xm)---ֱ��ѡ��Ƶ��(��98.8M)      \n");
  			printf("\r\n");
  			printf("\r\n");
		}
	
		rec_f=0;
		SetPLL();
	}
  }
  return 0;
}

