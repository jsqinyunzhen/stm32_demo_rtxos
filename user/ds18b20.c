#include <stdio.h>
#include <string.h>
#include "ds18b20.h"
#include "stm32f10x.h"
uint8_t serial_1[8]= {0x28,0x2d,0x9a,0xdd,0x02,0x00,0x00,0x3b};
uint8_t serial_2[8]= {0x28,0x3b,0x2b,0xbc,0x02,0x00,0x00,0x4f};
uint8_t serial_3[8]= {0x28,0x00,0x49,0x1b,0x03,0x00,0x00,0x4c}; //���кţ���Ҫ�����Լ���DS18B20�޸ģ������ȡ������������һƪ���ܡ�

uint8_t serial_4[8]= {0x28,0xff,0xd9,0x4c,0x31,0x17,0x03,0x02};


#define Delay_Us Delay_us
static void DS18B20_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    //RCC_APB2PeriphClockCmd(DS18B20_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = DS18B20_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DS18B20_PORT, &GPIO_InitStructure);
    GPIO_SetBits(DS18B20_PORT, DS18B20_PIN);
}

static void DS18B20_Mode_IPU(void) //ʹDS18B20-DATA���ű�Ϊ����ģʽ
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = DS18B20_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(DS18B20_PORT, &GPIO_InitStructure);
}

static void DS18B20_Mode_Out_PP(void) //ʹDS18B20-DATA���ű�Ϊ���ģʽ
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = DS18B20_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DS18B20_PORT, &GPIO_InitStructure);
}

static void DS18B20_Rst(void)//�������ӻ����͸�λ����
{
    DS18B20_Mode_Out_PP();
    DS18B20_DATA_OUT(HIGH);//�����ڲ�����λ�źź��轫��������
    Delay_Us(2);
    DS18B20_DATA_OUT(LOW);
    Delay_Us(750);//�������ٲ���480us�ĵ͵�ƽ��λ�ź�
    DS18B20_DATA_OUT(HIGH);//�����ڲ�����λ�źź��轫��������
    //Delay_Us(15);//�ӻ����յ������ĸ�λ�źź󣬻���15~60us���������һ����������
    Delay_us(60);//15-60us
}

static uint8_t DS18B20_Presence(void) //���ӻ����������صĴ�������
{
    uint8_t pulse_time = 0;

    DS18B20_Mode_IPU(); //��������Ϊ��������

    //�ȴ���������ĵ�������������Ϊһ��60~240us�ĵ͵�ƽ�ź�?
    //�����������û����������ʱ�����ӻ����յ������ĸ�λ�źź󣬻���15~60us���������һ����������

    while( DS18B20_DATA_IN() && pulse_time<100 )
    {
        pulse_time++;
        Delay_Us(1);
    }
    if( pulse_time >=100 ) //����100us�󣬴������嶼��û�е���
        return 1;
    else
        pulse_time = 0;
    while( !DS18B20_DATA_IN() && pulse_time<240 ) //�������嵽�����Ҵ��ڵ�ʱ�䲻�ܳ���240us?
    {
        pulse_time++;
        Delay_Us(1);
    }
    if( pulse_time >=240 )
        return 1;
    else
        return 0;
}

static uint8_t DS18B20_Read_Bit(void) //��DS18B20��ȡһ��bit
{
    uint8_t dat;

    DS18B20_Mode_Out_PP(); //��0�Ͷ�1��ʱ������Ҫ����60us
    DS18B20_DATA_OUT(LOW); //��ʱ�����ʼ���������������� >1us <15us �ĵ͵�ƽ�ź�
    Delay_Us(10);
    DS18B20_Mode_IPU(); //���ó����룬�ͷ����ߣ����ⲿ�������轫��������
    if( DS18B20_DATA_IN() == SET )
        dat = 1;
    else
        dat = 0;
    Delay_Us(45); //�����ʱ������ο�ʱ��ͼ

    return dat;
}

uint8_t DS18B20_Read_Byte(void) //��DS18B20��һ���ֽڣ���λ����
{
    uint8_t i, j, dat = 0;

    for(i=0; i<8; i++)
    {
        j = DS18B20_Read_Bit();
        dat = (dat) | (j<<i);
    }

    return dat;
}

void DS18B20_Write_Byte(uint8_t dat) //дһ���ֽڵ�DS18B20����λ����
{
    uint8_t i, testb;

    DS18B20_Mode_Out_PP();
    for( i=0; i<8; i++ )
    {
        testb = dat&0x01;
        dat = dat>>1;
        if (testb) //д0��д1��ʱ������Ҫ����60us
        {
            DS18B20_DATA_OUT(LOW);
            Delay_Us(8);
            DS18B20_DATA_OUT(HIGH);
            Delay_Us(58);
        }
        else
        {
            DS18B20_DATA_OUT(LOW);
            Delay_Us(70);
            DS18B20_DATA_OUT(HIGH);
            Delay_Us(2);
        }
    }
}

void DS18B20_Start(void)
{
    DS18B20_Rst();
    DS18B20_Presence();
    DS18B20_Write_Byte(0XCC); //���� ROM ?
    DS18B20_Write_Byte(0X44); //��ʼת��?
}

uint8_t DS18B20_Init(void)
{
    DS18B20_GPIO_Config();
    DS18B20_Rst();

    return DS18B20_Presence();
}

void DS18B20_Match_Serial(uint8_t a)//ƥ�����к�
{
    uint8_t i;
    DS18B20_Rst();
    DS18B20_Presence();
    DS18B20_Write_Byte(0X55); //ƥ�����к�ָ��
    if(a==1)
    {
        for(i=0; i<8; i++)
            DS18B20_Write_Byte(serial_1[i]);
    }
    else if(a==2)
    {
        for(i=0; i<8; i++)
            DS18B20_Write_Byte(serial_2[i]);
    }
    else if(a==3)
    {
        for(i=0; i<8; i++)
            DS18B20_Write_Byte(serial_3[i]);
    }
    else if(a==4)
    {
        for(i=0; i<8; i++)
            DS18B20_Write_Byte(serial_4[i]);
    }

}


//�洢���¶���16 λ�Ĵ�������չ�Ķ����Ʋ�����ʽ
//��������12λ�ֱ���ʱ������5������λ��7������λ��4��С��λ

// ? ? ? ? |---------����----------|-----С�� �ֱ��� 1/(2^4)=0.0625----|
//���ֽ� ?| 2^3 | 2^2 | 2^1 | 2^0 | 2^(-1) | 2^(-2) | 2^(-3) | 2^(-4) |

// ? ? ? ?|-----����λ��0->�� ?1->��-------|-----------����-----------|
//���ֽ� ?| ?s ?| ?s ?| ?s ?| ?s ?| ? ?s ? | ? 2^6 ?| ? 2^5 ?| ? 2^4 ?|
//�¶� = ����λ + ���� + С��*0.0625


float DS18B20_Get_Temp(uint8_t *a,uint8_t b)
{
    uint8_t tpmsb, tplsb;
    signed short s_tem;
    float f_tem;
    int temp_num;

    DS18B20_Rst();

    tpmsb = DS18B20_Presence();
    printf("tpmsb=%d\r\n",tpmsb);
	if(tpmsb == 1)
		return -100;
	//if(b != 0)
	
    DS18B20_Write_Byte(0XCC); // ���� ROM?
    DS18B20_Match_Serial(b); //ƥ�����к�
    DS18B20_Write_Byte(0X44); // ��ʼת��?
	
    DS18B20_Rst();
    tpmsb =DS18B20_Presence();
    printf("tpmsb=%d\r\n",tpmsb);
	if(tpmsb == 1)
		return -100;
    DS18B20_Write_Byte(0XCC); //���� ROM ?
    DS18B20_Match_Serial(b); //ƥ�����к�
    DS18B20_Write_Byte(0XBE); //���¶�ֵ?

    tplsb = DS18B20_Read_Byte();
    tpmsb = DS18B20_Read_Byte();

    s_tem = tpmsb<<8;
    s_tem = s_tem | tplsb;

    if( s_tem < 0 ) //���¶�
    {
        f_tem = (~s_tem+1) * 0.0625;
        temp_num = (~s_tem+1) * 0.0625*10;
        //go_temp= temp_num;
        if(a)
        {
            if(temp_num>=1000)
            {
                a[0]='-';
                a[1]= temp_num/1000+'0';
                a[2]= temp_num%1000/100+'0';
                a[3]= temp_num%100/10+'0';
                a[4]='.';
                a[5]= temp_num%10+'0';
                a[6]= '\0';
            }
            else
            {
                a[0]='-';
                a[1]= temp_num/100+'0';
                a[2]= temp_num%100/10+'0';
                a[3]='.';
                a[4]=temp_num%10+'0';
                a[5]= '\0';
            }
        }
    }
    else
    {
        f_tem = s_tem * 0.0625;
        temp_num = s_tem * 0.0625*10;
        //go_temp= temp_num;
        if(a)
        {

            if(temp_num>=1000)
            {
                a[0]='+';
                a[1]= temp_num/1000+'0';
                a[2]= temp_num%1000/100+'0';
                a[3]= temp_num%100/10+'0';
                a[4]='.';
                a[5]= temp_num%10+'0';
                a[6]= '\0';
            }
            else
            {
                a[0]='+';
                a[1]= temp_num/100+'0';
                a[2]= temp_num%100/10+'0';
                a[3]='.';
                a[4]=temp_num%10+'0';
                a[5]= '\0';
            }
        }
    }
    return f_tem;
}

void read_serial(uint8_t *serial) //��ȡ���к�
{
    uint8_t i;
    DS18B20_Rst();
    DS18B20_Presence();
    DS18B20_Write_Byte(0X33); //��ȡ���к�ָ��
    for(i=0; i<8; i++)
        serial[i] = DS18B20_Read_Byte();
}


int main_ds18b20_test(void)
{
    //int go_num;

    //DS18B20_GPIO_Config();

    int i = 0;
    uint8_t temp_ds18b20[8]= {0};

    read_serial(temp_ds18b20);
    for(i=0; i<8; i++)
        printf("temp_ds18b20[%d]=0x%02x ",i,temp_ds18b20[i]);
    printf("\r\n");
    //printf("���DS18B20�¶ȶ�ȡʵ�飡\r\n");
    printf("DS18B20_Get_Temp\r\n");
/*

temp_ds18b20[0]=0x28
temp_ds18b20[1]=0xff
temp_ds18b20[2]=0x29
temp_ds18b20[3]=0x69
temp_ds18b20[4]=0x31
temp_ds18b20[5]=0x17
temp_ds18b20[6]=0x03
temp_ds18b20[7]=0x55
tpmsb=0


*/
    memset(temp_ds18b20,0,sizeof(temp_ds18b20));
    DS18B20_Get_Temp(temp_ds18b20,4);

    printf("%s\r\n",temp_ds18b20);

    printf("DS18B20_Get_Temp %f\r\n",DS18B20_Get_Temp(temp_ds18b20,4));
    //Delay_Ms(200);
    return 0;
}
