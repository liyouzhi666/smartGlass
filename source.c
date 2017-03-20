//***************************************
// BH1750FVI IIC���Գ���
// ʹ�õ�Ƭ��STC89C51
// ����11.0592M
// ��ʾ��LCD1602
// ���뻷�� Keil uVision2
// �ο��꾧��վ24c04ͨ�ų���
// ʱ�䣺2011��4��20��
//****************************************
#include  <REG52.H>							  //��:#include <reg51.h>
#include  <math.h>    //Keil library
#include  <stdio.h>   //Keil library
#include  <INTRINS.H>
#define   uchar unsigned char
#define   uint unsigned int
#define   DataPort P0	 //LCD1602���ݶ˿�
sbit	  SCL=P3^7;      //IICʱ�����Ŷ���				   //�ģ�sbit SCL = P1^0;
sbit  	  SDA=P3^6;      //IIC�������Ŷ���				   //�ģ�sibt SDA = P1^1;
sbit      LCM_RS=P1^0;   //LCD1602����˿�				   //�ģ�sbit LCM_RS = P2^0;
sbit      LCM_RW=P1^1;   //LCD1602����˿�				   //�ģ�sbit LCM_RW = P2^1;
sbit      LCM_EN=P1^5;   //LCD1602����˿� */			   //�ģ�sbit LCM_EN=P2^2;
													   
#define	  SlaveAddress   0x46 //����������IIC�����еĴӵ�ַ,����ALT  ADDRESS��ַ���Ų�ͬ�޸�
                              //ALT  ADDRESS���Žӵ�ʱ��ַΪ0x46���ӵ�Դʱ��ַΪ0xB8
typedef   unsigned char BYTE;
typedef   unsigned short WORD;

BYTE    BUF[8];                         //�������ݻ�����
uchar   ge,shi,bai,qian,wan;            //��ʾ����
int     dis_data,triangle;                       //����	//�Ҽӵ�dis_data1��triangle

unsigned long beats = 0;                //��

void delay(unsigned int cnt);			//��
void TrunMotor(unsigned long angle);	//��

void delay_nms(unsigned int k);
void InitLcd();
void Init_BH1750(void);

void WriteDataLCM(uchar dataW);
void WriteCommandLCM(uchar CMD,uchar Attribc);
void DisplayOneChar(uchar X,uchar Y,uchar DData);
void conversion(uint temp_data);

void  Single_Write_BH1750(uchar REG_Address);               //����д������
uchar Single_Read_BH1750(uchar REG_Address);                //������ȡ�ڲ��Ĵ�������
void  Multiple_Read_BH1750();                               //�����Ķ�ȡ�ڲ��Ĵ�������
//------------------------------------
void Delay5us();
void Delay5ms();
void BH1750_Start();                    //��ʼ�ź�
void BH1750_Stop();                     //ֹͣ�ź�
void BH1750_SendACK(bit ack);           //Ӧ��ACK
bit  BH1750_RecvACK();                  //��ack
void BH1750_SendByte(BYTE dat);         //IIC�����ֽ�д
BYTE BH1750_RecvByte();                 //IIC�����ֽڶ�

//-----------------------------------
void initSer();          //���ڳ�ʼ��  
uchar  sendFlag =0;     //δ��������ʱ  
uchar  receFlag =0;     //δ���ܵ�����ʱ  
uchar  TEXT0[] = "00000\r\n";  //\r\n�ǻس�����   
uchar *pValue=TEXT0; 
void count(int display_light);
void sendChar(uchar sendValue);  //����һ�ֽ�����  
void sendAll(uchar *pValue);       //����һ������  
//*********************************************************
void conversion(uint temp_data)  //  ����ת���� ����ʮ���٣�ǧ����
{
    wan=temp_data/10000+0x30 ;
    temp_data=temp_data%10000;   //ȡ������
    qian=temp_data/1000+0x30 ;
    temp_data=temp_data%1000;    //ȡ������
    bai=temp_data/100+0x30   ;
    temp_data=temp_data%100;     //ȡ������
    shi=temp_data/10+0x30    ;
    temp_data=temp_data%10;      //ȡ������
    ge=temp_data+0x30;
}

//������ʱ**************************
void delay_nms(unsigned int k)
{
	unsigned int i,j;
	for(i=0;i<k;i++)
	{
		for(j=0;j<121;j++){;}
	}
}

/*******************************/
void timer()
{
	TMOD=0x21;
	TH0=(65536-50000)/256;
	TL0=(65536-50000)%256;
	TH1=0Xfd;        // 256-(11059200/(32*12*9600))  
	TL1=0xfd;  
	EA=1;
	ET0=1;
	TR0=1;
	SM0=0;           //����SCON�Ĵ���  
	SM1=1;           //���ڹ�����ʽ1,10λ�첽,�����ʿɸ�  
	REN=1;             //�����ڽ���  
	ES=1;             //�������ж�  
	TR1=1;       //������ʱ��  	
}
/*******************************/
/**************���˳ʱ����ת*****************/
void TrunMotor(unsigned long angle)
{
	unsigned char buf;

    unsigned char step = 0;

    unsigned long beats = 0;

    unsigned char code BeatCode[8] = {  //����������Ķ�Ӧ��IO���Ƶ�ƽ�Ĵ���

        0xE, 0xC, 0xD, 0x9, 0xB, 0x3, 0x7, 0x6

    };// ˳ʱ��ת
    
    beats = (angle * 512) / 360; //������Ҫ�Ľ�������4096�Ķ�ӦһȦ		   

    while (beats--)

    {

        buf  = P1 & 0xF0;       //��buf�ݴ�P1�ڵĸ�4λ������4λ����

        buf |= BeatCode[step];  //buf��4λ��Ϊ��Ӧ�Ľ��Ĵ���ֵ

        P1   = buf;             //�޸ĺ���Ϻ��ֵ�ͻص�P1��

        step++;        //�������ĵ���

        step &= 0x07;  //�á��롱��ʽʵ�ֵ�8����

        delay(100);    //��ʱ2ms����2msִ��һ��

    }

    P1 |= 0x0F; //�رյ�����е���
	
}
/********************************************/
/************�����ʱ����ת******************/
void TrunMotor1(unsigned long angle)
{
   	unsigned char buf;

    unsigned char step = 0;

    unsigned long beats = 0;

	   unsigned char code BeatCode[8] = {  //����������Ķ�Ӧ��IO���Ƶ�ƽ�Ĵ���

        0x6, 0x7, 0x3, 0xB, 0x9, 0xD, 0xC, 0xE

    };
    

    beats = (angle * 512) / 360; //������Ҫ�Ľ�������4096�Ķ�ӦһȦ

    while (beats--)

    {

        buf  = P1 & 0xF0;       //��buf�ݴ�P1�ڵĸ�4λ������4λ����

        buf |= BeatCode[step];  //buf��4λ��Ϊ��Ӧ�Ľ��Ĵ���ֵ

        P1   = buf;             //�޸ĺ���Ϻ��ֵ�ͻص�P1��

        step++;        //�������ĵ���

        step &= 0x07;  //�á��롱��ʽʵ�ֵ�8����

        delay(100);    //��ʱ2ms����2msִ��һ��

    }

    P1 |= 0x0F; //�رյ�����е���
	
}
/********************************/
 void StopMotor()							//��  ���ֹͣ����
	{ 
    	beats = 0;
	}
/********************************/

 void delay(unsigned int cnt)			//�� �����ʱ����

{

     while (cnt--);

}

/********************************/

void WaitForEnable(void)	
{					
	P0=0xff;		
	do
	{
		LCM_RS=0;
		LCM_RW=1;_nop_();
		LCM_EN=0;_nop_();_nop_();
		LCM_EN=1;
	}while(DataPort&0x80);	
	LCM_EN=0;				
}
/*******************************/

void WriteCommandLCM(uchar CMD, uchar Attribc)
{
	if(Attribc)WaitForEnable();
	LCM_EN=0;
	DataPort=CMD;	
	LCM_RS=0;
	LCM_RW=0;
	_nop_();
	_nop_();	
	LCM_EN=1;
	_nop_();
	_nop_();
	LCM_EN=0;
}
/*******************************/

void WriteDataLCM(unsigned char CMD)
{					
	WaitForEnable();
	LCM_EN=0;
	DataPort=CMD;	
	LCM_RS=1;
	LCM_RW=0;
	_nop_();
	_nop_();	
	LCM_EN=1;
	_nop_();
	_nop_();
	LCM_EN=0;
}
/***********************************/
void InitLcd()
{
	WriteCommandLCM(0x38,1);
	WriteCommandLCM(0x08,1);
	WriteCommandLCM(0x01,1);
	WriteCommandLCM(0x06,1);
	WriteCommandLCM(0x0c,1);
}
/***********************************/
void DisplayOneChar(uchar X,uchar Y,uchar DData)
{
	Y&=1;
	X&=15;
	if(Y)X|=0x40;
	X|=0x80;
	WriteCommandLCM(X,0);
	WriteDataLCM(DData);
}

/**************************************
��ʱ5΢��(STC90C52RC@12M)
��ͬ�Ĺ�������,��Ҫ�����˺�����ע��ʱ�ӹ���ʱ��Ҫ�޸�
������1T��MCUʱ,���������ʱ����
**************************************/
void Delay5us()
{
 	_nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();
}

/**************************************
��ʱ5����(STC90C52RC@12M)
��ͬ�Ĺ�������,��Ҫ�����˺���
������1T��MCUʱ,���������ʱ����
**************************************/
void Delay5ms()
{
    WORD n = 560;

    while (n--);
}

/**************************************
��ʼ�ź�
**************************************/
void BH1750_Start()
{
    SDA = 1;                    //����������
    SCL = 1;                    //����ʱ����
    Delay5us();                 //��ʱ
    SDA = 0;                    //�����½���
    Delay5us();                 //��ʱ
    SCL = 0;                    //����ʱ����
}

/**************************************
ֹͣ�ź�
**************************************/
void BH1750_Stop()
{
    SDA = 0;                    //����������
    SCL = 1;                    //����ʱ����
    Delay5us();                 //��ʱ
    SDA = 1;                    //����������
    Delay5us();                 //��ʱ
}

/**************************************
����Ӧ���ź�
��ڲ���:ack (0:ACK 1:NAK)
**************************************/
void BH1750_SendACK(bit ack)
{
    SDA = ack;                  //дӦ���ź�
    SCL = 1;                    //����ʱ����
    Delay5us();                 //��ʱ
    SCL = 0;                    //����ʱ����
    Delay5us();                 //��ʱ
}

/**************************************
����Ӧ���ź�
**************************************/
bit BH1750_RecvACK()
{
    SCL = 1;                    //����ʱ����
    Delay5us();                 //��ʱ
    CY = SDA;                   //��Ӧ���ź�
    SCL = 0;                    //����ʱ����
    Delay5us();                 //��ʱ

    return CY;
}

/**************************************
��IIC���߷���һ���ֽ�����
**************************************/
void BH1750_SendByte(BYTE dat)
{
    BYTE i;

    for (i=0; i<8; i++)         //8λ������
    {
        dat <<= 1;              //�Ƴ����ݵ����λ
        SDA = CY;               //�����ݿ�
        SCL = 1;                //����ʱ����
        Delay5us();             //��ʱ
        SCL = 0;                //����ʱ����
        Delay5us();             //��ʱ
    }
    BH1750_RecvACK();
}

/**************************************
��IIC���߽���һ���ֽ�����
**************************************/
BYTE BH1750_RecvByte()
{
    BYTE i;
    BYTE dat = 0;

    SDA = 1;                    //ʹ���ڲ�����,׼����ȡ����,
    for (i=0; i<8; i++)         //8λ������
    {
        dat <<= 1;
        SCL = 1;                //����ʱ����
        Delay5us();             //��ʱ
        dat |= SDA;             //������
        SCL = 0;                //����ʱ����
        Delay5us();             //��ʱ
    }
    return dat;
}

//*********************************

void Single_Write_BH1750(uchar REG_Address)
{
    BH1750_Start();                  //��ʼ�ź�
    BH1750_SendByte(SlaveAddress);   //�����豸��ַ+д�ź�
    BH1750_SendByte(REG_Address);    //�ڲ��Ĵ�����ַ��
  //  BH1750_SendByte(REG_data);       //�ڲ��Ĵ������ݣ�
    BH1750_Stop();                   //����ֹͣ�ź�
}
//��������BH1750�ڲ�����
//*********************************************************
void Multiple_read_BH1750(void)
{   uchar i;
    BH1750_Start();                          //��ʼ�ź�
    BH1750_SendByte(SlaveAddress+1);         //�����豸��ַ+���ź�

	 for (i=0; i<3; i++)                      //������ȡ2����ַ���ݣ��洢��BUF
    {
        BUF[i] = BH1750_RecvByte();          //BUF[0]�洢0x32��ַ�е�����
        if (i == 3)
        {

           BH1750_SendACK(1);                //���һ��������Ҫ��NOACK
        }
        else
        {
          BH1750_SendACK(0);                //��ӦACK
       }
   }

    BH1750_Stop();                          //ֹͣ�ź�
    Delay5ms();
}


//��ʼ��BH1750��������Ҫ��ο�pdf�����޸�****
void Init_BH1750()
{
   Single_Write_BH1750(0x01);

}
//***********************����ͨѶ����**********************
void sendChar(uchar Value)  //����һ���ֽ�����  
{  
     SBUF = Value;       
     sendFlag = 1;       //���÷��ͱ�־λ,��һ�ֽھ���λ  
     while(sendFlag);    //ֱ����������,��sendFlag�����,���˳�sendChar����  
}  
  
void sendAll(uchar *pValue) //����һ������  
{  
    while((*pValue) != '\0')   //���û�з�����Ͼͼ�����  
    {  
        sendChar(*pValue);      //����1�ֽ�����  
        pValue++;                 //ָ����1���ֽ�  
    }  
} 
void count(int display_light)
{
	*pValue=display_light/10000+48;
	pValue++;
	*pValue=display_light/1000%10+48;
	pValue++;
	*pValue=display_light/100%10+48;
	pValue++;
	*pValue=display_light/10%10+48;
	pValue++;
	*pValue=display_light%10+48;
	pValue=pValue-4;	
}
	 
//******************************************************	 
//*********************************************************
//������********
//*********************************************************
void main()
{
	float temp;
	delay_nms(100);	    //��ʱ100ms
	InitLcd();           //��ʼ��LCD
	Init_BH1750();       //��ʼ��BH1750
	timer();
	while(1)              //ѭ��
	{

    Single_Write_BH1750(0x01);   // power on
    Single_Write_BH1750(0x10);   // H- resolution mode

    delay_nms(180);              //��ʱ180ms

    Multiple_Read_BH1750();       //�����������ݣ��洢��BUF��

    dis_data=BUF[0];
    dis_data=(dis_data<<8)+BUF[1];//�ϳ����ݣ�����������
 //*****************����********************** 
while(receFlag)               //��Ƭ�������յ��������ݺ�,��ʼ��PC��������  
{
	count(dis_data);
	sendAll(TEXT0);         //��������  
	receFlag=0;				 //���������־      
}  
 // **********************************
    temp=(float)dis_data/1.2;

    conversion(temp);         //�������ݺ���ʾ
	DisplayOneChar(0,0,'L');
	DisplayOneChar(1,0,'i');
	DisplayOneChar(2,0,'g');
	DisplayOneChar(3,0,'h');
	DisplayOneChar(4,0,'t');
    DisplayOneChar(5,0,':');

    DisplayOneChar(7,0,wan); //��ʾ����
    DisplayOneChar(8,0,qian);
    DisplayOneChar(9,0,bai);
    DisplayOneChar(10,0,shi);
	DisplayOneChar(11,0,ge);

	DisplayOneChar(13,0,'l'); ////��ʾ����λ
	DisplayOneChar(14,0,'x');

	if(triangle>0)					  //��
		 TrunMotor(90);
	else if(triangle<0)
		 TrunMotor1(90);

  }
}
void timer0() interrupt 1
{
	TH0=(65536-50000)/256;
	TL0=(65536-50000)%256;
	if(dis_data>200)
	triangle=1 ;
	else if(dis_data<50)
	triangle=-1;
	else triangle=0;
}
void timer1() interrupt 4   //�жϺ���  
{  
    if(RI)                  //����յ���������  
    {  
        RI = 0;   
        receFlag=1;   //�޸Ľ��ܱ�־,��������������while�з�����  
    } 
      
    if(TI)  
    {  
        TI = 0;                //������һ������  
        sendFlag = 0;        //���־λ  
    }  
}  
		  
