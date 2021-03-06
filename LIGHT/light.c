					   //***************************************
// BH1750FVI IIC测试程序
// 使用单片机STC89C51
// 晶振：11.0592M
// 显示：LCD1602
// 编译环境 Keil uVision2
// 参考宏晶网站24c04通信程序
// 时间：2011年4月20日
//****************************************
#include  <REG52.H>							  //改:#include <reg51.h>
#include  <math.h>    //Keil library
#include  <stdio.h>   //Keil library
#include  <INTRINS.H>
#define   uchar unsigned char
#define   uint unsigned int
#define   DataPort P0	 //LCD1602数据端口
sbit	  SCL=P3^7;      //IIC时钟引脚定义				   //改：sbit SCL = P1^0;
sbit  	  SDA=P3^6;      //IIC数据引脚定义				   //改：sibt SDA = P1^1;
sbit      LCM_RS=P1^0;   //LCD1602命令端口				   //改：sbit LCM_RS = P2^0;
sbit      LCM_RW=P1^1;   //LCD1602命令端口				   //改：sbit LCM_RW = P2^1;
sbit      LCM_EN=P1^5;   //LCD1602命令端口 */			   //改：sbit LCM_EN=P2^2;
													   
#define	  SlaveAddress   0x46 //定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改
                              //ALT  ADDRESS引脚接地时地址为0x46，接电源时地址为0xB8
typedef   unsigned char BYTE;
typedef   unsigned short WORD;

BYTE    BUF[8];                         //接收数据缓存区
uchar   ge,shi,bai,qian,wan;            //显示变量
int     dis_data;                       //变量

unsigned long beats = 0;                //加

void delay(unsigned int cnt);			//加
void TrunMotor(unsigned long angle);	//加

void delay_nms(unsigned int k);
void InitLcd();
void Init_BH1750(void);

void WriteDataLCM(uchar dataW);
void WriteCommandLCM(uchar CMD,uchar Attribc);
void DisplayOneChar(uchar X,uchar Y,uchar DData);
void conversion(uint temp_data);

void  Single_Write_BH1750(uchar REG_Address);               //单个写入数据
uchar Single_Read_BH1750(uchar REG_Address);                //单个读取内部寄存器数据
void  Multiple_Read_BH1750();                               //连续的读取内部寄存器数据
//------------------------------------
void Delay5us();
void Delay5ms();
void BH1750_Start();                    //起始信号
void BH1750_Stop();                     //停止信号
void BH1750_SendACK(bit ack);           //应答ACK
bit  BH1750_RecvACK();                  //读ack
void BH1750_SendByte(BYTE dat);         //IIC单个字节写
BYTE BH1750_RecvByte();                 //IIC单个字节读

//-----------------------------------

//*********************************************************
void conversion(uint temp_data)  //  数据转换出 个，十，百，千，万
{
    wan=temp_data/10000+0x30 ;
    temp_data=temp_data%10000;   //取余运算
	qian=temp_data/1000+0x30 ;
    temp_data=temp_data%1000;    //取余运算
    bai=temp_data/100+0x30   ;
    temp_data=temp_data%100;     //取余运算
    shi=temp_data/10+0x30    ;
    temp_data=temp_data%10;      //取余运算
    ge=temp_data+0x30;
}

//毫秒延时**************************
void delay_nms(unsigned int k)
{
unsigned int i,j;
for(i=0;i<k;i++)
{
for(j=0;j<121;j++)
{;}}
}

/*******************************/
void timer()
{
       //配置T0工作在模式1，定时2ms

    TMOD = 0x01;

    TH0 = 0xF8;

     TL0 = 0xCD;

     TR0 = 1;

    ET0 = 1;

     EA = 1;
}

/*******************************/
void TrunMotor(unsigned long angle)
{
   	 unsigned char buf;

    unsigned char step = 0;

    unsigned long beats = 0;

    unsigned char code BeatCode[8] = {  //步进电机节拍对应到IO控制电平的代码

        0xE, 0xC, 0xD, 0x9, 0xB, 0x3, 0x7, 0x6

    };

    

    beats = (angle * 4096) / 360; //计算需要的节拍数，4096拍对应一圈

    while (beats--)

    {

        buf  = P1 & 0xF0;       //用buf暂存P1口的高4位，而低4位清零

        buf |= BeatCode[step];  //buf低4位改为相应的节拍代码值

        P1   = buf;             //修改后完毕后的值送回到P1口

        step++;        //步进节拍递增

        step &= 0x07;  //用“与”方式实现到8归零

        delay(200);    //延时2ms，即2ms执行一拍

    }

    P1 |= 0x0F; //关闭电机所有的相
	
}
/********************************/
 void StopMotor()							//加  电机停止函数，这个我后边还没用到
	{ 
    	beats = 0;
	}
/********************************/

 void delay(unsigned int cnt)			//加 电机延时函数

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
LCM_RS=0;LCM_RW=0;_nop_();_nop_();	
LCM_EN=1;_nop_();_nop_();LCM_EN=0;
}
/*******************************/

void WriteDataLCM(unsigned char CMD)
{					
WaitForEnable();
LCM_EN=0;
DataPort=CMD;	
LCM_RS=1;LCM_RW=0;_nop_();_nop_();	
LCM_EN=1;_nop_();_nop_();LCM_EN=0;
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
延时5微秒(STC90C52RC@12M)
不同的工作环境,需要调整此函数，注意时钟过快时需要修改
当改用1T的MCU时,请调整此延时函数
**************************************/
void Delay5us()
{
    _nop_();_nop_();_nop_();_nop_();
    _nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();
}

/**************************************
延时5毫秒(STC90C52RC@12M)
不同的工作环境,需要调整此函数
当改用1T的MCU时,请调整此延时函数
**************************************/
void Delay5ms()
{
    WORD n = 560;

    while (n--);
}

/**************************************
起始信号
**************************************/
void BH1750_Start()
{
    SDA = 1;                    //拉高数据线
    SCL = 1;                    //拉高时钟线
    Delay5us();                 //延时
    SDA = 0;                    //产生下降沿
    Delay5us();                 //延时
    SCL = 0;                    //拉低时钟线
}

/**************************************
停止信号
**************************************/
void BH1750_Stop()
{
    SDA = 0;                    //拉低数据线
    SCL = 1;                    //拉高时钟线
    Delay5us();                 //延时
    SDA = 1;                    //产生上升沿
    Delay5us();                 //延时
}

/**************************************
发送应答信号
入口参数:ack (0:ACK 1:NAK)
**************************************/
void BH1750_SendACK(bit ack)
{
    SDA = ack;                  //写应答信号
    SCL = 1;                    //拉高时钟线
    Delay5us();                 //延时
    SCL = 0;                    //拉低时钟线
    Delay5us();                 //延时
}

/**************************************
接收应答信号
**************************************/
bit BH1750_RecvACK()
{
    SCL = 1;                    //拉高时钟线
    Delay5us();                 //延时
    CY = SDA;                   //读应答信号
    SCL = 0;                    //拉低时钟线
    Delay5us();                 //延时

    return CY;
}

/**************************************
向IIC总线发送一个字节数据
**************************************/
void BH1750_SendByte(BYTE dat)
{
    BYTE i;

    for (i=0; i<8; i++)         //8位计数器
    {
        dat <<= 1;              //移出数据的最高位
        SDA = CY;               //送数据口
        SCL = 1;                //拉高时钟线
        Delay5us();             //延时
        SCL = 0;                //拉低时钟线
        Delay5us();             //延时
    }
    BH1750_RecvACK();
}

/**************************************
从IIC总线接收一个字节数据
**************************************/
BYTE BH1750_RecvByte()
{
    BYTE i;
    BYTE dat = 0;

    SDA = 1;                    //使能内部上拉,准备读取数据,
    for (i=0; i<8; i++)         //8位计数器
    {
        dat <<= 1;
        SCL = 1;                //拉高时钟线
        Delay5us();             //延时
        dat |= SDA;             //读数据
        SCL = 0;                //拉低时钟线
        Delay5us();             //延时
    }
    return dat;
}

//*********************************

void Single_Write_BH1750(uchar REG_Address)
{
    BH1750_Start();                  //起始信号
    BH1750_SendByte(SlaveAddress);   //发送设备地址+写信号
    BH1750_SendByte(REG_Address);    //内部寄存器地址，
  //  BH1750_SendByte(REG_data);       //内部寄存器数据，
    BH1750_Stop();                   //发送停止信号
}

//********单字节读取*****************************************
/*
uchar Single_Read_BH1750(uchar REG_Address)
{  uchar REG_data;
    BH1750_Start();                          //起始信号
    BH1750_SendByte(SlaveAddress);           //发送设备地址+写信号
    BH1750_SendByte(REG_Address);                   //发送存储单元地址，从0开始
    BH1750_Start();                          //起始信号
    BH1750_SendByte(SlaveAddress+1);         //发送设备地址+读信号
    REG_data=BH1750_RecvByte();              //读出寄存器数据
	BH1750_SendACK(1);
	BH1750_Stop();                           //停止信号
    return REG_data;
}
*/
//*********************************************************
//
//连续读出BH1750内部数据
//
//*********************************************************
void Multiple_read_BH1750(void)
{   uchar i;
    BH1750_Start();                          //起始信号
    BH1750_SendByte(SlaveAddress+1);         //发送设备地址+读信号

	 for (i=0; i<3; i++)                      //连续读取2个地址数据，存储中BUF
    {
        BUF[i] = BH1750_RecvByte();          //BUF[0]存储0x32地址中的数据
        if (i == 3)
        {

           BH1750_SendACK(1);                //最后一个数据需要回NOACK
        }
        else
        {
          BH1750_SendACK(0);                //回应ACK
       }
   }

    BH1750_Stop();                          //停止信号
    Delay5ms();
}


//初始化BH1750，根据需要请参考pdf进行修改****
void Init_BH1750()
{
   Single_Write_BH1750(0x01);

}
//*********************************************************
//主程序********
//*********************************************************

/*void main()
{
	InitLcd();           //初始化LCD
	WriteCommandLCM(0x80,0);
	WriteDataLCM('S');
	
	while (1);
} */
void main()
{
   float temp;
   delay_nms(100);	    //延时100ms
   InitLcd();           //初始化LCD
   Init_BH1750();       //初始化BH1750

  while(1)              //循环
  {

    Single_Write_BH1750(0x01);   // power on
    Single_Write_BH1750(0x10);   // H- resolution mode

     delay_nms(180);              //延时180ms

    Multiple_Read_BH1750();       //连续读出数据，存储在BUF中

    dis_data=BUF[0];
    dis_data=(dis_data<<8)+BUF[1];//合成数据，即光照数据

    temp=(float)dis_data/1.2;

    conversion(temp);         //计算数据和显示
	DisplayOneChar(0,0,'L');
	DisplayOneChar(1,0,'i');
	DisplayOneChar(2,0,'g');
	DisplayOneChar(3,0,'h');
	DisplayOneChar(4,0,'t');
    DisplayOneChar(5,0,':');

    DisplayOneChar(7,0,wan); //显示数据
    DisplayOneChar(8,0,qian);
    DisplayOneChar(9,0,bai);
    DisplayOneChar(10,0,shi);
	DisplayOneChar(11,0,ge);

	DisplayOneChar(13,0,'l'); ////显示数单位
	DisplayOneChar(14,0,'x');

	if(dis_data>50)					  //加
		 TrunMotor(90); 
	

  }
}
		  
