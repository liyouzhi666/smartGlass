		#include <reg52.h>

 

void delay(unsigned int cnt);

void TrunMotor(unsigned long angle);

 

void main()

{

    TrunMotor(360*25); //360度*25，即25圈

     while(1);

}

 

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

void delay(unsigned int cnt)

{

     while (cnt--);

}
