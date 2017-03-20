		#include <reg52.h>

 

void delay(unsigned int cnt);

void TrunMotor(unsigned long angle);

 

void main()

{

    TrunMotor(360*25); //360��*25����25Ȧ

     while(1);

}

 

void TrunMotor(unsigned long angle)

{

    unsigned char buf;

    unsigned char step = 0;

    unsigned long beats = 0;

    unsigned char code BeatCode[8] = {  //����������Ķ�Ӧ��IO���Ƶ�ƽ�Ĵ���

        0xE, 0xC, 0xD, 0x9, 0xB, 0x3, 0x7, 0x6

    };

    

    beats = (angle * 4096) / 360; //������Ҫ�Ľ�������4096�Ķ�ӦһȦ

    while (beats--)

    {

        buf  = P1 & 0xF0;       //��buf�ݴ�P1�ڵĸ�4λ������4λ����

        buf |= BeatCode[step];  //buf��4λ��Ϊ��Ӧ�Ľ��Ĵ���ֵ

        P1   = buf;             //�޸ĺ���Ϻ��ֵ�ͻص�P1��

        step++;        //�������ĵ���

        step &= 0x07;  //�á��롱��ʽʵ�ֵ�8����

        delay(200);    //��ʱ2ms����2msִ��һ��

    }

    P1 |= 0x0F; //�رյ�����е���

}

void delay(unsigned int cnt)

{

     while (cnt--);

}
