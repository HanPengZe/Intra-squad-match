#ifndef __M3508_MOTOR_H
#define __M3508_MOTOR_H

#include "BSP_CAN.h"
typedef struct
{
    uint16_t realAngle;  //�������Ļ�е�Ƕ�
    int16_t realSpeed;   //���������ٶ�
    int16_t realCurrent; //��������ʵ�ʵ���
    uint8_t temperture;  //�������ĵ���¶�

    int16_t targetSpeed;  //Ŀ���ٶ�
    uint16_t targetAngle; //Ŀ��Ƕ�
    uint16_t lastAngle;   //�ϴεĽǶ�
    int32_t totalAngle;   //�ۻ��ܹ��Ƕ�
    int16_t turnCount;    //ת����Ȧ��

    int16_t outCurrent; //�������

    uint8_t InfoUpdateFlag;   //��Ϣ��ȡ���±�־
    uint16_t InfoUpdateFrame; //֡��
    uint8_t OffLineFlag;      //�豸���߱�־
} M3508s_t;

#define M3508_READID_START 0x201
#define M3508_READID_END 0x204


void M3508_getInfo(Can_Export_Data_t RxMessage);
extern M3508s_t M3508s[4];
extern M3508s_t M3508_PowerL; //Ħ���ֵ�� 201
extern M3508s_t M3508_PowerR; //Ħ���ֵ��; 202



#endif
