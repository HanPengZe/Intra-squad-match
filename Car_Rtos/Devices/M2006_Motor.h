#ifndef __M2006_MOTOR_H
#define __M2006_MOTOR_H
#include <stdint.h>
#include "BSP_CAN.h"

#define M2006_READID_START 0x207
#define M2006_READID_END 0x207
#define M2006_SENDID 0x1FF    //����5-8�ĵ��ID
#define M2006_MaxOutput 10000 //���͸������������ֵ
#define M2006_LOADANGLE 36864
#define M2006_ReductionRatio 36 //������ٱ�


typedef struct
{
    uint16_t realAngle; //�������Ļ�е�Ƕ�
    int16_t realSpeed;  //���������ٶ�
    int16_t realTorque; //��������ʵ��ת��

    int16_t targetSpeed; //Ŀ���ٶ�
    int32_t targetAngle; //Ŀ��Ƕ�

    uint16_t lastAngle; //�ϴεĽǶ�
    int32_t totalAngle; //�ۻ��ܹ��Ƕ�
    int16_t turnCount;  //ת����Ȧ��

    int16_t outCurrent;      //�������
    int16_t inneroutCurrent; //�������

    uint8_t InfoUpdateFlag;   //��Ϣ��ȡ���±�־
    uint16_t InfoUpdateFrame; //֡��
    uint8_t OffLineFlag;      //�豸���߱�־
} M2006s_t;

extern M2006s_t M2006_Reload; //����������


















#endif

