#ifndef __M6020_MOTOR_H
#define __M6020_MOTOR_H
#include "stdint.h"
#include "BSP_CAN.h"
#define M6020_READID_START 0x205 //��IDΪ1ʱ�ı���ID
#define M6020_READID_END 0x206
#define M6020_SENDID 0x1FF //1~4�ĵ����0x2FFΪ5~7

typedef struct
{
    uint16_t realAngle;  //�������Ļ�е�Ƕ�
    int16_t realSpeed;   //���������ٶ�
    int16_t realCurrent; //��������ʵ��ת�ص���
    uint8_t temperture;  //�������ĵ���¶�

    int16_t targetSpeed; //Ŀ���ٶ�
    int32_t targetAngle; //Ŀ��Ƕ�
    uint16_t lastAngle;  //�ϴεĽǶ�
    float totalAngle;  //�ۻ��ܹ��Ƕ�
    int16_t turnCount;   //ת����Ȧ��

    int16_t outCurrent; //�������

//    uint8_t InfoUpdateFlag;   //��Ϣ��ȡ���±�־
//    uint16_t InfoUpdateFrame; //֡��
//    uint8_t OffLineFlag;      //�豸���߱�־
} M6020s_t;

void M6020_getInfo(Can_Export_Data_t RxMessage);
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
void CAN_0x1FF_Pitch( int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);

extern M6020s_t M6020s_Yaw;                                    //IDΪ1
extern M6020s_t M6020s_Pitch;                                  //2


















#endif
