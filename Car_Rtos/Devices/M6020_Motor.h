#ifndef __M6020_MOTOR_H
#define __M6020_MOTOR_H
#include "stdint.h"
#include "BSP_CAN.h"
#define M6020_READID_START 0x205 //当ID为1时的报文ID
#define M6020_READID_END 0x206
#define M6020_SENDID 0x1FF //1~4的电机，0x2FF为5~7

typedef struct
{
    uint16_t realAngle;  //读回来的机械角度
    int16_t realSpeed;   //读回来的速度
    int16_t realCurrent; //读回来的实际转矩电流
    uint8_t temperture;  //读回来的电机温度

    int16_t targetSpeed; //目标速度
    int32_t targetAngle; //目标角度
    uint16_t lastAngle;  //上次的角度
    float totalAngle;  //累积总共角度
    int16_t turnCount;   //转过的圈数

    int16_t outCurrent; //输出电流

//    uint8_t InfoUpdateFlag;   //信息读取更新标志
//    uint16_t InfoUpdateFrame; //帧率
//    uint8_t OffLineFlag;      //设备离线标志
} M6020s_t;

void M6020_getInfo(Can_Export_Data_t RxMessage);
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
void CAN_0x1FF_Pitch( int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);

extern M6020s_t M6020s_Yaw;                                    //ID为1
extern M6020s_t M6020s_Pitch;                                  //2


















#endif
