#ifndef __M3508_MOTOR_H
#define __M3508_MOTOR_H

#include "BSP_CAN.h"
typedef struct
{
    uint16_t realAngle;  //读回来的机械角度
    int16_t realSpeed;   //读回来的速度
    int16_t realCurrent; //读回来的实际电流
    uint8_t temperture;  //读回来的电机温度

    int16_t targetSpeed;  //目标速度
    uint16_t targetAngle; //目标角度
    uint16_t lastAngle;   //上次的角度
    int32_t totalAngle;   //累积总共角度
    int16_t turnCount;    //转过的圈数

    int16_t outCurrent; //输出电流

    uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
} M3508s_t;

#define M3508_READID_START 0x201
#define M3508_READID_END 0x204


void M3508_getInfo(Can_Export_Data_t RxMessage);
extern M3508s_t M3508s[4];
extern M3508s_t M3508_PowerL; //摩擦轮电机 201
extern M3508s_t M3508_PowerR; //摩擦轮电机; 202



#endif
