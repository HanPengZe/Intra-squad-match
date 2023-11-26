#ifndef __M2006_MOTOR_H
#define __M2006_MOTOR_H
#include <stdint.h>
#include "BSP_CAN.h"

#define M2006_READID_START 0x207
#define M2006_READID_END 0x207
#define M2006_SENDID 0x1FF    //控制5-8的电机ID
#define M2006_MaxOutput 10000 //发送给电机的最大控制值
#define M2006_LOADANGLE 36864
#define M2006_ReductionRatio 36 //电机减速比


typedef struct
{
    uint16_t realAngle; //读回来的机械角度
    int16_t realSpeed;  //读回来的速度
    int16_t realTorque; //读回来的实际转矩

    int16_t targetSpeed; //目标速度
    int32_t targetAngle; //目标角度

    uint16_t lastAngle; //上次的角度
    int32_t totalAngle; //累积总共角度
    int16_t turnCount;  //转过的圈数

    int16_t outCurrent;      //输出电流
    int16_t inneroutCurrent; //输出电流

    uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
} M2006s_t;

extern M2006s_t M2006_Reload; //拨盘运输电机


















#endif

