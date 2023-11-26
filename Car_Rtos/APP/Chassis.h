#ifndef __CHIASSIS_H
#define __CHIASSIS_H
#include <stdint.h>
#include "PID.h"

#define CAN_CHASSIS_ALL_ID  	0x200
#define CAN_2006_M1_ID  			0x201
#define CAN_2006_M2_ID  			0x202
#define CAN_2006_M3_ID  			0x203
#define CAN_2006_M4_ID  			0x204
#define CAN_YAW_MOTOR_ID  		0x205
#define M6020_mAngleRatio 22.7527f //机械角度与真实角度的比率
#define Cloud_Yaw_Center 0
#define WheelMaxSpeed 9000.0f //单轮最大速度
#define M3508_MaxOutput 16384     //发送给电机的最大控制值

#define LFWHEEL_PID_PARAM     \
    {                         \
        0,                    \
            0,                \
            0,                \
            0,                \
            0,                \
            10,            \
						0.01,             \
            10,             \
            0,                \
            0,                \
            0,                \
            0,                \
            M3508_MaxOutput,  \
            3000,             \
            &Incremental_PID, \
    }
#define RFWHEEL_PID_PARAM     \
    {                         \
        0,                    \
            0,                \
            0,                \
            0,                \
            0,                \
            10,            \
            0.01,             \
            10,             \
            0,                \
            0,                \
            0,                \
            0,                \
            M3508_MaxOutput,  \
            3000,             \
            &Incremental_PID, \
    }
#define LBWHEEL_PID_PARAM     \
    {                         \
        0,                    \
            0,                \
            0,                \
            0,                \
            0,                \
            10,            \
            0.01,             \
            10,             \
            0,                \
            0,                \
            0,                \
            0,                \
            M3508_MaxOutput,  \
            3000,             \
            &Incremental_PID, \
    }
#define RBWHEEL_PID_PARAM     \
    {                         \
        0,                    \
            0,                \
            0,                \
            0,                \
            0,                \
            10,            \
            0.01,             \
            10,             \
            0,                \
            0,                \
            0,                \
            0,                \
            M3508_MaxOutput,  \
            3000,             \
            &Incremental_PID, \
    }


//typedef struct
//{
//    uint16_t ecd;
//    int16_t speed_rpm;
//    int16_t given_current;
//    uint8_t temperate;
//    int16_t last_ecd;
//} motor_measure_t;

typedef struct 
{
	float NowYaw;
	float TargetYaw;
	float In_Put;
	float Out_Put;
	
	
}Chassis_t;

extern Chassis_t Chassis;

//void get_motor_measure(motor_measure_t *ptr,uint8_t *data);
//void set_2006_current(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
int ComputeMinOffset(int target, int value) ;
void Chassis_Follow( int Yaw_Center);
void set_3508_current(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void Chassis_Move(int16_t *speed);
void MecanumCalculate(float Vx, float Vy, float VOmega, int16_t *Speed);




#endif










