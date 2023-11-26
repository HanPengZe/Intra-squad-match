#ifndef ___PID_H
#define ___PID_H

#include "main.h"
#define FollowYawAttitude_pidInit \
    {                             \
        0,                        \
            0,                    \
            0,                    \
            0,                    \
            0,                    \
            10,              			\
            0.0f,                 \
            10,                 \
            0,                    \
            0,                    \
            0,                    \
            0,                    \
            5500,                 \
            0,                    \
            0,                    \
            &Position_PID,        \
    }//底盘跟随位置环初始化
#define FollowYawSpeed_pidInit \
    {                          \
        0,                     \
            0,                 \
            0,                 \
            0,                 \
            0,                 \
            5,            \
            0.0f,              \
            10,              \
            0,                 \
            0,                 \
            0,                 \
            0,                 \
            10000,             \
            0,                 \
            0,                 \
            &Position_PID,     \
    }//底盘跟随速度环初始化
#define YawAttitude_PIDInit    \
    {                          \
        0,                     \
            0,                 \
            0,                 \
            0,                 \
            0,                 \
            180,             \
            0.0f,              \
            10,              \
            0,                 \
            0,                 \
            0,                 \
            0,                 \
            18000,             \
            0,                 \
            0,                 \
            &Cloud_IMUYAWOPID, \
    }//云台陀螺仪外环PID初始化
#define YawSpeed_PIDInit       \
    {                          \
        0,                     \
            0,                 \
            0,                 \
            0,                 \
            0,                 \
            6,             \
            0.0f,              \
            55,              \
            0,                 \
            0,                 \
            0,                 \
            0,                 \
            M6020_MaxOutput,   \
            1000,              \
            15000,             \
            &Cloud_IMUYAWIPID, \
    }//云台陀螺仪PID内环初始化
#define M6020s_PitchOPIDInit  \
    {                         \
        0,                    \
            0,                \
            0,                \
            0,                \
            0,                \
            0.35f,            \
            0.0f,             \
            4.5f,             \
            0,                \
            0,                \
            0,                \
            0,                \
            100,              \
            0,                \
            3000,             \
            &Cloud_PITCHOPID, \
    }
#define M6020s_PitchIPIDInit  \
    {                         \
        0,                    \
            0,                \
            0,                \
            0,                \
            0,                \
            100.0f,           \
            2.5f,             \
            0.0f,             \
            0,                \
            0,                \
            0,                \
            0,                \
            M6020_MaxOutput,  \
            30,               \
            4000,             \
            &Cloud_PITCHIPID, \
    }

#define abs_t(x) ((x)>0?(x):-(x))


typedef struct incrementalpid_t
{
    float Target;         //设定目标值
    float Measured;       //测量值
    float err;            //本次偏差值
    float err_last;       //上一次偏差
    float err_beforeLast; //上上次偏差
    float Kp;
    float Ki;
    float Kd; //Kp, Ki, Kd控制系数
    float p_out;
    float i_out;
    float d_out;            //各部分输出值
    float pwm;              //pwm输出
    uint32_t MaxOutput;     //输出限幅
    uint32_t IntegralLimit; //积分限幅
    float (*Incremental_PID)(struct incrementalpid_t *pid_t, float target, float measured);
} incrementalpid_t;
typedef struct positionpid_t
{
    float Target;     //设定目标值
    float Measured;   //测量值
    float err;        //本次偏差值
    float err_last;   //上一次偏差
    float err_change; //误差变化率
    float Kp;
    float Ki;
    float Kd; //Kp, Ki, Kd控制系数
    float p_out;
    float i_out;
    float d_out;               //各部分输出值
    float pwm;                 //pwm输出
    float MaxOutput;           //输出限幅
    float Integral_Separation; //积分分离阈值
    float IntegralLimit;       //积分限幅
    float (*Position_PID)(struct positionpid_t *pid_t, float target, float measured);
} positionpid_t;//底盘跟随PID
 float Position_PID(positionpid_t *pid_t, float target, float measured);
 void abs_limit(float *a, float ABS_MAX);
float Cloud_IMUYAWOPID(positionpid_t *pid_t, float target, float measured);
float Cloud_IMUYAWIPID(positionpid_t *pid_t, float target, float measured);
float Cloud_PITCHIPID(positionpid_t *pid_t, float target, float measured);
float Cloud_PITCHOPID(positionpid_t *pid_t, float target, float measured);
float Incremental_PID(incrementalpid_t *pid_t, float target, float measured);
void Clear_IncrementalPIDData(incrementalpid_t *pid_t);

#endif









