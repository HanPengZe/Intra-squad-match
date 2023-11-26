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
    }//���̸���λ�û���ʼ��
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
    }//���̸����ٶȻ���ʼ��
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
    }//��̨�������⻷PID��ʼ��
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
    }//��̨������PID�ڻ���ʼ��
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
    float Target;         //�趨Ŀ��ֵ
    float Measured;       //����ֵ
    float err;            //����ƫ��ֵ
    float err_last;       //��һ��ƫ��
    float err_beforeLast; //���ϴ�ƫ��
    float Kp;
    float Ki;
    float Kd; //Kp, Ki, Kd����ϵ��
    float p_out;
    float i_out;
    float d_out;            //���������ֵ
    float pwm;              //pwm���
    uint32_t MaxOutput;     //����޷�
    uint32_t IntegralLimit; //�����޷�
    float (*Incremental_PID)(struct incrementalpid_t *pid_t, float target, float measured);
} incrementalpid_t;
typedef struct positionpid_t
{
    float Target;     //�趨Ŀ��ֵ
    float Measured;   //����ֵ
    float err;        //����ƫ��ֵ
    float err_last;   //��һ��ƫ��
    float err_change; //���仯��
    float Kp;
    float Ki;
    float Kd; //Kp, Ki, Kd����ϵ��
    float p_out;
    float i_out;
    float d_out;               //���������ֵ
    float pwm;                 //pwm���
    float MaxOutput;           //����޷�
    float Integral_Separation; //���ַ�����ֵ
    float IntegralLimit;       //�����޷�
    float (*Position_PID)(struct positionpid_t *pid_t, float target, float measured);
} positionpid_t;//���̸���PID
 float Position_PID(positionpid_t *pid_t, float target, float measured);
 void abs_limit(float *a, float ABS_MAX);
float Cloud_IMUYAWOPID(positionpid_t *pid_t, float target, float measured);
float Cloud_IMUYAWIPID(positionpid_t *pid_t, float target, float measured);
float Cloud_PITCHIPID(positionpid_t *pid_t, float target, float measured);
float Cloud_PITCHOPID(positionpid_t *pid_t, float target, float measured);
float Incremental_PID(incrementalpid_t *pid_t, float target, float measured);
void Clear_IncrementalPIDData(incrementalpid_t *pid_t);

#endif









