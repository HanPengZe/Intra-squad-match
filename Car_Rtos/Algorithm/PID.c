#include "PID.h"
#include "math.h"
#include "stdlib.h" 

static void abs_limit(float *a, float ABS_MAX)
{
    if (*a > ABS_MAX)
        *a = ABS_MAX;
    if (*a < -ABS_MAX)
        *a = -ABS_MAX;
}
/****************************************/

/***************λ��ʽPID****************/
float Position_PID(positionpid_t *pid_t, float target, float measured)
{

    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //�����޷�
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //ȡ������������޷���

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    //����޷�
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}
///****************************************/

///***************����ʽPID****************/
float Incremental_PID(incrementalpid_t *pid_t, float target, float measured)
{

    pid_t->Target = target;
    pid_t->Measured = measured;
    pid_t->err = pid_t->Target - pid_t->Measured;

    //	if(abs(pid_t->err)<0.1f)
    //		pid_t->err = 0.0f;
    //return 0;

    pid_t->p_out = pid_t->Kp * (pid_t->err - pid_t->err_last);
    pid_t->i_out = pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - 2.0f * pid_t->err_last + pid_t->err_beforeLast);

    //�����޷�
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit);

    pid_t->pwm += (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    //����޷�
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_beforeLast = pid_t->err_last;
    pid_t->err_last = pid_t->err;

    return pid_t->pwm;
}
/**
 * @brief ��̨YAW���⻷PID
 * 
 * @param pid_t 
 * @param target 
 * @param measured 
 */
float Cloud_IMUYAWOPID(positionpid_t *pid_t, float target, float measured)
{
    if (pid_t == NULL)
    {
        return 0;
    }
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
//    pid_t->d_out = KalmanFilter(&Cloud_YAWODKalman, pid_t->d_out);
    //�����޷�
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //ȡ������������޷���

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    //����޷�
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}
/**
 * @brief ��̨YAW���ڻ�PID
 * 
 * @param pid_t 
 * @param target 
 * @param measured 
 */
float Cloud_IMUYAWIPID(positionpid_t *pid_t, float target, float measured)
{
    if (pid_t == NULL)
    {
        return 0;
    }
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;
    // float Absolute_Measured = measured;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //�����޷�
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //ȡ������������޷���
    if (abs_t(pid_t->err) >= pid_t->Integral_Separation)
    {
        pid_t->pwm = (pid_t->p_out + pid_t->d_out);
    }
    else
    {
        pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
    }

    //����޷�
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}
/**
 * @brief ��̨PITCH���⻷PID
 * 
 * @param pid_t 
 * @param target 
 * @param measured 
 */
float Cloud_PITCHOPID(positionpid_t *pid_t, float target, float measured)
{
    if (pid_t == NULL)
    {
        return 0;
    }
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
//    pid_t->d_out = KalmanFilter(&Cloud_PITCHODKalman, pid_t->d_out);
    //�����޷�
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //ȡ������������޷���

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    //����޷�
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

/**
 * @brief ��̨PITCH���ڻ�PID
 * 
 * @param pid_t 
 * @param target 
 * @param measured 
 */
float Cloud_PITCHIPID(positionpid_t *pid_t, float target, float measured)
{
    if (pid_t == NULL)
    {
        return 0;
    }
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //�����޷�
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //ȡ������������޷���

    if (abs_t(pid_t->err) >= pid_t->Integral_Separation)
    {
        pid_t->pwm = (pid_t->p_out + pid_t->d_out);
    }
    else
    {
        pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
    }

    //����޷�
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

void Clear_IncrementalPIDData(incrementalpid_t *pid_t)
{
    pid_t->Target = 0;
    pid_t->Measured = 0;
    pid_t->err = 0;
    pid_t->err_last = 0;
    pid_t->err_beforeLast = 0;
    pid_t->p_out = 0;
    pid_t->i_out = 0;
    pid_t->d_out = 0;
    pid_t->pwm = 0;
}











