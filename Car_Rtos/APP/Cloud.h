#ifndef __CLOUD_H
#define __CLOUD_H
#define CAN_GIMBAL_ALL_ID  		0x1FF
#define M6020_MaxOutput 30000 //发送给电机的最大控制值
#include <stdint.h>

typedef struct 
{
	float NowYaw;
	float TargetYaw;
	float In_Put;
	float Out_Put;
	float NowPitch;
	float TargetPitch;
	
}Cloud_t;


extern Cloud_t Cloud_Yaw;
extern Cloud_t Cloud_Pitch;

//void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
//void Cloud_Init(void);
void Cloud_Yaw_PID(int16_t yaw);
void Cloud_Pitch_PID(int16_t pitch);

void cloud_Mpu(void);
//void PID_Init(void);













#endif
