#ifndef __Monitor_RM_CAN
#define __Monitor_RM_CAN
#include "BSP_CAN.h"
#include "cmsis_os.h"
#include "can.h"

#define Monitor_CAN_FUNGroundInit \
	{                             	\
			&CAN_0x1FF_SendData,      	\
			&CAN_0x200_SendData,  			\
			&CAN_0x2FF_SendData,  			\
	}		
//			&CAN_0x601_SendData,  			\
	

//底盘工作模式
typedef enum
{
    ChassisWorkMode_Follow = 0,        //跟随云台模式
    ChassisWorkMode_NewFollow = 1,     //跟随底盘模式
    ChassisWorkMode_Reversefollow = 2, //反向跟随云台模式
    ChassisWorkMode_Spin = 3,          //边走边旋模式
    ChassisWorkMode_Twister,           //扭腰模式
    ChassisWorkMode_AutoTrace,         //自动追踪模式
    ChassisWorkMode_Supply,            //补给模式
    ChassisWorkMode_Lock,
    ChassisWorkMode_Disable //失能模式
} ChassisWorkMode_e;



typedef struct
{
	void (*CAN_0x1FF_SendData)(CAN_HandleTypeDef *CAN_Num, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
	void (*CAN_0x200_SendData)(CAN_HandleTypeDef *CAN_Num, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
	void (*CAN_0x2FF_SendData)(CAN_HandleTypeDef *CAN_Num, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
//	void (*CAN_0x601_SendData)(CAN_HandleTypeDef *CAN_Num, uint8_t data[8]);

} Monitor_CAN_FUN_t;

extern Monitor_CAN_FUN_t Monitor_CAN_FUN;
extern osMessageQId CAN1_ReceiveHandle;
extern osMessageQId CAN2_ReceiveHandle;
extern osMessageQId CAN_SendHandle;





#endif
