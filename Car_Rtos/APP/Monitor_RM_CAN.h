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
	

//���̹���ģʽ
typedef enum
{
    ChassisWorkMode_Follow = 0,        //������̨ģʽ
    ChassisWorkMode_NewFollow = 1,     //�������ģʽ
    ChassisWorkMode_Reversefollow = 2, //���������̨ģʽ
    ChassisWorkMode_Spin = 3,          //���߱���ģʽ
    ChassisWorkMode_Twister,           //Ť��ģʽ
    ChassisWorkMode_AutoTrace,         //�Զ�׷��ģʽ
    ChassisWorkMode_Supply,            //����ģʽ
    ChassisWorkMode_Lock,
    ChassisWorkMode_Disable //ʧ��ģʽ
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
