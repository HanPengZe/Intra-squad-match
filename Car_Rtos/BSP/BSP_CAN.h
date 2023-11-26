#ifndef __BSP_CAN
#define __BSP_CAN

#include "can.h"
#include "cmsis_os.h"
#include <string.h>

#define can1_type 1
#define can2_type 2
#define Can_FunGroundInit           \
    {                               \
        &CAN_RxMessage_Export_Date \
    }

extern osMessageQId CAN1_ReceiveHandle;
extern osMessageQId CAN2_ReceiveHandle;

typedef struct
{
    CAN_RxHeaderTypeDef CAN_RxHeader;
    uint8_t CANx_Export_RxMessage[8];
} Can_Export_Data_t;

typedef struct
{
    CAN_HandleTypeDef *Canx;
    CAN_TxHeaderTypeDef CAN_TxHeader;
    uint8_t CANx_Send_RxMessage[8];
} Can_Send_Data_t;

typedef struct
{
    void (*CAN_RxMessage_Export_Date)(CAN_HandleTypeDef *hcanx, osMessageQId CANx_Handle, uint8_t Can_type);
    void (*CAN_SendData)(osMessageQId CANx_Handle, CAN_HandleTypeDef *CANx, uint8_t id_type, uint32_t id, uint8_t data[8]);
} Can_Fun_t;


void CAN_FILTER_Init(CAN_HandleTypeDef *hcanx,uint8_t type);
extern Can_Fun_t Can_Fun;


















#endif
