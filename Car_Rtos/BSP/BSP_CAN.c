#include "BSP_CAN.h"

void CAN_RxMessage_Export_Date(CAN_HandleTypeDef *hcanx, osMessageQId CANx_Handle, uint8_t Can_type);
void CAN_SendData(osMessageQId CANx_Handle, CAN_HandleTypeDef *CANx, uint8_t id_type, uint32_t id, uint8_t data[8]);

Can_Fun_t Can_Fun = Can_FunGroundInit;
#undef Can_FunGroundInit


CAN_FilterTypeDef can_filter_st[2];
void CAN_FILTER_Init(CAN_HandleTypeDef *hcanx,uint8_t type)
{
    can_filter_st[type].FilterActivation = ENABLE;
    can_filter_st[type].FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st[type].FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st[type].FilterIdHigh = 0x0000;
    can_filter_st[type].FilterIdLow = 0x0000;
    can_filter_st[type].FilterMaskIdHigh = 0x0000;
    can_filter_st[type].FilterMaskIdLow = 0x0000;
    can_filter_st[type].FilterBank = 0;
    can_filter_st[type].FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(hcanx, &can_filter_st[type]);
    HAL_CAN_Start(hcanx);																											//CAN启动!!
    HAL_CAN_ActivateNotification(hcanx, CAN_IT_RX_FIFO0_MSG_PENDING);          //开启CAN接收中断FIFO0 筛选
	
}



void CAN_RxMessage_Export_Date(CAN_HandleTypeDef *hcanx, osMessageQId CANx_Handle, uint8_t canx_type)
{
    Can_Export_Data_t Can_Export_Data[2];
    uint8_t Canx_type = canx_type - 1;
    HAL_CAN_GetRxMessage(hcanx, CAN_RX_FIFO0, &Can_Export_Data[Canx_type].CAN_RxHeader, Can_Export_Data[Canx_type].CANx_Export_RxMessage);
    xQueueSendToBackFromISR(CANx_Handle, &Can_Export_Data[Canx_type], 0); //把接收数据发给接收队列
}
void CAN_SendData(osMessageQId CANx_Handle, CAN_HandleTypeDef *CANx, uint8_t id_type, uint32_t id, uint8_t data[8])
{
    Can_Send_Data_t Can_Send_Data;

    Can_Send_Data.Canx = CANx;
    if (id_type == CAN_ID_STD)
    {
        Can_Send_Data.CAN_TxHeader.StdId = id;
    }
    else
    {
        Can_Send_Data.CAN_TxHeader.ExtId = id; //ID号
    }

    Can_Send_Data.CAN_TxHeader.IDE = id_type;      //ID类型
    Can_Send_Data.CAN_TxHeader.RTR = CAN_RTR_DATA; //发送的为数据
    Can_Send_Data.CAN_TxHeader.DLC = 0x08;         //数据长度为8字节

    memcpy(Can_Send_Data.CANx_Send_RxMessage,
           data,
           sizeof(uint8_t[8]));

    xQueueSend(CANx_Handle, &Can_Send_Data, 0);
}




