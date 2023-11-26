#include "M6020_Motor.h"
#include "Cloud.h"

M6020s_t M6020s_Yaw;                                    //ID为1
M6020s_t M6020s_Pitch;                                  //2
M6020s_t *M6020_Array[] = {&M6020s_Yaw, &M6020s_Pitch}; //对应电机的ID必须为：索引+1

void M6020_getInfo(Can_Export_Data_t RxMessage)
{

    int32_t StdId;
    StdId = (int32_t)RxMessage.CAN_RxHeader.StdId - M6020_READID_START; //由0开始
    // if (IndexOutofBounds(StdId, M6020_Amount))
    // {
    // 	Device_setAlertType(Alert_Times_SoftWare);
    // 	return;
    // }

    //解包数据，数据格式详见C620电调说明书P33
    M6020_Array[StdId]->lastAngle = M6020_Array[StdId]->realAngle;
    M6020_Array[StdId]->realAngle = (uint16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);
    M6020_Array[StdId]->realSpeed = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);
    M6020_Array[StdId]->realCurrent = (int16_t)(RxMessage.CANx_Export_RxMessage[4] << 8 | RxMessage.CANx_Export_RxMessage[5]);
    M6020_Array[StdId]->temperture = RxMessage.CANx_Export_RxMessage[6];

    if (M6020_Array[StdId]->realAngle - M6020_Array[StdId]->lastAngle < -6500)
    {
        M6020_Array[StdId]->turnCount++;
    }

    if (M6020_Array[StdId]->lastAngle - M6020_Array[StdId]->realAngle < -6500)
    {
        M6020_Array[StdId]->turnCount--;
    }

    M6020_Array[StdId]->totalAngle = M6020_Array[StdId]->realAngle + (8192 * M6020_Array[StdId]->turnCount);

    //帧率统计，数据更新标志位
//    M6020_Array[StdId]->InfoUpdateFrame++;
//    M6020_Array[StdId]->InfoUpdateFlag = 1;
}
static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
int a=0;
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;
   a= HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

uint8_t data[8];
static CAN_TxHeaderTypeDef  Pitch_tx_message;
 int p=0;
void CAN_0x1FF_Pitch( int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{

	uint32_t send_Pitch_box;
  Pitch_tx_message.StdId = CAN_GIMBAL_ALL_ID;
  Pitch_tx_message.IDE = CAN_ID_STD;
  Pitch_tx_message.RTR = CAN_RTR_DATA;
  Pitch_tx_message.DLC = 0x08;
	data[0] = iq1 >> 8;
	data[1] = iq1;
	data[2] = iq2 >> 8;
	data[3] = iq2;
	data[4] = iq3 >> 8;
	data[5] = iq3;
	data[6] = iq4 >> 8;
	data[7] = iq4;
	p=HAL_CAN_AddTxMessage(&hcan2, &Pitch_tx_message, data, &send_Pitch_box);

}

