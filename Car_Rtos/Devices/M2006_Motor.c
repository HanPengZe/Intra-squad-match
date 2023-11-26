#include "M2006_Motor.h"
M2006s_t M2006_Reload; //����������
M2006s_t *M2006_Array[] = {&M2006_Reload};


void M2006_getInfo(Can_Export_Data_t RxMessage)
{
    //����idȷ��
    // if ((RxMessage.CAN_RxHeader.StdId < M2006_READID_START) || (RxMessage.CAN_RxHeader.StdId > M2006_READID_END))
    // 	return;
    int32_t StdId;
    StdId = (int32_t)RxMessage.CAN_RxHeader.StdId - M2006_READID_START;
    // if (IndexOutofBounds(StdId, M2006_Amount))
    // {
    // 	Device_setAlertType(Alert_Times_SoftWare);
    // 	return;
    // }
    M2006_Array[StdId]->lastAngle = M2006_Array[StdId]->realAngle;
    //������ݣ����ݸ�ʽ���C610���˵����P9
    M2006_Array[StdId]->realAngle = (uint16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);
    M2006_Array[StdId]->realSpeed = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);
    M2006_Array[StdId]->realTorque = (int16_t)(RxMessage.CANx_Export_RxMessage[4] << 8 | RxMessage.CANx_Export_RxMessage[5]);

    if (M2006_Array[StdId]->realAngle - M2006_Array[StdId]->lastAngle < -6000)
    {
        M2006_Array[StdId]->turnCount++;
    }

    if (M2006_Array[StdId]->lastAngle - M2006_Array[StdId]->realAngle < -6000)
    {
        M2006_Array[StdId]->turnCount--;
    }
    M2006_Array[StdId]->totalAngle = M2006_Array[StdId]->realAngle + (8192 * M2006_Array[StdId]->turnCount);
    M2006_Array[StdId]->lastAngle = M2006_Array[StdId]->realAngle;
//    M2006_Array[StdId]->InfoUpdateFrame++;
//    M2006_Array[StdId]->InfoUpdateFlag = 1;
}
