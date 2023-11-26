#include "M3508_Motor.h"
M3508s_t M3508s[4];

M3508s_t M3508_PowerL; //Ħ���ֵ�� 201
M3508s_t M3508_PowerR; //Ħ���ֵ��; 202
M3508s_t *M3508_Power[] = {&M3508_PowerL, &M3508_PowerR};




void M3508_getInfo(Can_Export_Data_t RxMessage)
{
    uint32_t StdId;
    StdId = (int32_t)(RxMessage.CAN_RxHeader.StdId - M3508_READID_START);
    //������ݣ����ݸ�ʽ���C620���˵����P33
    M3508s[StdId].realAngle = (uint16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);
    M3508s[StdId].realSpeed = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);
    M3508s[StdId].realCurrent = (int16_t)(RxMessage.CANx_Export_RxMessage[4] << 8 | RxMessage.CANx_Export_RxMessage[5]);
    M3508s[StdId].temperture = RxMessage.CANx_Export_RxMessage[6];

    //֡��ͳ�ƣ����ݸ��±�־λ
//    M3508s[StdId].InfoUpdateFrame++;
//    M3508s[StdId].InfoUpdateFlag = 1;
}

 /* @brief ��CAN�����л�ȡM3508Ħ���ֵ����Ϣ
 * 
 * @param RxMessage 
 * @return  
 */
void M3508_Friction_getInfo(Can_Export_Data_t RxMessage)
{
    M3508s_t *Motor_P = NULL;
    switch (RxMessage.CAN_RxHeader.StdId)
    {
    case 0x205:
        Motor_P = M3508_Power[0];
        break;
    case 0x208:
        Motor_P = M3508_Power[1];
        break;
    }
    if (Motor_P == NULL)
        return;
    //������ݣ����ݸ�ʽ���C620���˵����P33
    Motor_P->realAngle = (uint16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);
    Motor_P->realSpeed = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);
    Motor_P->realCurrent = (int16_t)(RxMessage.CANx_Export_RxMessage[4] << 8 | RxMessage.CANx_Export_RxMessage[5]);
    Motor_P->temperture = RxMessage.CANx_Export_RxMessage[6];

    //֡��ͳ�ƣ����ݸ��±�־λ
//    Motor_P->InfoUpdateFrame++;
//    Motor_P->InfoUpdateFlag = 1;
}
