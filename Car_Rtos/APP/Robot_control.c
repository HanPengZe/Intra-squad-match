#include "Robot_control.h"
#include "Monitor_RM_CAN.h"
#include "M3508_Motor.h"
#include "M6020_Motor.h"
#include "M2006_Motor.h"
#include "Chassis.h"
#include "Cloud.h"
#include "DR16_Remote.h"


void CAN_SendControl(void)
{
    //解决一个报文发给不同电机的问题。
    //CAN1 ：yaw = 1 ， 拨盘 = 7
    //CAN2 ： pitch = 2

    // -------  CAN1
    Monitor_CAN_FUN.CAN_0x200_SendData(&hcan1, M3508s[0].outCurrent, M3508s[1].outCurrent, M3508s[2].outCurrent, M3508s[3].outCurrent); //底盘电机(3508)

    Monitor_CAN_FUN.CAN_0x1FF_SendData(&hcan1, M6020s_Yaw.outCurrent, 0, 0, 0); //pitch轴电机(6020)，yaw轴电机(6020)。

    // Monitor_CAN_FUN.CAN_0x2FF_SendData(&hcan1, 5000, 0, 0, 0); //pitch轴电机(6020)，yaw轴电机(6020)。

//    Monitor_CAN_FUN.CAN_0x601_SendData(&hcan1, supercapacitor.SendData.data);
    // -------  CAN2
    Monitor_CAN_FUN.CAN_0x1FF_SendData(&hcan2, M3508_PowerL.outCurrent, M6020s_Pitch.outCurrent, M2006_Reload.outCurrent, M3508_PowerR.outCurrent);
}

void Robot_control()
{
	Cloud_Yaw_PID(RC_Ctl.rc.ch0);	
	Chassis_Follow(Cloud_Yaw_Center);	
	if(RC_Ctl.rc.s1==2&&RC_Ctl.rc.s2==2)
	{
		RC_Ctl.rc.ch0=0;
		RC_Ctl.rc.ch1=0;
		RC_Ctl.rc.ch2=0;
		RC_Ctl.rc.ch3=0;
		for (int i = 0; i < 4; i++)
    {
			
			
		  M3508s[i].outCurrent = 0;
		  
//             Clear_IncrementalPIDData(Wheel_PID[i]);
    } 
		M6020s_Yaw.outCurrent=0;
		 
	}
			M6020s_Pitch.outCurrent=0;
			M3508_PowerL.outCurrent=0;
			M2006_Reload.outCurrent=0;
			M3508_PowerR.outCurrent=0;

	CAN_SendControl();
	
}







