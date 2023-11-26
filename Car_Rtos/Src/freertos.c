/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BSP_CAN.h"
#include "M6020_Motor.h"
#include "M3508_Motor.h"
#include "DJI_C_IMU.h"
#include "Robot_control.h"
#include "M6020_Motor.h"
#include "DR16_Remote.h"
#include "Cloud.h"
#include "Monitor_RM_CAN.h"
#include "Chassis.h"
#include "PID.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osMessageQId CAN_SendHandle;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId Task_CAN_ReceivHandle;
osThreadId Tack_CAN_SendHandle;
osThreadId Task_Can2_RXHandle;
osMessageQId CAN1_ReceiveHandle;
osMessageQId CAN2_ReceiveHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void CAN_Receive_All(void const * argument);
void CAN_Send(void const * argument);
void CAN2_RX(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of CAN1_Receive */
  osMessageQDef(CAN1_Receive, 32, Can_Export_Data_t);
  CAN1_ReceiveHandle = osMessageCreate(osMessageQ(CAN1_Receive), NULL);

  /* definition and creation of CAN2_Receive */
  osMessageQDef(CAN2_Receive, 32, Can_Export_Data_t);
  CAN2_ReceiveHandle = osMessageCreate(osMessageQ(CAN2_Receive), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	osMessageQDef(CAN_Send_All,32,Can_Send_Data_t);
	CAN_SendHandle=osMessageCreate(osMessageQ(CAN_Send_All),NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityLow, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Task_CAN_Receiv */
  osThreadDef(Task_CAN_Receiv, CAN_Receive_All, osPriorityHigh, 0, 256);
  Task_CAN_ReceivHandle = osThreadCreate(osThread(Task_CAN_Receiv), NULL);

  /* definition and creation of Tack_CAN_Send */
  osThreadDef(Tack_CAN_Send, CAN_Send, osPriorityNormal, 0, 256);
  Tack_CAN_SendHandle = osThreadCreate(osThread(Tack_CAN_Send), NULL);

  /* definition and creation of Task_Can2_RX */
  osThreadDef(Task_Can2_RX, CAN2_RX, osPriorityHigh, 0, 256);
  Task_Can2_RXHandle = osThreadCreate(osThread(Task_Can2_RX), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {

    osDelay(5);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_CAN_Receive_All */
/**
* @brief Function implementing the Task_CAN_Receiv thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Receive_All */
void CAN_Receive_All(void const * argument)
{
  /* USER CODE BEGIN CAN_Receive_All */
		uint32_t ID;
    Can_Export_Data_t Can_Export_Data;

  /* Infinite loop */
  for(;;)
  {
    xQueueReceive(CAN1_ReceiveHandle, &Can_Export_Data, portMAX_DELAY);
    ID = Can_Export_Data.CAN_RxHeader.StdId;
		if(ID==M6020_READID_START)
		{
			M6020_getInfo(Can_Export_Data);
		}
    else if (ID >= M3508_READID_START || ID <= M3508_READID_END)
    {
      M3508_getInfo(Can_Export_Data);
    }
  }
  /* USER CODE END CAN_Receive_All */
}

/* USER CODE BEGIN Header_CAN_Send */
/**
* @brief Function implementing the Tack_CAN_Send thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Send */
void CAN_Send(void const * argument)
{
  /* USER CODE BEGIN CAN_Send */
	
//	  Can_Send_Data_t Can_Send_Data;

	
  /* Infinite loop */
  for(;;)
  {
		Cloud_Pitch_PID(RC_Ctl.rc.ch1);
		Cloud_Yaw_PID(RC_Ctl.rc.ch0);	
		Chassis_Follow(Cloud_Yaw_Center);
		int16_t speed[4];
		MecanumCalculate(RC_Ctl.rc.ch3,RC_Ctl.rc.ch2,-Chassis.Out_Put,speed);	
		Chassis_Move(speed);
		
//	 else if(RC_Ctl.rc.s2==3)
//	 {
//		 		 
//	 }
	  if(RC_Ctl.rc.s1==2&&RC_Ctl.rc.s2==2)
		{
			Cloud_Yaw .Out_Put=0;
			Cloud_Pitch.Out_Put=0;
			for (int i = 0; i < 4; i++)
        {
          M3508s[i].outCurrent = 0;
								
        }		
		}
	
			CAN_cmd_gimbal(-Cloud_Yaw .Out_Put,0,0,0);
			set_3508_current(M3508s[0].outCurrent, M3508s[1].outCurrent, M3508s[2].outCurrent, M3508s[3].outCurrent);		
			CAN_0x1FF_Pitch(0,-Cloud_Pitch.Out_Put,0,0);

//		Monitor_CAN_FUN.CAN_0x1FF_SendData(&hcan1, M6020s_Yaw.outCurrent, 0, 0, 0); //pitch轴电机(6020)，yaw轴电机(6020)。

//				Robot_control();
//		    xQueueReceive(CAN_SendHandle, &Can_Send_Data, portMAX_DELAY);
//        HAL_CAN_AddTxMessage(Can_Send_Data.Canx, &Can_Send_Data.CAN_TxHeader, Can_Send_Data.CANx_Send_RxMessage, (uint32_t *)CAN_TX_MAILBOX0);
						osDelay(5);
  }
  /* USER CODE END CAN_Send */
}

/* USER CODE BEGIN Header_CAN2_RX */
/**
* @brief Function implementing the Task_Can2_RX thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN2_RX */
void CAN2_RX(void const * argument)
{
  /* USER CODE BEGIN CAN2_RX */
	  uint32_t ID;
    Can_Export_Data_t Can_Export_Data;

  /* Infinite loop */
  for(;;)
  {
		static uint8_t mark=0;
    xQueueReceive(CAN2_ReceiveHandle, &Can_Export_Data, portMAX_DELAY);
    ID = Can_Export_Data.CAN_RxHeader.StdId;
		if(ID==M6020_READID_END)
		{
				M6020_getInfo(Can_Export_Data);
		}
    else if (ID == DJI_C_Angle)
    {
        DJI_C_IMUFUN.DJI_C_Euler_getInfo(Can_Export_Data);
    }
    else if (ID == DJI_C_Gyro)
    {
        DJI_C_IMUFUN.DJI_C_Gyro_getInfo(Can_Export_Data);                                  
    }
		DJI_C_IMUFUN.Updata_Hand_Euler_Gyro_Data();
		if(mark==0)
		{
			cloud_Mpu();
			mark=1;
		}
				
  }
  /* USER CODE END CAN2_RX */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
