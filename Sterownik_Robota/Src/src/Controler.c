/**
  ******************************************************************************
  * @file	 Controler.c
  * @author  Przemek
  * @version V1.0.0
  * @date    03.04.2018
  * @brief   Modu� g��wnego sterownika robota
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "MotorInterface.h"
#include "MPU/rx_data.h"
#include "MPU/MPU6050.h"
#include "OSCan.h"
#include "OSUart.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
/* Private typedef -----------------------------------------------------------*/
typedef struct{
	tMPUHandler hmpu;
	tCAN2UARTHandle c2uf;
	OsUARTHandler huart;
	tMotorInterfaceHandler leftMotor;
	tMotorInterfaceHandler rightMotor;
	tMPUMeasuremenet mmpu;
	xTaskHandle task;
	xTaskHandle logTask;
	char log[256];
}tControler;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
tControler controler;
/* Private function prototypes -----------------------------------------------*/
void Controler_Task(void* ptr);
void Controler_SenderTask(void* ptr);
/* Public  functions ---------------------------------------------------------*/
/**
  * @brief  Funkcja inicjuje g��wny kontroler ruchu robota
  * @param[in]  None
  * @retval None
  */
int Controler_Init(void){
	tMPUHardwareSetting mpuhw;
	tMPUConfiguration mpucfg;
	tCanInit cfg;
	tCAN2UARTConfig c2uc;
	tMotorInterfaceConfig mic;
	//inicjuje modu� CAN
	cfg.rxBufferSize=30;
	OSCan_Init(&hcan1,&cfg);
	//inicjuje modu� UART2CAN
	c2uc.canFifoNumber=1;
	c2uc.maxNumberOfChannels=3;
	if(CAN2UART_Init(&controler.c2uf,&c2uc)){
		return 1;
	}
	//inicjuje UART
	if(OsUART_Init(&controler.huart,&huart1)){
		return 4;
	}
	//inicjuje modu� interfejsu silnik�w
	mic.c2u = controler.c2uf;
	mic.canId = 1;
	mic.numPolePairs = 15;
	mic.reversMode = 0;
	if(MotorInterface_Init(&controler.leftMotor,&mic)){
		return 2;
	}
	mic.c2u = controler.c2uf;
	mic.canId = 2;
	mic.numPolePairs = 15;
	mic.reversMode = 1;
	if(MotorInterface_Init(&controler.rightMotor,&mic)){
		return 3;
	}
	//inicjuje modu� MPU
	mpucfg.queueDepth = 24;
	if(MPU6050_Init(&controler.hmpu,&mpuhw,&mpucfg)){
		return 4;
	}
	//tworze w�tek kontrolera
	xTaskCreate(Controler_Task,"controller",256,&controler,5,&controler.task);
	//tworze wątek podglądu parametrów
	xTaskCreate(Controler_SenderTask,"ctrl_log",256,&controler,5,&controler.logTask);
	return 0;
}
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  W�tek kontrolera ruchu
  * @param[in]  None
  * @retval None
  */
void Controler_Task(void* ptr){
	vTaskDelay(400);
	//uruchamiam licznik do przechwytywania zdażeń od MPU
	HAL_TIM_Base_Start(&htim12);
	HAL_TIM_IC_Start_IT(&htim12,TIM_CHANNEL_1);
	while(1){
		if(MPU6050_GetMeasurement(controler.hmpu,&controler.mmpu,100)==0){
			//odebrano nowy pomiar
		}else{
			//nie odebrano pomiaru z MPU
		}
	}
}
/**
  * @brief  Wątek wysyła z pewnym okresem dane pomairowe naport UART
  * @param[in]  None
  * @retval None
  */
void Controler_SenderTask(void* ptr){
	int s;
	vTaskDelay(1000);
	while(1){
		vTaskDelay(100);
		s=sprintf(controler.log,"LOG: Roll:%d Pitch:%d Yaw: %d AccX:%d AccY:%d AccZ:%d \n\r",\
				(int)controler.mmpu.rpy[0],\
				(int)controler.mmpu.rpy[1],\
				(int)controler.mmpu.rpy[2],\
				(int)controler.mmpu.acceleration[0],\
				(int)controler.mmpu.acceleration[1],\
				(int)controler.mmpu.acceleration[2]);
		//wysyłam
		OsUART_Transmit(controler.huart,(unsigned char*)controler.log,s,0);
	}
}
/**
  * @brief  Funkcja przerwania od timera TIM12, obsłógującego linie INT modułu MPU
  * @param[in]  None
  * @retval None
  */
void TIM8_BRK_TIM12_IRQHandler(void)
{
	static unsigned int lastCapture=0;
	if(__HAL_TIM_GET_FLAG(&htim12, TIM_FLAG_CC1) != RESET){
	    if(__HAL_TIM_GET_IT_SOURCE(&htim12, TIM_IT_CC1) !=RESET){
	        __HAL_TIM_CLEAR_IT(&htim12, TIM_IT_CC1);
	        MPU6050_UpdateFromISR(controler.hmpu,TIM12->CCR1-lastCapture);
	        lastCapture = TIM12->CCR1;
	    }
	}
}
