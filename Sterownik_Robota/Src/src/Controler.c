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
#include "can.h"
/* Private typedef -----------------------------------------------------------*/
typedef struct{
	tMPUHandler hmpu;
	tCAN2UARTHandle c2uf;
	tMotorInterfaceHandler leftMotor;
	tMotorInterfaceHandler rightMotor;
	xTaskHandle task;
}tControler;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
tControler controler;
/* Private function prototypes -----------------------------------------------*/
void Controler_Task(void* ptr);
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
	return 0;
}
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  W�tek kontrolera ruchu
  * @param[in]  None
  * @retval None
  */
void Controler_Task(void* ptr){
	while(1){

	}
}
