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
#include "UartLogStreamer.h"
#include "LabViewUartStreamer.h"
#include "LedIndicator.h"
#include "Radio.h"
/* Private typedef -----------------------------------------------------------*/
typedef enum{eSupervisorLeftMotorInactive=0,/*<lewy silnik w trybie nieaktywnym*/
			eSupervisorRightMotorInactive,/*<prawy silnik w trybie nieaktywnym*/
			eSupervisorLeftMotorActive,/*<lewy silnik w trybie aktywnym*/
			eSupervisorRightMotorActive,/*<prawy silnik w trybie aktywnym*/
			eSupervisorMPUSensorNoTrigger/*<Czujnik MPU przestał zgłaszac pomiiary na linii INT*/
			}tSupervisorMsg;
typedef enum{
	eSystem_InternalInit=0,/*<tryb pocztkowej inicjacji*/
	eSystem_MotorInit,/*<tryb inicjacji silników i oczekiwania na przełączenie w tryb pracy aktywnej*/
	eSystem_ReadyToWork,/*<tryb aktywnej pracy silników bez stabilizacji robota*/
	eSystem_RobotStabilisation,/*<tryb pełnej stabilizacji robota*/
	eSystem_FaultState,/*<awaria podsystemu czujników lub silników*/
}tSupervisorSystemState;

typedef struct{
	tMPUHandler hmpu;
	tCAN2UARTHandle c2uf;
	OsUARTHandler huart;
	tMotorInterfaceHandler leftMotor;
	tMotorInterfaceHandler rightMotor;
	tMotorMeasuremenets *leftMotorMeasurement;
	tMotorMeasuremenets *rightMotorMeasurement;
	tMPUMeasuremenet mmpu;
	tLogerHandler loger;
	tLedIndictorHandler ledIndicator;
	void* radio;
	int* radioMeasuremenets;
	xTaskHandle task;
	xTaskHandle supervisorTask;
	xQueueHandle supervisorMsgQueue;
	struct{
		int leftMotorActive:1;
		int rightMotorActive:1;
		tSupervisorSystemState state;
	}supervisor;
}tControler;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
tControler controler;
volatile unsigned long ulHighFrequencyTimerTicks = 0;
/* Private function prototypes -----------------------------------------------*/
void Controler_Task(void* ptr);
void Controler_SupervisorTask(void* ptr);
void Controler_SenderTask(void* ptr);
void Supervisor_NewMsg(tSupervisorMsg msg);
void Supervisor_SwitchToNewState(tSupervisorSystemState newState);
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
	tLogerCfg logCfg;
	//tUartStreamConfig uartStreamCfg;
	tLabViewUartStreamConfig lvUartStreamCfg;
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
	//inicjuje moduł radioodbornika
	controler.radio=Radio_Init(8);
	controler.radioMeasuremenets = Radio_GetChannelMeasurements();
	//aktywuje timer od pomiarów z radioodbiornika
	HAL_TIM_Base_Start(&htim10);
	HAL_TIM_IC_Start_IT(&htim10,TIM_CHANNEL_1);
	//inicjuje moduł logera
	logCfg.dtms=100;
	logCfg.maxNumberOfParams=30;
	logCfg.memoryPoolSize=0;

	/*uartStreamCfg.bufferPtr=0;
	uartStreamCfg.bufferSize = 512;
	uartStreamCfg.huart = controler.huart;*/

	lvUartStreamCfg.bufferPtr=0;
	lvUartStreamCfg.bufferSize = logCfg.maxNumberOfParams*4+7;
	lvUartStreamCfg.huart = controler.huart;
	//Loger_Create(&controler.loger,&logCfg,UartLogStreamer_Init(&uartStreamCfg));
	Loger_Create(&controler.loger,&logCfg,LabViewUartStreamer_Init(&lvUartStreamCfg));

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
	//odczytuje wskaxniki do struktury z parametrami pomiarowymi silników
	controler.leftMotorMeasurement = MotorInterface_GetMeasurements(controler.leftMotor);
	controler.rightMotorMeasurement = MotorInterface_GetMeasurements(controler.rightMotor);
	//inicjuje moduł LEDIndicator
	LedIndicator_Init(&controler.ledIndicator);
	//inicjuje parametry wewnętrzne
	controler.supervisorMsgQueue = xQueueCreate(20,sizeof(tSupervisorMsg));
	controler.supervisor.state = eSystem_InternalInit;
	controler.supervisor.leftMotorActive=0;
	controler.supervisor.rightMotorActive=0;
	//tworze wątek kontrolera
	xTaskCreate(Controler_Task,"controller",256,&controler,5,&controler.task);
	//tworzę wątek zarządzający pracą systemu
	xTaskCreate(Controler_SupervisorTask,"supervisor",128,&controler,4,&controler.supervisorTask);
	return 0;
}
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  W�tek kontrolera ruchu
  * @param[in]  None
  * @retval None
  */
void Controler_Task(void* ptr){
	int cnt=0;
	//dodaje parametry do logowania
	Loger_AddParams(controler.loger,&controler.mmpu.rpy[0],"roll",eParamTypeSGL);
	Loger_AddParams(controler.loger,&controler.mmpu.rpy[1],"pitch",eParamTypeSGL);
	Loger_AddParams(controler.loger,&controler.mmpu.rpy[2],"yaw",eParamTypeSGL);
	Loger_AddParams(controler.loger,&controler.mmpu.omega[0],"gyro_x",eParamTypeSGL);
	Loger_AddParams(controler.loger,&controler.mmpu.omega[1],"gyro_y",eParamTypeSGL);
	Loger_AddParams(controler.loger,&controler.mmpu.omega[2],"gyro_z",eParamTypeSGL);
	Loger_AddParams(controler.loger,&controler.mmpu.acceleration[0],"acc_x",eParamTypeSGL);
	Loger_AddParams(controler.loger,&controler.mmpu.acceleration[1],"acc_y",eParamTypeSGL);
	Loger_AddParams(controler.loger,&controler.mmpu.acceleration[2],"acc_z",eParamTypeSGL);
	/*Loger_AddParams(controler.loger,&controler.leftMotorMeasurement->voltage,"leftMotor_Voltage",eParamTypeSGL);
	Loger_AddParams(controler.loger,&controler.rightMotorMeasurement->voltage,"rightMotor_Voltage",eParamTypeSGL);*/
	//Loger_AddParams(controler.loger,&controler.leftMotorMeasurement->current,"leftMotor_current",eParamTypeSGL);
	//Loger_AddParams(controler.loger,&controler.rightMotorMeasurement->current,"rightMotor_current",eParamTypeSGL);
	/*Loger_AddParams(controler.loger,&controler.radioMeasuremenets[Channel_LeftVertical],"radio_left_v",eParamTypeU32);
	Loger_AddParams(controler.loger,&controler.radioMeasuremenets[Channel_LeftHorizontal],"radio_left_h",eParamTypeU32);
	Loger_AddParams(controler.loger,&controler.radioMeasuremenets[Channel_RightVertical],"radio_right_v",eParamTypeU32);
	Loger_AddParams(controler.loger,&controler.radioMeasuremenets[Channel_RightHorizontal],"radio_right_h",eParamTypeU32);*/
	//uruchamiam licznik do przechwytywania zdażeń od MPU
	HAL_TIM_Base_Start(&htim12);
	HAL_TIM_IC_Start_IT(&htim12,TIM_CHANNEL_1);
	//czekam na inicjację czujnika i pojawienie się pomiarów
	while(MPU6050_GetMeasurement(controler.hmpu,&controler.mmpu,200)!=0){
		vTaskDelay(200);
	}
	//przechodzę do inicjacji silników
	Supervisor_SwitchToNewState(eSystem_MotorInit);
	//uruchamiam loger
	Loger_OpenSesion(controler.loger);
	while(1){
		if(MPU6050_GetMeasurement(controler.hmpu,&controler.mmpu,200)==0){
			//odebrano nowy pomiar
		}else{
			//nie odebrano pomiaru z MPU, zgłaszam problem
			Supervisor_NewMsg(eSupervisorMPUSensorNoTrigger);
		}
		//wyknuje operacje zgodnie ze stanem w którym się znajduje
		switch(controler.supervisor.state){
		case eSystem_InternalInit:
			break;
		case eSystem_MotorInit:/*<tryb inicjacji silników i oczekiwania na przełączenie w tryb pracy aktywnej*/
			break;
		case eSystem_ReadyToWork:/*<tryb aktywnej pracy silników bez stabilizacji robota*/
			cnt++;
			if(cnt>=2){
				//wysyłam sterownaie na CAN
				MotorInterface_UpdateControl(controler.leftMotor,Radio_GetValue(Channel_LeftVertical));
				MotorInterface_UpdateControl(controler.rightMotor,Radio_GetValue(Channel_LeftVertical));
				cnt=0;
			}
			break;
		case eSystem_RobotStabilisation:/*<tryb pełnej stabilizacji robota*/
			break;
		case eSystem_FaultState:/*<awaria podsystemu czujników lub silników*/
			break;
		}
	}
}
/**
  * @brief  Wątek procesu nadzorującego stany pracy całego systemu
  * @param[in]  None
  * @retval None
  */
void Controler_SupervisorTask(void* ptr){
	tSupervisorMsg msg;
	while(1){
		//odbieram rozkazy
		if(xQueueReceive(controler.supervisorMsgQueue,&msg,200)==pdTRUE){
			switch(msg){
			case eSupervisorLeftMotorInactive:/*<lewy silnik w trybie nieaktywnym*/
				controler.supervisor.leftMotorActive=0;
				//sprawdzam, czy prawy silnik jest w trybie aktywnym
				if(controler.supervisor.rightMotorActive){
					//pravy silnik jest aktywny, więc go deaktywuje
					MotorInterface_SetMode(controler.rightMotor,eInactiveMode);
					//ustawiam stan głownej maszyny stanowej
					Supervisor_SwitchToNewState(eSystem_MotorInit);
				}else{
					//przełączam się do trybu aktywacji silników
					Supervisor_SwitchToNewState(eSystem_MotorInit);
				}
				break;
			case eSupervisorRightMotorInactive:/*<prawy silnik w trybie nieaktywnym*/
				controler.supervisor.rightMotorActive=0;
				//sprawdzam, czy lewy silnik jest w trybie aktywnym
				if(controler.supervisor.leftMotorActive){
					//pravy silnik jest aktywny, więc go deaktywuje
					MotorInterface_SetMode(controler.leftMotor,eInactiveMode);
					//ustawiam stan głownej maszyny stanowej
					Supervisor_SwitchToNewState(eSystem_MotorInit);
					//czekam
					vTaskDelay(1000);
				}else{
					//przełączam się do trybu aktywacji silników
					Supervisor_SwitchToNewState(eSystem_MotorInit);
				}
				break;
			case eSupervisorLeftMotorActive:/*<lewy silnik w trybie aktywnym*/
				controler.supervisor.leftMotorActive=1;
				//sprawdzam, czy prawy silnik jest aktywny
				if(controler.supervisor.rightMotorActive){
					//oba silniki są w trybie aktywnym, przełączam tryb pracy
					Supervisor_SwitchToNewState(eSystem_ReadyToWork);
				}
				break;
			case eSupervisorRightMotorActive:/*<prawy silnik w trybie aktywnym*/
				controler.supervisor.rightMotorActive=1;
				//sprawdzam, czy prawy silnik jest aktywny
				if(controler.supervisor.leftMotorActive){
					//oba silniki są w trybie aktywnym, przełączam tryb pracy
					Supervisor_SwitchToNewState(eSystem_ReadyToWork);
				}
				break;
			case eSupervisorMPUSensorNoTrigger:/*<Czujnik MPU przestał zgłaszac pomiiary na linii INT*/
				//wyłaczam silniki
				Supervisor_SwitchToNewState(eSystem_FaultState);
				break;
			}
		}
	}
}
/**
  * @brief  Funkcja zgłasza nowe zdażenie dla modułu nadzorującego pracę systemu
  * @param[in]  None
  * @retval None
  */
void Supervisor_NewMsg(tSupervisorMsg msg){
	xQueueSend(controler.supervisorMsgQueue,&msg,40);
}
/**
  * @brief  Funkcja realizuje rządanie przełaczenia trybu pracy maszyny stanowej
  * @param[in]  None
  * @retval None
  */
void Supervisor_SwitchToNewState(tSupervisorSystemState newState){
	if(newState==controler.supervisor.state){
		//stan nie uległ zmianie
		return;
	}
	switch(newState){
	case eSystem_InternalInit:
		break;
	case eSystem_MotorInit:/*<tryb inicjacji silników i oczekiwania na przełączenie w tryb pracy aktywnej*/
		MotorInterface_SetMode(controler.leftMotor,eActiveMode);
		MotorInterface_SetMode(controler.rightMotor,eActiveMode);
		LedIndicator_SetState(controler.ledIndicator,eLedIndicator_MotorInit);
		break;
	case eSystem_ReadyToWork:/*<tryb aktywnej pracy silników bez stabilizacji robota*/
		LedIndicator_SetState(controler.ledIndicator,eLedIndicator_ReadyToWork);
		break;
	case eSystem_RobotStabilisation:/*<tryb pełnej stabilizacji robota*/
		LedIndicator_SetState(controler.ledIndicator,eLedIndicator_RobotStabilisation);
		break;
	case eSystem_FaultState:/*<awaria podsystemu czujników lub silników*/
		MotorInterface_SetMode(controler.leftMotor,eInactiveMode);
		MotorInterface_SetMode(controler.rightMotor,eInactiveMode);
		LedIndicator_SetState(controler.ledIndicator,eLedIndicator_FaultState);
		break;
	}
	//ustawiam nowy stan
	controler.supervisor.state = newState;
}
/**
  * @brief  Funkcja callBack z modułu interfejsu silników zgłasz wzmianę stanu pracy sterowników silników
  * @param[in]  None
  * @retval None
  */
void MotorInterface_NewMotorState(tMotorInterfaceHandler h,tMotorInterfaceMode mode){
	if(h == controler.leftMotor){
		//lewy silnik
		if(mode == eInactiveMode){
			Supervisor_NewMsg(eSupervisorLeftMotorInactive);
		}else{
			Supervisor_NewMsg(eSupervisorLeftMotorActive);
		}
	}else if(h == controler.rightMotor){
		//prawy silnik
		if(mode == eInactiveMode){
			Supervisor_NewMsg(eSupervisorRightMotorInactive);
		}else{
			Supervisor_NewMsg(eSupervisorRightMotorActive);
		}
	}else{
		//bład
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
/**
  * @brief  Przerwanie od licznka TIM8 na potrzeby prowadzenia ststystyk OS
  * @param[in]  None
  * @retval None
  */
void TIM8_UP_TIM13_IRQHandler(void)
{
	__HAL_TIM_CLEAR_IT(&htim8, TIM_IT_UPDATE);
	ulHighFrequencyTimerTicks++;
}
/**
  * @brief Funkcja przerwania od kanału licznika TIM10
  * Funkcja przenosi informacje z kanału pomiarowego podłącoznego do radioodbiornika
  * @param[in]  None
  * @retval None
  */
void TIM1_UP_TIM10_IRQHandler(void){
	__HAL_TIM_CLEAR_IT(&htim10, TIM_IT_CC1);
	Radio_Update(controler.radio,TIM10->CCR1);
}
/**
  * @brief
  * @param[in]  None
  * @retval None
  */
void SetupRunTimeStatsTimer(void){
	HAL_TIM_Base_Start_IT(&htim8);
}
