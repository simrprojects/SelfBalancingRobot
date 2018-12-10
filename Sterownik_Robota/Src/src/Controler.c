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
#include <OsUART.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "MotorInterface.h"
#include "MotorInterfaceUart.h"
#include "MPU/rx_data.h"
#include "MPU/MPU6050.h"
#include "OSCan.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "UartLogStreamer.h"
#include "LabViewUartStreamer.h"
#include "LedIndicator.h"
#include "Radio.h"
#include "arm_math.h"
#include "LinearModules.h"
#include "LipoGuard.h"

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

	tMotorInterfaceUartHandler leftMotor;
	tMotorInterfaceUartHandler rightMotor;
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
	tLipoGuardHandler lipoGuard;
	struct{
		int leftMotorActive:1;
		int rightMotorActive:1;
		tSupervisorSystemState state;
		float voltage;
	}supervisor;
	struct{
		float k_pitch;/*<espółczynnik wzmocenienia od odchyłki kąt*/
		float k_dpitch;/*<współczynniki wzmocnienia od odchyłki prędkości kątowej*/
		float k_ipitch;/*<współczynnik wzmocnienia od odchyłki całki kąta*/
		float e_pitch;
		float e_dpitch;
		float e_ipitch;
		float cv;
		tIntegrator pitch_integrator;
	}pid;
	struct{
		signed short lastAngleLeftMotor;
		signed short lastAngleRightMotor;
		float leftWheelAngle;
		float rightWheelAngle;
		tIntegrator integrator;
		float k_w;/**<współczynnik wzmocenienia od odchyłi prędkości*/
		float k_a;/**<współczynnik wzmocenieina od odchyłki kąta*/
		float max_cv_a;/**<maksymalny zakres sterowania od odchyłki kąta*/
		float ctrl2w;/**<współczynnik skalujący sterowanie aparatury na prędkośc kątową*/
		float log_e_w;
		float log_e_a;
		float log_yaw_cv;
		float log_lMotor_u;
		float log_rMotor_u;
		float log_w;/**<prędkośc kaowa wokol osi yaw wylicozna na podstawie pomiarow silnikow*/
	}yawController;
	struct{
		float k_v;/**<współczynnik wzmocnienia od odchyłki prędkości*/
		float k_s;/**<wspólczynnik wzmocnienia od odchyłki drogi*/
		float max_e_s;/**<maksymalna wartośc odchyłki drogi*/
		float ctrl2v;/**<wspólczynik skalujacy z wartosci aparatury na predkosc m/s*/
		tIntegrator integrator;
	}velocityController;
	struct{
		float r;/**<promień koła*/
		float L;/**<rozstaw kół*/
		float kw;/**<współczynnik stałej elektrycznej silnika V/rad/s*/
		float h;/**<wysokośc środka ciężkości robota*/
	}robotParams;
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
void Controler_ControlProcesUpdate(void);
void Supervisor_NewMsg(tSupervisorMsg msg);
void Supervisor_SwitchToNewState(tSupervisorSystemState newState);
float Controler_PitchControlLoop(float pitch_sp,float omegaZ_sp);
void Controler_YawControlLoop(float omega_yaw_sp,float pitch_cv,int *leftMotor,int *rightMotor);
float Controler_VelocityControlLoop(float velocity_sp);
float Controler_RadioToAngle(signed int radioChValue);
float Controler_RadioToOmega(signed int radioChValue);
float Controler_RadioToVelocity(signed int radioChValue);
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
	tMotorInterfaceUartConfig mic;
	//tLogerCfg logCfg;
	//tUartStreamConfig uartStreamCfg;
	//tLabViewUartStreamConfig lvUartStreamCfg;
	//inicjuje modu� CAN
	/*cfg.rxBufferSize=30;
	OSCan_Init(&hcan1,&cfg);
	//inicjuje modu� UART2CAN
	c2uc.canFifoNumber=1;
	c2uc.maxNumberOfChannels=3;
	if(CAN2UART_Init(&controler.c2uf,&c2uc)){
		return 1;
	}*/

	//inicjuje UART
	/*if(OsUART_Init(&controler.huart,&huart1)){
		return 4;
	}*/

	//inicjuje moduł radioodbornika
	controler.radio=Radio_Init(8);
	controler.radioMeasuremenets = Radio_GetChannelMeasurements();
	//aktywuje timer od pomiarów z radioodbiornika
	HAL_TIM_Base_Start(&htim10);
	HAL_TIM_IC_Start_IT(&htim10,TIM_CHANNEL_1);
	//aktywuje timer do generowania sygnalow ostrzegawczych
	HAL_TIM_Base_Start(&htim8);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
	//inicjuje moduł logera
	/*logCfg.dtms=20;
	logCfg.maxNumberOfParams=40;
	logCfg.memoryPoolSize=0;*/

	/*uartStreamCfg.bufferPtr=0;
	uartStreamCfg.bufferSize = 512;
	uartStreamCfg.huart = controler.huart;*/

	/*lvUartStreamCfg.bufferPtr=0;
	lvUartStreamCfg.bufferSize = logCfg.maxNumberOfParams*4+7;
	lvUartStreamCfg.huart = controler.huart;*/
	//Loger_Create(&controler.loger,&logCfg,UartLogStreamer_Init(&uartStreamCfg));
	//Loger_Create(&controler.loger,&logCfg,LabViewUartStreamer_Init(&lvUartStreamCfg));
	controler.loger=0;

	//inicjuje modu� interfejsu silnik�w
	mic.numPolePairs = 15;
	mic.reversMode = 0;
	mic.uart = &huart1;
	if(MotorInterfaceUart_Init(&controler.leftMotor,&mic)){
		return 2;
	}
	mic.numPolePairs = 15;
	mic.reversMode = 1;
	mic.uart = &huart6;
	if(MotorInterfaceUart_Init(&controler.rightMotor,&mic)){
		return 3;
	}
	//inicjuje lipoGuarda - moduł ochrony pakietu
	tLipoGuardConfig lgCfg;
	lgCfg.numOfCells=4;
	lgCfg.refreshRate = 40;//40Hz
	lgCfg.warningLevel=3.4f;//napięcie ostrzegawcze na jedną cele
	lgCfg.errorLevel = 3.2f;//poziom napięcia informujący o konieczności naładowania pakietu
	lgCfg.histeresis = 0.15f;//histereza zapobiega częstym przełączeniom w okolicach poziomów granicznych
	controler.lipoGuard = LipoGuard_Init(&lgCfg);
	//wysyłam komendę deinicjującą
	//MotorInterface_SetMode(controler.leftMotor,eInactiveMode);
	//MotorInterface_SetMode(controler.rightMotor,eInactiveMode);
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
	controler.supervisor.voltage=14.5f;

	controler.pid.k_pitch=2500.f/45.f*180.f/PI;//przelicznik wzmocnienia proporcjonalnego. odchylenie o 45 stopni powoduje wysterowanie silników na 50%
	controler.pid.k_dpitch=500.f/100.f*180.f/PI;//wzmocnenie uchybu predkości: 20% wysterowania przy 100stopniach/sekundę
	controler.pid.k_ipitch=0;//wzmocnienie uchybu całki odchyłki kąt
	Integrator_SetTime(&controler.pid.pitch_integrator,0.005);

	controler.robotParams.r=0.197f/2;
	controler.robotParams.L=0.4f;
	controler.robotParams.kw=12.f/(300.f/60.f*2.f*PI);
	controler.robotParams.h=0.2;

	controler.yawController.k_w=1.5;
	controler.yawController.k_a=1.3;
	controler.yawController.max_cv_a=30.f/180.f*PI;
	controler.yawController.ctrl2w=0.5f*2.f*PI/1000.f;/**<współczynnik skalujący aparatura do w_yaw */
	Integrator_SetTime(&controler.yawController.integrator,0.005);

	controler.velocityController.ctrl2v=1.0f/1000.f;
	controler.velocityController.k_s=5*PI/180;
	controler.velocityController.k_v=10*PI/180;
	controler.velocityController.max_e_s=1;
	Integrator_SetTime(&controler.velocityController.integrator,0.005);

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
	int lm,rm=0;
	float v;
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
	Loger_AddParams(controler.loger,&controler.leftMotorMeasurement->voltage,"leftMotor_Voltage",eParamTypeSGL);/*10*/
	Loger_AddParams(controler.loger,&controler.rightMotorMeasurement->voltage,"rightMotor_Voltage",eParamTypeSGL);
	Loger_AddParams(controler.loger,&controler.leftMotorMeasurement->current,"leftMotor_current",eParamTypeSGL);
	Loger_AddParams(controler.loger,&controler.rightMotorMeasurement->current,"rightMotor_current",eParamTypeSGL);
	Loger_AddParams(controler.loger,&controler.leftMotorMeasurement->rpm,"leftMotor_rpm",eParamTypeSGL);
	Loger_AddParams(controler.loger,&controler.rightMotorMeasurement->rpm,"rightMotor_rpm",eParamTypeSGL);
	Loger_AddParams(controler.loger,&controler.radioMeasuremenets[Channel_LeftVertical],"radio_left_v",eParamTypeI32);/*16*/
	Loger_AddParams(controler.loger,&controler.radioMeasuremenets[Channel_LeftHorizontal],"radio_left_h",eParamTypeI32);
	Loger_AddParams(controler.loger,&controler.radioMeasuremenets[Channel_RightVertical],"radio_right_v",eParamTypeI32);
	Loger_AddParams(controler.loger,&controler.radioMeasuremenets[Channel_RightHorizontal],"radio_right_h",eParamTypeI32);
	Loger_AddParams(controler.loger,&controler.pid.e_pitch,"pid_e_yaw",eParamTypeSGL);/*20*/
	Loger_AddParams(controler.loger,&controler.pid.e_dpitch,"pid_e_yaw",eParamTypeSGL);
	Loger_AddParams(controler.loger,&controler.pid.cv,"pid_cv",eParamTypeSGL);
	Loger_AddParams(controler.loger,&controler.yawController.log_e_a,"yawCtrl_ea",eParamTypeSGL);/*23*/
	Loger_AddParams(controler.loger,&controler.yawController.log_e_w,"yawCtrl_ew",eParamTypeSGL);
	Loger_AddParams(controler.loger,&controler.yawController.log_lMotor_u,"yawCtrl_u_lMotor",eParamTypeSGL);
	Loger_AddParams(controler.loger,&controler.yawController.log_rMotor_u,"yawCtrl_u_rMotor",eParamTypeSGL);
	Loger_AddParams(controler.loger,&controler.yawController.log_w,"yawCtrl_w_yaw",eParamTypeSGL);
	Loger_AddParams(controler.loger,&controler.yawController.log_yaw_cv,"yawCtrl_cv",eParamTypeSGL);
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
		//wykonuje operacje zgodnie ze stanem w którym się znajduje
		switch(controler.supervisor.state){
		case eSystem_InternalInit:
			break;
		case eSystem_MotorInit:/*<tryb inicjacji silników i oczekiwania na przełączenie w tryb pracy aktywnej*/
			break;
		case eSystem_ReadyToWork:/*<tryb aktywnej pracy silników bez stabilizacji robota*/
			v=Controler_VelocityControlLoop(Controler_RadioToVelocity(-Radio_GetValue(Channel_LeftVertical)));
			//pitch=Controler_PitchControlLoop(-v,0);
			//pitch=Controler_PitchControlLoop(Controler_RadioToAngle(Radio_GetValue(Channel_LeftVertical)),0);
			//pitch = (float)Radio_GetValue(Channel_LeftVertical)/1000.f*controler.supervisor.voltage;
			//Controler_YawControlLoop(Controler_RadioToOmega(Radio_GetValue(Channel_LeftHorizontal)),pitch,&lm,&rm);
			Controler_YawControlLoop(Controler_RadioToOmega(Radio_GetValue(Channel_LeftHorizontal)),v,&lm,&rm);

			/*cnt++;
			if(cnt>=2){*/
				//wysyłam sterownaie na CAN
				MotorInterfaceUart_UpdateControl(controler.leftMotor,lm);
				MotorInterfaceUart_UpdateControl(controler.rightMotor,rm);
			/*	cnt=0;
			}*/
			//uaktualniam lipoGuarda
			LipoGuard_Update(controler.lipoGuard,(controler.leftMotorMeasurement->voltage+controler.rightMotorMeasurement->voltage)/2.f);
			break;
		case eSystem_RobotStabilisation:/*<tryb pełnej stabilizacji robota*/
			break;
		case eSystem_FaultState:/*<awaria podsystemu czujników lub silników*/
			break;
		}
	}
}
/**
  * @brief  Główna funkcja kontrolera
  * @param[in]  pitch_sp: zadany kąt przechyłu wyrazony w radianach
  * @param[in]  omegaZ_sp: zadana prędkośc obracania się robota względem osi Z [rad/s]
  * @retval None
  */
float Controler_PitchControlLoop(float pitch_sp,float omegaZ_sp){
	float cv,e_pitch,e_dpitch,e_ipitch;
	//static int cnt=0;
	static signed char fb=0;
#define MAX_CV_OUT	900

	e_pitch = pitch_sp-controler.mmpu.rpy[1];
	e_dpitch = -controler.mmpu.omega[0];
	e_ipitch = Integrator_Execute(&controler.pid.pitch_integrator,fb,e_pitch);
	controler.pid.e_pitch = e_pitch;
	controler.pid.e_dpitch = e_dpitch;
	controler.pid.e_ipitch = e_ipitch;
	cv = e_pitch*controler.pid.k_pitch+e_dpitch*controler.pid.k_dpitch+e_ipitch*controler.pid.k_ipitch;
	controler.pid.cv=cv;
	//ograniczam
	if(cv>MAX_CV_OUT){
		cv = MAX_CV_OUT;
		fb=1;
	}else if(cv<-MAX_CV_OUT){
		cv = -MAX_CV_OUT;
		fb=-1;
	}else{
		fb=0;
	}
	//skaluje sterowanie na napięcie
	cv = cv*controler.supervisor.voltage/1000;
	//ustwiam wartości sterujące
	/*if(cnt==0){
		MotorInterface_UpdateControl(controler.leftMotor,cv);
		MotorInterface_UpdateControl(controler.rightMotor,cv);
	}
	cnt++;
	cnt%=2;*/
	return cv;
}
/**
  * @brief  Funkcja regulatora prędkosci pionowej segwaya. Funkcja oddziałuje bezposrednio na silniki
  * zapewniając odpowiednie sterowanie na każdym silniku.
  * @param[in]  omega_yaw_sp: wartośc zadana prędkosci obrotowej wokół osi Yaw(pionowej)
  * @param[in]  pitch_cv: wartosc sterująca wypracowana przez regulator kąta Pitch wyrażona w voltah
  * @retval None
  */
void Controler_YawControlLoop(float omega_yaw_sp,float pitch_cv,int *leftMotor,int *rightMotor){
	float /*dAngleLeft,dAngleRight,*/wwl,wwr,wp,ew,cv_w,cv_a,cv,dU1,dU2,ul,up;
	static signed char fb=0;

	//odczytuje przyrost kąta i skaluje go do rdianów
	/*dAngleLeft = (controler.leftMotorMeasurement->angle-controler->yawController.lastAngleLeftMotor)*2.f*PI/(6*15);
	controler->yawController.lastAngleLeftMotor=controler.leftMotorMeasurement->angle;
	controler->yawController.leftWheelAngle+=dAngleLeft;
	//ograniczam zakres zmian kąta do +- PI
	if(controler->yawController.leftWheelAngle>PI){
		controler->yawController.leftWheelAngle-=PI;
	}else if(controler->yawController.leftWheelAngle<PI){
		controler->yawController.leftWheelAngle+=PI;
	}*/
	//odczytuje prędkości kół i skaluje do rad/s
	wwl = controler.leftMotorMeasurement->rpm/60.f*2*PI;
	wwr = controler.rightMotorMeasurement->rpm/60.f*2*PI;
	//przeliczam to na prędkośc platformy
	wp=controler.robotParams.r/controler.robotParams.L*(wwr-wwl);
	controler.yawController.log_w = wp;
	//obliczam ochyłkę
	ew = omega_yaw_sp-wp;
	controler.yawController.log_e_w=ew;
	//wyznaczam sterowanie
	cv_w = ew*controler.yawController.k_w;
	cv_a = Integrator_Execute(&controler.yawController.integrator,fb,ew);
	controler.yawController.log_e_a=cv_a;
	//ograniczam zakres oddziaływania kąta
	if(cv_a>controler.yawController.max_cv_a){
		cv_a = controler.yawController.max_cv_a;
		fb = 1;
	}else if(cv_a<-controler.yawController.max_cv_a){
		cv_a = -controler.yawController.max_cv_a;
		fb = -1;
	}else{
		fb=0;
	}
	//uwzględniam współczynnik intensywności oddziaływania sterowania od odchyłki kąta
	cv_a*=controler.yawController.k_a;
	//wyznaczam całkowite wysterowanie
	cv = cv_a+cv_w+omega_yaw_sp;
	controler.yawController.log_yaw_cv=cv;
	//skaluje sterowanie na różnice napięc Up-Ul=dU1=w*L*kw/r
	dU1 = cv*controler.robotParams.L/controler.robotParams.r*controler.robotParams.kw;
	//sterowanie od wychyłu
	dU2 = 2*pitch_cv;
	//wyznaczam zadane napiecia
	up = 0.5f*(dU2+dU1);
	ul = 0.5f*(dU2-dU1);
	controler.yawController.log_lMotor_u=ul;
	controler.yawController.log_rMotor_u=up;
	//przeskalowuje zadane napiecia na wysterowanie silnika +-1000
	*leftMotor=ul/controler.supervisor.voltage*1000;
	*rightMotor=up/controler.supervisor.voltage*1000;
}
/**
  * @brief  Pętla sterująca prędkością segwaya. Pętla wyracowuje wartośc zadaną dla regulatora kąta
  * @param[in]  velocity_sp: prędkośc zadana [m/s]
  * @retval wartosc zadana kąta pitch
  */
float Controler_VelocityControlLoop(float velocity_sp){
	float v,e_v,e_s,cv;
	signed char fb=0;
	//obliczam prędkośc robota na podstawie pomiarow predkosci koł
	v = 0.5*(controler.leftMotorMeasurement->rpm+controler.rightMotorMeasurement->rpm)*2.f*PI/60.f*controler.robotParams.r;
	//wyznaczam korektę w celu określenia predkości srodka masy
	v += controler.mmpu.omega[0]*controler.robotParams.h*cosf(controler.mmpu.rpy[1]);
	//obliczam odchyłki
	e_v = velocity_sp-v;

	e_s = Integrator_Execute(&controler.velocityController.integrator,fb,e_v);
	//ograniczam odchyłkę
	if(e_s>controler.velocityController.max_e_s){
		e_s = controler.velocityController.max_e_s;
		fb=1;
	}else if(e_s<-controler.velocityController.max_e_s){
		e_s = -controler.velocityController.max_e_s;
		fb=-1;
	}else{
		fb=0;
	}
	cv = e_v*controler.velocityController.k_v+e_s*controler.velocityController.k_s;
	return cv;
}
/**
  * @brief  Funkcja przelicza sterowanie z aparatury +-1000 na radiany z odpowiednim współpczynnikiem skalującym
  * @param[in]  radioChValue: wartośc z kanału radiowego
  * @retval kąt wyrażony w radianach
  */
float Controler_RadioToAngle(signed int radioChValue){
	return (float)radioChValue/1000.f*15.f/180.f*PI;
}
/**
  * @brief  Funkcja przeskalowuje wartosci z aparatury na prędkośc kątową wyrażoną w rad/s
  * @param[in]  None
  * @retval None
  */
float Controler_RadioToOmega(signed int radioChValue){
	return (float)radioChValue*controler.yawController.ctrl2w;
}
/**
  * @brief  Funkcja przeskalowuje wartosci z aparatury na prędkośc liniową m/s
  * @param[in]  None
  * @retval None
  */
float Controler_RadioToVelocity(signed int radioChValue){
	return (float)radioChValue*controler.velocityController.ctrl2v;
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
					MotorInterfaceUart_SetMode(controler.rightMotor,eInactiveMode);
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
					MotorInterfaceUart_SetMode(controler.leftMotor,eInactiveMode);
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
  * @brief  Funkcja obliczająca sterowanie na podstawie pomiarów i wartości zadanych
  * @param[in]  None
  * @retval None
  */
void Controler_ControlProcesUpdate(void){

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
		MotorInterfaceUart_SetMode(controler.leftMotor,eActiveMode);
		MotorInterfaceUart_SetMode(controler.rightMotor,eActiveMode);
		LedIndicator_SetState(controler.ledIndicator,eLedIndicator_MotorInit);
		break;
	case eSystem_ReadyToWork:/*<tryb aktywnej pracy silników bez stabilizacji robota*/
		LedIndicator_SetState(controler.ledIndicator,eLedIndicator_ReadyToWork);
		break;
	case eSystem_RobotStabilisation:/*<tryb pełnej stabilizacji robota*/
		LedIndicator_SetState(controler.ledIndicator,eLedIndicator_RobotStabilisation);
		break;
	case eSystem_FaultState:/*<awaria podsystemu czujników lub silników*/
		MotorInterfaceUart_SetMode(controler.leftMotor,eInactiveMode);
		MotorInterfaceUart_SetMode(controler.rightMotor,eInactiveMode);
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
  * @brief  Funkcja typu CallBack wywoływana przez moduł LipoGuard przy osiągnięciu nowego stanu pakietu
  * @param[in]  None
  * @retval None
  */
void LipoGuard_NewStateCallBack(tLipoGuardHandler h,tLipoGuardBateryState state){
	switch(state){
	case eLipoOk:
		TIM8->CCR3=0;
		break;
	case eLipoWarning:
		TIM8->CCR3=1000;//impuls 0.1s i okres 1s
		TIM8->ARR = 10000;
		break;
	case eLipoDischarged:
		TIM8->CCR3=3000;//impuls 0.3s i okres 0.5s
		TIM8->ARR = 5000;
		break;
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
