/**
  ******************************************************************************
  * @file	 LedIndicator.c
  * @author  Przemek
  * @version V1.0.0
  * @date    06.04.2018
  * @brief   Moduł dostarcza funkconalności sygnalizoatora różnych stanów pracy robota
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"
#include "queue.h"
#include "gpio.h"
#include "LedIndicator.h"
/* Private typedef -----------------------------------------------------------*/
typedef struct{
	xTimerHandle timer;
	GPIO_TypeDef * gpio;
	unsigned short pin;
	char state;
}tLedControler;

typedef struct{
	tLedControler *led1;
	tLedControler *led2;
}tLedInidicator;


/* Private define ------------------------------------------------------------*/
#define HLI()	((tLedInidicator*)h)
#define HLC()	((tLedControler*)h)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
tLedControler* LedControler_Init(GPIO_TypeDef* gpio,unsigned short pin);
void LedControler_SetLedState(tLedControler* led,int constanOnOf);
void LedControler_SetLedBlinking(tLedControler* led,int period);
void LedControler_TimerTask(TimerHandle_t xTimer);
/* Public  functions ---------------------------------------------------------*/
/**
  * @brief  Funkcja inicjuje moduł
  * @param[in]  None
  * @retval None
  */
int LedIndicator_Init(tLedIndictorHandler *h){
	tLedInidicator * li;
	li = pvPortMalloc(sizeof(tLedInidicator));
	li->led1 = LedControler_Init(GPIOJ,LD_USER1_Pin);
	li->led2 = LedControler_Init(GPIOJ,LD_USER2_Pin);
	*h = li;
	return 0;
}
/**
  * @brief  Funkcja aktywuje wybraną sekwencję błyskową sygnalizującą stan pracy
  * @param[in]  None
  * @retval None
  */
int LedIndicator_SetState(tLedIndictorHandler h,tLedIndicatorState state){
	switch(state){
	case eLedIndicator_MotorInit:/*<tryb inicjacji silników i oczekiwania na przełączenie w tryb pracy aktywnej*/
		LedControler_SetLedState(HLI()->led2,0);
		LedControler_SetLedBlinking(HLI()->led1,200);
		break;
	case eLedIndicator_ReadyToWork:/*<tryb aktywnej pracy silników bez stabilizacji robota*/
		LedControler_SetLedState(HLI()->led1,1);
		LedControler_SetLedState(HLI()->led2,0);
		break;
	case eLedIndicator_RobotStabilisation:/*<tryb pełnej stabilizacji robota*/
		LedControler_SetLedState(HLI()->led1,1);
		LedControler_SetLedState(HLI()->led2,1);
		break;
	case eLedIndicator_FaultState:/*<awaria podsystemu czujników lub silników*/
		LedControler_SetLedBlinking(HLI()->led1,400);
		LedControler_SetLedBlinking(HLI()->led2,400);
		break;
	}
	return 0;
}
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Funkcja inicjuje instację modułu kontrolującego diodę
  * @param[in]  None
  * @retval None
  */
tLedControler* LedControler_Init(GPIO_TypeDef* gpio,unsigned short pin){
	tLedControler* led;
	led = pvPortMalloc(sizeof(tLedControler));
	led->gpio = gpio;
	led->pin = pin;
	led->state=0;
	//zeruje diode
	gpio->ODR &= ~pin;
	//alokuje timer
	led->timer = xTimerCreate("ledTimer",100,1,led,LedControler_TimerTask);
	xTimerStop(led->timer,0);
	return led;
}
/**
  * @brief  Funkcja włącza lub wyłącza diodę LED
  * @param[in]  None
  * @retval None
  */
void LedControler_SetLedState(tLedControler* led,int constanOnOf){
	xTimerStop(led->timer,0);
	led->state=constanOnOf;
	if(constanOnOf){
		led->gpio->ODR |= led->pin;
	}else{
		led->gpio->ODR &= ~led->pin;
	}
}/**
  * @brief  Funkcja ustawia miganie diody LED
  * @param[in]  None
  * @retval None
  */
void LedControler_SetLedBlinking(tLedControler* led,int period){
	LedControler_SetLedState(led,0);
	xTimerStart(led->timer,period);
	xTimerChangePeriod(led->timer,period,period);
	led->state=0;
}
/**
  * @brief  funkcja callback wywoływana prze timer OS
  * @param[in]  None
  * @retval None
  */
void LedControler_TimerTask(TimerHandle_t xTimer){
	tLedControler* led = (tLedControler*)pvTimerGetTimerID(xTimer);
	if(led->state==0){
		led->state=1;
		led->gpio->ODR |= led->pin;
	}else{
		led->state=0;
		led->gpio->ODR &= ~led->pin;
	}
}

