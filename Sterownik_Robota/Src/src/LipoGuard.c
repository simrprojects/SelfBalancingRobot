/**
  ******************************************************************************
  * @file	 LipoGuard.c
  * @author  Przemek
  * @version V1.0.0
  * @date    14.05.2018
  * @brief
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "LinearModules.h"
#include "LipoGuard.h"
/* Private typedef -----------------------------------------------------------*/

typedef struct{
	unsigned int numOfCells:4;/**<;iczba cel w pakiecie Lipo*/
	unsigned int internalState:2;/**<wewnętrzny stan maszyny stanowej urządzenia*/
	unsigned int newMeasurement:1;/**<flaga do monitorowania o nowym pomiarze napięcia*/
	float warningLevel;/**<poziom napięcia poniżej którego włączy się stan ostrzegawczy*/
	float errorLevel;/**<poziom napięcia ponizej którego włącza się tan alarmowy*/
	float histeresis;/**histereza napięcia*/
	unsigned int period;
	float newVoltage;/**<zmienna do przenoszenia informacji o nowym pomiarze napięcia*/
	xTaskHandle task;
	tDelay delay;
}tLipoGuard;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void LipoGuard_Task(tLipoGuardHandler h);
/* Public  functions ---------------------------------------------------------*/
/**
  * @brief  Funkjca inicjuje moduł LipoGuarda tj. modułu nadzowującego poziom napięcia na pakiecie
  * @param[in]  cfg: wskaźnik do struktury konfiguracyjnej
  * @retval uchwyt do obiektu lipoGuarda
  */
tLipoGuardHandler LipoGuard_Init(tLipoGuardConfig *cfg){
	tLipoGuard *lipo;
	//alokuje pamięc na modul
	lipo = pvPortMalloc(sizeof(tLipoGuard));
	//inijuje parametry wewnętrzne
	lipo->numOfCells=cfg->numOfCells;
	lipo->internalState=0;
	lipo->warningLevel=cfg->warningLevel*cfg->numOfCells;
	lipo->errorLevel=cfg->errorLevel*cfg->numOfCells;
	lipo->histeresis=cfg->histeresis;
	lipo->period=1000.f/cfg->refreshRate;
	lipo->newMeasurement=0;
	xDelay_Init(&lipo->delay,1.0f,1.f/cfg->refreshRate);
	//inicjuje task
	xTaskCreate(LipoGuard_Task,"LipoGuard",128,lipo,2,&lipo->task);
	return lipo;
}
/**
  * @brief  Funkcja przekazuje do modułu wartośc napiecia pakietu
  * @param[in]  h: uchwyt do obiektu
  * @param[in] voltage: napięcie na pakiecie
  * @retval None
  */
void LipoGuard_Update(tLipoGuardHandler h, float voltage){
	((tLipoGuard*)h)->newVoltage=voltage;
	((tLipoGuard*)h)->newMeasurement=1;
}
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Funkcja wątku lipoGuarda
  * @param[in]  None
  * @retval None
  */
void LipoGuard_Task(tLipoGuardHandler h){
	TickType_t time;
	tLipoGuard *lg = (tLipoGuard*)h;
	float fVoltage;

	time = xTaskGetTickCount();
	while(1){
		//przetwarzam stan maszyny stanowej
		switch(lg->internalState){
		case 0://stan początkowy w którym oczekuje na pierwsze uaktualnienie pomiaru napięcia
			if(lg->newMeasurement){
				//odebrałem nowy pomiar, sprawdzam czy poziom zmierzony jest akceptowalny
				if(lg->newVoltage>3.f){
					//uznaje, ze nowy pomiar jest ok, ustawiam filtr
					xDelay_SetState(&lg->delay,lg->newVoltage);
					//ustawiam nowy stan
					lg->internalState=1;
					//zgłaszam informacje o stanie pakietu
					LipoGuard_NewStateCallBack(lg,eLipoOk);
				}else{
					//resetuje flagę
					lg->newMeasurement=0;
				}
			}
			break;
		case 1://stan naładowanego pakietu
			//odfiltrowuje napięcie
			fVoltage=xDelay_Execute(&lg->delay,lg->newVoltage);
			//sprawdzam poziom napięcia
			if(fVoltage<lg->errorLevel){
				//przechodzę do stanu awaryjnego
				lg->internalState=3;
				//zgłaszam informacje o stanie pakietu
				LipoGuard_NewStateCallBack(lg,eLipoDischarged);
			}else if(fVoltage<lg->warningLevel){
				//przechodzę do stanu ostrzegawczego
				lg->internalState=2;
				//zgłaszam informacje o stanie pakietu
				LipoGuard_NewStateCallBack(lg,eLipoWarning);
			}
			break;
		case 2://stan ostrzegawczy
			//odfiltrowuje napięcie
			fVoltage=xDelay_Execute(&lg->delay,lg->newVoltage);
			//sprawdzam poziom napięcia
			if(fVoltage>(lg->warningLevel+lg->histeresis)){
				//przechodzę do stanu normalnego
				lg->internalState=1;
				//zgłaszam informacje o stanie pakietu
				LipoGuard_NewStateCallBack(lg,eLipoOk);
			}else if(fVoltage<lg->errorLevel){
				//przechodzę do stanu awaryjnego
				lg->internalState=3;
				//zgłaszam informacje o stanie pakietu
				LipoGuard_NewStateCallBack(lg,eLipoDischarged);
			}
			break;
		case 3://stan rozładowania pakietu
			//nie robię już nic, nie można opóścic tego stanu
			break;
		}
		vTaskDelayUntil(&time,lg->period);
	}
}
/**
  * @brief  Funkcja zgłaszana w momencie zmiany stanu naładowania pakietu
  * @param[in]  None
  * @retval None
  */
__weak void LipoGuard_NewStateCallBack(tLipoGuardHandler h,tLipoGuardBateryState state){

}
