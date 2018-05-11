/**
  ******************************************************************************
  * @file		 Radio.h
  * @author  PS
  * @version V1.0.0
  * @date    07.02.2017
  * @brief   Modul wspierajacy obsługę odbiornika radiowego modelarskiego
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "Radio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "tim.h"
#include "Loger.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef struct{
	unsigned short *value;
	signed int *convertedValue;
	int lostConnection;
	int numberOfSerialChannels;
	xTaskHandle task;
	QueueHandle_t queue;
}tRadio;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
tRadio *radio;
/* Private function prototypes -----------------------------------------------*/
void RadioScaleChannels(tRadio *radio);
void Radio_ChCallBack(void *ptr,unsigned int time,int pinState);
void Radio_Task(void* ptr);
void Radio_LogHederDescription(char* buffer,int bufSize,int* size);
void Radio_LogData(char* buffer,int bufSize,int* size);
/* Public function -----------------------------------------------------------*/
/**
  * @brief  Funkcja inicjuje modul odbiornika szeregowego RC
  * @param  None
  * @retval None
  */
void* Radio_Init(int numberOfChannels){
	tRadio *r;
	//przydzielam pamiec na obiekt
	r=pvPortMalloc(sizeof(tRadio));
	radio = r;
	r->numberOfSerialChannels=numberOfChannels;
	r->convertedValue = pvPortMalloc(sizeof(int)*numberOfChannels);
	r->value = pvPortMalloc(sizeof(unsigned short)*numberOfChannels);
	for(int i=0;i<numberOfChannels;i++){
		r->convertedValue[i]=0;
		r->value[i]=1500;
	}
	//kolejka
	r->queue=xQueueCreate(50,sizeof(int));
	//watek radioodbiornika
	xTaskCreate( Radio_Task, "Radio", 256, r, 3, &r->task );
	return r;
}
/**
  * @brief  
  * @param  None
  * @retval None
  */
signed int Radio_GetValue(int channel){
	if(channel<radio->numberOfSerialChannels){
		return radio->convertedValue[channel];
	}else{
		return 0;
	}
}
/**
  * @brief  
  * @param  None
  * @retval None
  */
unsigned short Radio_GetPwmValue(int channal){
	if(channal<radio->numberOfSerialChannels){
		return radio->value[channal];
	}else{
		return 0;
	}
}
/**
  * @brief  Funkcja zwraca refernecję do tablicy pomiarów po konwersji
  * @param[in]  None
  * @retval None
  */
int* Radio_GetChannelMeasurements(void){
	return radio->convertedValue;
}
/**
  * @brief  Funkcja dokonuje analizy rejestrowanych czasow zboczy i zamienia je na informacje
	* 				przyporzadkowana odpowiednim kanalom
  * @param  time - czas wykrycia zbocza
  * @retval None
  */
unsigned short radio_buf[64];
int rcnt=0;
void Radio_Update(void* handler,unsigned int time){
	tRadio *radio = (tRadio*)handler;
	static char firstTrigger = 0; //Znacznik pierwszego wyzwolenia timera
	static unsigned char channelCounter = 0; //Licznik kanalow
	static unsigned short lastTrigger = 0; //Zmienna przechowujaca wartosc zatrzasnieta w poprzednim wyzwoleniu
	unsigned short capture = 0; //Zmienna przechowujaca wartosc zmierzonego czasu
  static unsigned char frameComplete;	
	
		if (firstTrigger == 0) { //Sprawdzamy czy pierwsze wyzwolenie
			lastTrigger = time;
			firstTrigger = 1;
		} else { //Jezeli jest to kolejne wyzwolenie przeliczamy czas pomiedzy aktualnym a poprzednim
            capture = ((unsigned short)time - lastTrigger);


			if (capture > 4000) { //Jezeli wykryto impuls synchronizacji	
                //w odbiorniku HOTT mozna wykryc utrate nadajnika po dlugosci impulsu synchronizujacego
                if(capture>15000){
                    //wykryto utrate poloczenia
                    //radio->lostConnection=1;
                	radio_buf[rcnt++]=capture;
                	rcnt&=0x3F;
                }else{
                    //poprawny impuls synchronizujacy ramke
                    if(frameComplete){
                        //ustawiam flage informujaca o poprawnym poloczeniu dopiero po odebraniu kompletnej ramki
                        radio->lostConnection=0;
                    }
                    //ustawiam liczbe kanalow
                    //radio->numberOfSerialChannels = channelCounter;
                }
                frameComplete = 0;
                channelCounter = 0;
			} else { //Jezeli zlapany impuls jest dluzszy niz 500 jednostek, wtedy traktuj go jako wartosc kanalu
                //zabezpieczenie przed odbiornikiem o zbyt duzej liczbie kanalow
                if(channelCounter<radio->numberOfSerialChannels){
                    if(capture>700&&capture<2300){                
                        radio->value[channelCounter++] = capture;
                        //ustawiam flage informujaca o kompletnej ramce
                        if(channelCounter>=radio->numberOfSerialChannels){
                            frameComplete = 1;
														//konwertuje 
														//RadioScaleChannels(radio);
                        }
                    }
                }
			}
			lastTrigger = time;
		}
}
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Watek odpowiedzialny za anlie sygnalu szeregowego odbiornika
  * @param  None
  * @retval None
  */
void Radio_Task(void* ptr){
//	unsigned int time;
	tRadio *radio=(tRadio*)ptr;

	while(1){
//		if(xQueueReceive(radio->queue,&time,40)==pdTRUE){
//			Radio_Update(radio,time);
//		}
		vTaskDelay(40);
		RadioScaleChannels(radio);
	}
}

/**
  * @brief  Funkcja dokonuje przeskalowania wartosci odczytanych z odbiornika na wartosci w zakresie -1000 - 1000
	*         dla pierwszych czterech kanalow i od 0 do 1000 dla pozostaBych kanalow
  * @param  None
  * @retval None
  */
void RadioScaleChannels(tRadio *radio){
	signed int v;
#define dead_zone 20
	//pierwsze cztery kanaly skalowane sa od -1000 do 1000
	for(int i=0;i<4;i++){
		v = radio->value[i];
		//ograniczam
		if(v<1000){
			v = 1000;
		}else if(v>2000){
			v = 2000;
		}
		v=(v-1500)*2;
		//strefa martwa
		if(v>dead_zone){
			v = v-dead_zone;
		}else if(v>-dead_zone){
			v = 0;
		}else{
			v = v+dead_zone;
		}
		radio->convertedValue[i]=v;

	}
	//kolejne kanaly skalowane sa od 0 do 1000
	//to do
}
void Radio_ChCallBack(void *ptr,unsigned int time,int pinState){
	BaseType_t xHigherPriorityTaskWoken;

	// We have not woken a task at the start of the ISR.
	xHigherPriorityTaskWoken = pdFALSE;

		//wysylam wartosc do kolejki
	xQueueSendFromISR( ((tRadio*)ptr)->queue, &time, &xHigherPriorityTaskWoken );

	// Now the buffer is empty we can switch context if necessary.
	if( xHigherPriorityTaskWoken )
	{
		// Actual macro used here is port specific.
		portYIELD();
	}
}
