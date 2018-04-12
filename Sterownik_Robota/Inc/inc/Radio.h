/**\file	Radio.h
 * \brief Modul odpowiedzialny za nadzorowanie pracy radioodbiornika - plik naglowkowy
 */

#ifndef RADIO_H_
#define RADIO_H_

//#include "TimChManager.h"

#define Channel_LeftVertical	0
#define Channel_LeftHorizontal	3
#define Channel_RightVertical	2
#define Channel_RightHorizontal	1
#define Channel_Switch_1	6

void* Radio_Init(int numberOfChannels);
signed int Radio_GetValue(int channal);
unsigned short Radio_GetPwmValue(int channal);
int* Radio_GetChannelMeasurements(void);
void Radio_Update(void* handler,unsigned int time);

#endif
