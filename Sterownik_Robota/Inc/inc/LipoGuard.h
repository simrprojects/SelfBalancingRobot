/*
 * LipoGuard.h
 *
 *  Created on: 14.05.2018
 *      Author: Przemek
 */

#ifndef INC_LIPOGUARD_H_
#define INC_LIPOGUARD_H_

typedef void* tLipoGuardHandler;

typedef enum{eLipoOk,eLipoWarning,eLipoDischarged}tLipoGuardBateryState;

typedef struct{
	float refreshRate;/**<częstotliwośc pracy modulu [Hz]*/
	int numOfCells;/**<liczba cel w pakiecie*/
	float warningLevel;/**<poziom napięcia pojedynczej celi poniżej którego włączy się stan ostrzegawczy*/
	float errorLevel;/**<poziom napięciapojedynczej celi ponizej którego włącza się tan alarmowy*/
	float histeresis;/**histereza napięcia*/
}tLipoGuardConfig;

tLipoGuardHandler LipoGuard_Init(tLipoGuardConfig *cfg);
void LipoGuard_Update(tLipoGuardHandler h, float voltage);

void LipoGuard_NewStateCallBack(tLipoGuardHandler h,tLipoGuardBateryState state);

#endif /* INC_LIPOGUARD_H_ */
