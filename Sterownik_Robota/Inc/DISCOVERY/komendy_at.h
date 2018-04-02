/*
 * komendy_at.h
 *
 *  Created on: 06.08.2017
 *      Author: Mariusz
 */

#ifndef KOMENDY_AT_H_
#define KOMENDY_AT_H_




// definicja typu strukturalnego
typedef struct{
	char polecenie_at[15];
	int8_t (* at_service)(uint8_t inout, char *params);
}TATCMD;


extern TATCMD polecenia_at[];


void parse_uart_data(char *pBuf);

int8_t at_service(uint8_t inout, char *params);
int8_t ati_service(uint8_t inout, char *params);
int8_t enter_service(uint8_t inout, char *params);
int8_t at_pid1_p_service(uint8_t inout, char *params);


#endif /* KOMENDY_AT_H_ */
