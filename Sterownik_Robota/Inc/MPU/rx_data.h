/*
 * rx_data.h
 *
 *  Created on: 02.09.2017
 *      Author: Mariusz
 */

#ifndef MPU_RX_DATA_H_
#define MPU_RX_DATA_H_


extern volatile uint8_t dmp_data;



void DMP_Rx_Data_Event(void);
void register_dmp_rx_data_event_callback(void (*callback)(void));


void konfiguracja_dmp(void);
void dmp_rx_parse(void);



#endif /* MPU_RX_DATA_H_ */
