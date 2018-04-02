/*
 * rx_data.c
 *
 *  Created on: 02.09.2017
 *      Author: Mariusz
 */

#include "math.h"
#include "stdio.h"
#include "string.h"

#include "DISCOVERY/uart.h"

#include "MPU/inv_mpu.h"
#include "MPU/inv_mpu_dmp_motion_driver.h"

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */


static void (*dmp_rx_data_event_callback)(void);


static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};


volatile uint8_t dmp_data=0;


#define DEFAULT_MPU_HZ  (200)


void register_dmp_rx_data_event_callback(void (*callback)(void))
{
	dmp_rx_data_event_callback = callback;
}


void DMP_Rx_Data_Event(void)
{
	if(dmp_data)
	{
		if(dmp_rx_data_event_callback)
		{
			(*dmp_rx_data_event_callback)();
		}
	}
}





void dmp_rx_parse(void)
{
	unsigned long sensor_timestamp;
    short gyro[3], accel[3], sensors;
    unsigned char more;
    long quat[4];
    char b[64];
    float quatf[4];

	if( !dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more) )
	{
		double x = (pow(quat[0],2) + pow(quat[1],2) + pow(quat[2],2) + pow(quat[3],2));

			double d = sqrt( x );

			for(uint8_t i=0; i<=3;i++)
			{
				quatf[i] = quat[i]/d;
			}

			double roll, pitch, yaw;
			char buf[40] = "Y,";

			roll = 57.2*atan2( -2*quatf[1]*quatf[3] + 2*quatf[0]*quatf[2] , pow(quatf[3],2) - pow(quatf[2],2) - pow(quatf[1],2) + pow(quatf[0],2) );
			sprintf(b, "%2.2f", roll);
			strcat(buf,b);
			strcat(buf,",");


			pitch = 57.2*asin(2*quatf[2]*quatf[3]+2*quatf[0]*quatf[1]);
			sprintf(b, "%2.2f", pitch);
		//        	uart_putstring(b);
		//        	uart_putstring("; ");
			strcat(buf,b);
			strcat(buf,",");

		    yaw = 57.2*atan2( -2*quatf[1]*quatf[2] + 2*quatf[0]*quatf[3] , pow(quatf[2],2) - pow(quatf[3],2) - pow(quatf[1],2) + pow(quatf[0],2) );
		    sprintf(b, "%2.2f", yaw);
		//  uart_putstring(b);
		//  uart_putstring("; ");
		    strcat(buf,b);
		    strcat(buf,"\r\n");

		    uart_putstring(buf);
		    dmp_data--;
	}
}


void konfiguracja_dmp(void)
{
	/*
	 *  konfiguracja mpu
	 */
	// inr param ?? --  ??
	struct int_param_s int_param;
	uart_putstring("konfiguracja mpu...\n");
	if(0==mpu_init(&int_param)) uart_putstring("OK \r\n");
	///////////////////////////////////////////////////////////

	/*
	 *  wake up all sensors
	 */
	uart_putstring("konfiguracja czujnikow...\n");
	if(0==mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL )) uart_putstring("OK \r\n");
	///////////////////////////////////////////////////////////////////////////////

	/*
	 *  push both gyro and accel data into the FIFO
	 */
	uart_putstring("konfiguracja fifo...\n");
	if(0==mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL )) uart_putstring("OK \r\n");
	////////////////////////////////////////////

	/*
	 * ustawienie sample rate
	 */
	uart_putstring("ustawianie sample rate...\n");
	if(0==mpu_set_sample_rate(DEFAULT_MPU_HZ)) uart_putstring("OK \r\n");
	//////////////////////////////////////

//	????????????????????????????????????????????????
//
//	/*
//	 * 	Read back configuration in case it was set improperly.
//	 */
//    mpu_get_sample_rate(&gyro_rate);
//    mpu_get_gyro_fsr(&gyro_fsr);
//    mpu_get_accel_fsr(&accel_fsr);
//	///////////////////////////////////////////////


	/*
	 * dmp firmware loading
	 */
	uart_putstring("Program loading, please wait... \n");

	int a = dmp_load_motion_driver_firmware();

	if(0==a) uart_putstring("Program loaded \r\n");
	else if(-1==a) uart_putstring("error -1 \r\n");
	else if(-2==a) uart_putstring("error -2 \r\n");
	else if(-3==a) uart_putstring("error -3 \r\n");
	else uart_putstring("error nwmco \r\n");
	////////////////////////////////////////////////////

	/*
	 * zapisywanie macierzy orientacji (czujnika w ukladzie ?)
	 */
	uart_putstring("Push orientation matrix to the DMP...\n");
	if(0==dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation))) uart_putstring("OK \r\n");
	////////////////////////////////////////////////

	/*
	 *  // rejestracja callbacka od tap'a
	 */
//	uart_putstring("Rejestracja callbacka od tap'a...");
//	if(0==dmp_register_tap_cb(tap_cb)) uart_putstring("OK \r\n");
	/////////////////////////////////////////////////////
	/*
	 *  rejestracja callbacka od orientacji androidowej
	 */
//	uart_putstring("Rejestracja callbacka andriod orienta...");
//	if(0==dmp_register_android_orient_cb(android_orient_cb)) uart_putstring("OK \r\n");
	/////////////////////////////////////////////////

	/*
	 * konfiguracja DMP
	 */
	uart_putstring("Konfiguracja features DMP...\n");
	if(0==dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL| DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_TAP)) uart_putstring("OK \r\n");
	////////////////////////////////////////////////////////////////////
	/*
	 *  czestotliwosc FIFO
	 */
	uart_putstring("Ustawienie fifo rate...\n");
	if(0==dmp_set_fifo_rate(DEFAULT_MPU_HZ)) uart_putstring("OK \r\n");
	/////////////////////////////////////////////////////

	/*
	 *  wlaczenie DMP
	 */
	uart_putstring("Wlaczenie DMP...\n");
	if(0==mpu_set_dmp_state(1)) uart_putstring("OK \r\n");
	/////////////////////////////////////////////////////
	/*
	 *
	 */

//	uart_putstring("\r\n\r\nodczyt rejestrow po konfiguracji :\r\n");
//	read_mpu_register();

	/*
	 *   konfiguracja przerwania zewnetrznego
	 */

}


void EXTI0_IRQHandler(void)
{
}
