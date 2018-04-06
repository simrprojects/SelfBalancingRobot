/**
  ******************************************************************************
  * @file	 MPUI2CDriver.c
  * @author  Przemek
  * @version V1.0.0
  * @date    03.04.2018
  * @brief   Modu� implementuje funkcje komunikacji po I2C dla modu�u MPU
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include "OSI2C.h"
#include "FreeRTOS.h"
#include "task.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
I2CComPortHandle hi2c;
/* Private function prototypes -----------------------------------------------*/
/* Public  functions ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void i2c_init(void)
{

	OSI2C_Init(&hi2c,&hi2c1);

}

int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data)
{
	/*if(HAL_I2C_Mem_Read(&hi2c1,slave_addr,reg_addr,1,data,length,100)==HAL_OK){
		return 0;
	}else{
		return 1;
	}*/
	return OSI2C_Mem1Read(hi2c,slave_addr,reg_addr,data,length,100);
}

int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data)
{

	/*if(HAL_I2C_Mem_Write(&hi2c1,slave_addr,reg_addr,1,data,length,100)==HAL_OK){
		return 0;
	}else{
		return 1;
	}*/
	return OSI2C_Mem1Write(hi2c,slave_addr,reg_addr,data,length,100);

}

/*unsigned char i2c_write_bajt(unsigned char SLA, unsigned char adr, unsigned char data)
{
	if(HAL_I2C_Mem_Write(&hi2c1,SLA,adr,1,&data,1,40)==HAL_OK){
		return 0;
	}else{
		return 1;
	}
	return OSI2C_Mem1Write(hi2c,SLA,adr,&data,1,50);

}*/

void delay_nus(int n) {
	HAL_Delay(1);
}

void delay_1ms(void) {
	vTaskDelay(1);
}

void delay_ms(int n) {
	vTaskDelay(n);
}

int get_ms(unsigned long *count)
{
	return 0;
}

void __no_operation(void){

}
