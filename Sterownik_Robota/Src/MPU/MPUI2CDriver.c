/**
  ******************************************************************************
  * @file	 MPUI2CDriver.c
  * @author  Przemek
  * @version V1.0.0
  * @date    03.04.2018
  * @brief   Modu³ implementuje funkcje komunikacji po I2C dla modu³u MPU
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Public  functions ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void i2c_init(void)
{



}

int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data)
{
	if(HAL_I2C_Mem_Read(&hi2c1,slave_addr,reg_addr,1,data,length,100)==HAL_OK){
		return 0;
	}else{
		return 1;
	}
}

int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data)
{

	if(HAL_I2C_Mem_Write(&hi2c1,slave_addr,reg_addr,1,data,length,100)==HAL_OK){
		return 0;
	}else{
		return 1;
	}

}

unsigned char i2c_write_bajt(unsigned char SLA, unsigned char adr, unsigned char data)
{
	if(HAL_I2C_Mem_Write(&hi2c1,SLA,adr,1,&data,1,40)==HAL_OK){
		return 0;
	}else{
		return 1;
	}

}
