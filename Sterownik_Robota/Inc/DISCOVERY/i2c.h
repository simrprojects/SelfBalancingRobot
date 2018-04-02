/*
 * i2c.h
 *
 *  Created on: 22.08.2017
 *      Author: Mariusz
 */

#ifndef I2C_H_
#define I2C_H_





void i2c_init(void);
int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);
int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);
unsigned char i2c_write_bajt(unsigned char SLA, unsigned char adr, unsigned char data);


#endif /* I2C_H_ */
