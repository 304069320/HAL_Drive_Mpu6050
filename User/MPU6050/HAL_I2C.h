#ifndef __HAL_I2C
#define __HAL_I2C

/*
  include header file
*/
#include "stdint.h"
#include "stm32f1xx_hal.h"
//#include "stm32F1xx_hal_conf.h"


/*
 function declaration
*/
int i2cRead(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *tmp);//   (addr,register,length,*buf)
int i2cWrite(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *tmp); //   (addr,register,length,*buf)

uint8_t IICwriteBit(uint8_t addr, uint8_t reg, uint8_t bitNum, uint8_t data);
uint8_t IICreadBytes(uint8_t addr, uint8_t reg, uint8_t length, uint8_t *data);
uint8_t IICwriteBits(uint8_t dev,uint8_t reg,uint8_t bitStart,uint8_t length,uint8_t data);
uint8_t I2C_ReadOneByte(uint8_t addr,uint8_t reg);
#endif