/**************
  匹配inv_mpu.c的I2C读写接口
***************/

#include "HAL_I2C.h"

extern I2C_HandleTypeDef hi2c1;
//extern I2C_HandleTypeDef hi2c2;
//extern I2C_HandleTypeDef hi2c3;

#define MpuI2c hi2c1

int i2cRead(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *tmp)
{
    int req;
    req = HAL_I2C_Mem_Read( &MpuI2c, addr<<1, reg,I2C_MEMADD_SIZE_8BIT,tmp,len,1000);
    if(!req) return 0;
    else return -1;
}

int i2cWrite(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *tmp)
{
    int req;
    req = HAL_I2C_Mem_Write( &MpuI2c, addr<<1, reg,I2C_MEMADD_SIZE_8BIT,tmp,len,1000);
    if(!req) return 0;
    else return -1;
}

/**************************实现函数********************************************
*函数原型:		uint8_t IICwriteBit(uint8_t addr, uint8_t reg, uint8_t bitNum, uint8_t data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的1个位
输入	addr  目标设备地址
		reg	   寄存器地址
		bitNum  要修改目标字节的bitNum位
		data  为0 时，目标位将被清0 否则将被置位
返回   成功 为1 
 		失败为0
*******************************************************************************/ 
uint8_t IICwriteBit(uint8_t addr, uint8_t reg, uint8_t bitNum, uint8_t data)
{
    uint8_t Byte;
    
    HAL_I2C_Mem_Read( &MpuI2c,addr<<1, reg,1,&Byte,1,1000);
    Byte = (data != 0) ? (Byte | (1 << bitNum)) : (Byte & ~(1 << bitNum));
    int req;
    req = HAL_I2C_Mem_Write( &MpuI2c, addr<<1, reg,1,&Byte,1,1000);

    if(req) return 0;
    else return -1;
}


/**************************实现函数********************************************
*函数原型:		uint8_t IICreadBytes(uint8_t addr, uint8_t reg, uint8_t length, uint8_t *data)
*功　　能:	    读取指定设备 指定寄存器的 length个值
输入	addr  目标设备地址
		reg	  寄存器地址
		length 要读的字节数
		*data  读出的数据将要存放的指针
返回   读出来的字节数量
*******************************************************************************/ 
uint8_t IICreadBytes(uint8_t addr, uint8_t reg, uint8_t length, uint8_t *data)
{
    uint8_t req;
  req = i2cRead(addr, reg, length, data);
  if(req) return 0;
  else return -1;
}

uint8_t IICreadByte(uint8_t dev, uint8_t reg, uint8_t *data){
	*data=I2C_ReadOneByte(dev, reg);
    return 1;
}

/**************************实现函数********************************************
*函数原型:		uint8_t IICwriteBits(uint8_t dev,uint8_t reg,uint8_t bitStart,uint8_t length,uint8_t data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的多个位
输入	dev  目标设备地址
		reg	   寄存器地址
		bitStart  目标字节的起始位
		length   位长度
		data    存放改变目标字节位的值
返回   成功 为1 
 		失败为0
*******************************************************************************/ 
uint8_t IICwriteBits(uint8_t dev,uint8_t reg,uint8_t bitStart,uint8_t length,uint8_t data)
{
    uint8_t byte;
    if (IICreadByte(dev, reg, &byte) != 0) 
    {
        uint8_t mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        byte &= mask;
        byte |= data;
    }
}

uint8_t I2C_ReadOneByte(uint8_t addr,uint8_t reg)
{
  uint8_t tmp;
  i2cRead(addr, reg, 1, &tmp);
  return tmp;
}
