/**************
  ƥ��inv_mpu.c��I2C��д�ӿ�
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

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t IICwriteBit(uint8_t addr, uint8_t reg, uint8_t bitNum, uint8_t data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �е�1��λ
����	addr  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		bitNum  Ҫ�޸�Ŀ���ֽڵ�bitNumλ
		data  Ϊ0 ʱ��Ŀ��λ������0 ���򽫱���λ
����   �ɹ� Ϊ1 
 		ʧ��Ϊ0
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


/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t IICreadBytes(uint8_t addr, uint8_t reg, uint8_t length, uint8_t *data)
*��������:	    ��ȡָ���豸 ָ���Ĵ����� length��ֵ
����	addr  Ŀ���豸��ַ
		reg	  �Ĵ�����ַ
		length Ҫ�����ֽ���
		*data  ���������ݽ�Ҫ��ŵ�ָ��
����   ���������ֽ�����
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

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t IICwriteBits(uint8_t dev,uint8_t reg,uint8_t bitStart,uint8_t length,uint8_t data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �еĶ��λ
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		bitStart  Ŀ���ֽڵ���ʼλ
		length   λ����
		data    ��Ÿı�Ŀ���ֽ�λ��ֵ
����   �ɹ� Ϊ1 
 		ʧ��Ϊ0
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
