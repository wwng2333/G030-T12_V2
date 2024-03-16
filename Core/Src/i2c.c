/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include "main.h"
#include "stdio.h"

#define I2C_WRITE 0
#define I2C_READ 1
#define TMP75_ADDR (0x48 << 1)

void I2C_Delay(void)
{
  __IO uint16_t i;
  for (i = 0; i < 500; i++)
	{
		__asm volatile("nop");
	}
	//LL_mDelay(1);
}

#define Pin_SCL_L LL_GPIO_ResetOutputPin(PB8_SCL_GPIO_Port, PB8_SCL_Pin)
#define Pin_SCL_H LL_GPIO_SetOutputPin(PB8_SCL_GPIO_Port, PB8_SCL_Pin)

#define Pin_SDA_L LL_GPIO_ResetOutputPin(PB9_SDA_GPIO_Port, PB9_SDA_Pin)
#define Pin_SDA_H LL_GPIO_SetOutputPin(PB9_SDA_GPIO_Port, PB9_SDA_Pin)

#define Read_SDA_Pin LL_GPIO_IsInputPinSet(PB9_SDA_GPIO_Port, PB9_SDA_Pin)
#define Read_SCL_Pin LL_GPIO_IsInputPinSet(PB8_SCL_GPIO_Port, PB8_SCL_Pin)

float TMP75_ReadTemp(void)
{
	float temp = 0.0f;
	int16_t dat = 0;
	dat = TMP75_Read_2Byte(0x00);
	if(dat&0x8000) 
	{
		dat = ~(dat - 1);
	}
	temp = (float)dat / 256;
	//printf("TMP75 0x00=%.2f\r\n", temp);
//	printf("TMP75 0x00=%d\n", dat);
	return temp;
}

uint16_t TMP75_Read_2Byte(uint8_t addr)
{
	uint16_t dat = 0;
	I2C_Start();
	I2C_SendData(TMP75_ADDR);
	I2C_RecvACK();
	I2C_SendData(addr);
	I2C_RecvACK();
	I2C_Start();
	I2C_SendData(TMP75_ADDR + 1);
	I2C_RecvACK();
	dat = I2C_RecvData();
	dat <<= 8;
	I2C_SendACK();
	dat |= I2C_RecvData();
	I2C_SendNAK();
	I2C_Stop();

	return dat;
}

void I2C_Start()
{
	Pin_SDA_H;
	Pin_SCL_H;
	I2C_Delay();
	Pin_SDA_L;
	I2C_Delay();
	Pin_SCL_L;
}

void I2C_SendData(uint8_t dat)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		Pin_SCL_L;
		I2C_Delay();
		if (dat & 0x80)
			Pin_SDA_H;
		else
			Pin_SDA_L;
		dat <<= 1;
		Pin_SCL_H;
		I2C_Delay();
	}
	Pin_SCL_L;
}

void I2C_SendACK()
{
	Pin_SCL_L;
	I2C_Delay();
	Pin_SDA_L;
	Pin_SCL_H;
	I2C_Delay();
	Pin_SCL_L;
	Pin_SDA_H;
	I2C_Delay();
}

void I2C_RecvACK()
{
	Pin_SCL_L;
	I2C_Delay();
	Pin_SDA_H;
	Pin_SCL_H;
	I2C_Delay();
	Pin_SCL_L;
	I2C_Delay();
}

uint8_t I2C_RecvData()
{
	uint8_t dat, i;
	for (i = 0; i < 8; i++)
	{
		dat <<= 1;
		Pin_SCL_L;
		I2C_Delay();
		Pin_SCL_H;
		I2C_Delay();
		if (Read_SDA_Pin == 1)
		{
			dat |= 0x01;
		}
	}
	return dat;
}

void I2C_SendNAK()
{
	Pin_SCL_L;
	I2C_Delay();
	Pin_SDA_H;
	Pin_SCL_H;
	I2C_Delay();
	Pin_SCL_L;
	I2C_Delay();
}

void I2C_Stop()
{
	Pin_SDA_L;
	Pin_SCL_H;
	I2C_Delay();
	Pin_SDA_H;
	I2C_Delay();
}