/*
 * I3G4250D_Lib.c
 *
 *  Created on: Jun 20, 2023
 *      Author: legionarul
 */

#include "I3G4250D_Lib.h"


void I3G4250D_RegisterWriteMultiple(uint8_t address, uint8_t * pData)
{
	address |= 0x40;
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &address, 1, 1000);
	HAL_SPI_Transmit(&hspi1, pData, strlen((const char *)pData), 1000);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}

void I3G4250D_RegisterReadMultiple(uint8_t address, uint8_t ret_value)
{
	// This needs work
//	address |= 0xC0;
//	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
//	HAL_SPI_Transmit(&hspi1, &address, 1, 100);
//	HAL_SPI_Receive(&hspi1, &ret_value, 1, 100);
//	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}

void I3G4250D_RegisterWrite(uint8_t address, uint8_t value)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &address, 1, 100);
	HAL_SPI_Transmit(&hspi1, &value, 1, 100);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}

uint8_t I3G4250D_RegisterRead(uint8_t address)
{
	uint8_t ret_value;
	address |= 0x80;
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &address, 1, 100);
	HAL_SPI_Receive(&hspi1, &ret_value, 1, 100);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

	return ret_value;
}


/********************************************************************/
/* Access WHO_AM_I register */
/********************************************************************/
uint8_t I3G4250D_WhoIAm()
{
	uint8_t ret_value;
	ret_value = I3G4250D_RegisterRead((uint8_t)I3G4250D_WHO_AM_I);
	return ret_value;
}


/********************************************************************/
/* Access CTRL_REG1 register */
/********************************************************************/
void I3G4250D_Set_CtrlReg1(uint8_t value)
{
	uint8_t address = (uint8_t)I3G4250D_CTRL_REG1;
	I3G4250D_RegisterWrite(address, value);
}

uint8_t I3G4250D_Get_CtrlReg1()
{
	uint8_t value;
	uint8_t address = (uint8_t)I3G4250D_CTRL_REG1;
	value = I3G4250D_RegisterRead(address);

	return value;
}


/********************************************************************/
/* Access CTRL_REG2 register */
/********************************************************************/
void I3G4250D_Set_CtrlReg2(uint8_t value)
{
	uint8_t address = (uint8_t)I3G4250D_CTRL_REG2;
	I3G4250D_RegisterWrite(address, value);
}

uint8_t I3G4250D_Get_CtrlReg2()
{
	uint8_t value;
	uint8_t address = (uint8_t)I3G4250D_CTRL_REG2;
	value = I3G4250D_RegisterRead(address);

	return value;
}


/********************************************************************/
/* Access CTRL_REG3 register */
/********************************************************************/
void I3G4250D_Set_CtrlReg3(uint8_t value)
{
	uint8_t address = (uint8_t)I3G4250D_CTRL_REG3;
	I3G4250D_RegisterWrite(address, value);
}
uint8_t I3G4250D_Get_CtrlReg3()
{
	uint8_t value;
	uint8_t address = (uint8_t)I3G4250D_CTRL_REG3;
	value = I3G4250D_RegisterRead(address);

	return value;
}


/********************************************************************/
/* Access CTRL_REG4 register */
/********************************************************************/
void I3G4250D_Set_CtrlReg4(uint8_t value)
{
	uint8_t address = (uint8_t)I3G4250D_CTRL_REG4;
	I3G4250D_RegisterWrite(address, value);
}

uint8_t I3G4250D_Get_CtrlReg4()
{
	uint8_t value;
	uint8_t address = (uint8_t)I3G4250D_CTRL_REG5;
	value = I3G4250D_RegisterRead(address);

	return value;
}


/********************************************************************/
/* Access CTRL_REG5 register */
/********************************************************************/
void I3G4250D_Set_CtrlReg5(uint8_t value)
{
	uint8_t address = (uint8_t)I3G4250D_CTRL_REG5;
	I3G4250D_RegisterWrite(address, value);
}

uint8_t I3G4250D_Get_CtrlReg5()
{
	uint8_t value;
	uint8_t address = (uint8_t)I3G4250D_CTRL_REG5;
	value = I3G4250D_RegisterRead(address);

	return value;
}


/********************************************************************/
/* Access REFERENCE register */
/********************************************************************/
void I3G4250D_Set_Reference(uint8_t value)
{
	uint8_t address = (uint8_t)I3G4250D_REFERENCE;
	I3G4250D_RegisterWrite(address, value);
}

uint8_t I3G4250D_Get_Reference()
{
	uint8_t value;
	uint8_t address = (uint8_t)I3G4250D_REFERENCE;
	value = I3G4250D_RegisterRead(address);

	return value;
}


/********************************************************************/
/* Access OUT_TEMP register */
/********************************************************************/
uint8_t I3G4250D_Get_OutTemp()
{
	uint8_t value;
	uint8_t address = (uint8_t)I3G4250D_OUT_TEMP;
	value = I3G4250D_RegisterRead(address);

	return value;
}


/********************************************************************/
/* Access STATUS_REG register */
/********************************************************************/
uint8_t I3G4250D_GetStatusReg()
{
	uint8_t value;
	uint8_t address = (uint8_t)I3G4250D_STATUS_REG;
	value = I3G4250D_RegisterRead(address);

	return value;
}


/********************************************************************/
/* Read OUT_X_L register */
/********************************************************************/
uint8_t I3G4250D_Get_OutXL()
{
	uint8_t value;
	uint8_t address = (uint8_t)I3G4250D_OUT_X_L;
	value = I3G4250D_RegisterRead(address);

	return value;
}


/********************************************************************/
/* Read OUT_X_H register */
/********************************************************************/
uint8_t I3G4250D_Get_OutXH()
{
	uint8_t value;
	uint8_t address = (uint8_t)I3G4250D_OUT_X_H;
	value = I3G4250D_RegisterRead(address);

	return value;
}


/********************************************************************/
/* Read OUT_Y_L register */
/********************************************************************/
uint8_t I3G4250D_Get_OutYL()
{
	uint8_t value;
	uint8_t address = (uint8_t)I3G4250D_OUT_Y_L;
	value = I3G4250D_RegisterRead(address);

	return value;
}


/********************************************************************/
/* Read OUT_Y_H register */
/********************************************************************/
uint8_t I3G4250D_Get_OutYH()
{
	uint8_t value;
	uint8_t address = (uint8_t)I3G4250D_OUT_Y_H;
	value = I3G4250D_RegisterRead(address);

	return value;
}


/********************************************************************/
/* Read OUT_Z_L register */
/********************************************************************/
uint8_t I3G4250D_Get_OutZL()
{
	uint8_t value;
	uint8_t address = (uint8_t)I3G4250D_OUT_Z_L;
	value = I3G4250D_RegisterRead(address);

	return value;
}


/********************************************************************/
/* Read OUT_Z_H register */
/********************************************************************/
uint8_t I3G4250D_Get_OutZH()
{
	uint8_t value;
	uint8_t address = (uint8_t)I3G4250D_OUT_Z_H;
	value = I3G4250D_RegisterRead(address);

	return value;
}


/********************************************************************/
/* Access FIFO_CTRL_REG register */
/********************************************************************/
void I3G4250D_Set_FifoCtrlReg(uint8_t value)
{
	uint8_t address = (uint8_t)I3G4250D_FIFO_CTRL_REG;
	I3G4250D_RegisterWrite(address, value);
}

uint8_t I3G4250D_Get_FifoCtrlReg()
{
	uint8_t value;
	uint8_t address = (uint8_t)I3G4250D_FIFO_CTRL_REG;
	value = I3G4250D_RegisterRead(address);

	return value;
}


/********************************************************************/
/* Read FIFO_SRC_REG register */
/********************************************************************/
uint8_t I3G4250D_Get_FifoSrcReg()
{
	uint8_t value;
	uint8_t address = (uint8_t)I3G4250D_FIFO_SRC_REG;
	value = I3G4250D_RegisterRead(address);

	return value;
}


/********************************************************************/
/* Access INT1_CFG register */
/********************************************************************/
void I3G4250D_Set_Int1Cfg(uint8_t value)
{
	uint8_t address = (uint8_t)I3G4250D_INT1_CFG;
	I3G4250D_RegisterWrite(address, value);
}

uint8_t I3G4250D_Get_Int1Cfg()
{
	uint8_t value;
	uint8_t address = (uint8_t)I3G4250D_INT1_CFG;
	value = I3G4250D_RegisterRead(address);

	return value;
}


/********************************************************************/
/* Read INT1_SRC register */
/********************************************************************/
uint8_t I3G4250D_Get_Int1Src()
{
	uint8_t value;
	uint8_t address = (uint8_t)I3G4250D_INT1_SRC;
	value = I3G4250D_RegisterRead(address);

	return value;
}


/********************************************************************/
/* Access INT1_THS_XH register */
/********************************************************************/
void I3G4250D_Set_Int1ThsXH(uint8_t value)
{
	uint8_t address = (uint8_t)I3G4250D_INT1_TSH_XH;
	I3G4250D_RegisterWrite(address, value);
}

uint8_t I3G4250D_Get_Int1ThsXH()
{
	uint8_t value;
	uint8_t address = (uint8_t)I3G4250D_INT1_TSH_XH;
	value = I3G4250D_RegisterRead(address);

	return value;
}


/********************************************************************/
/* Access INT1_THS_XL register */
/********************************************************************/
void I3G4250D_Set_Int1ThsXL(uint8_t value)
{
	uint8_t address = (uint8_t)I3G4250D_INT1_TSH_XL;
	I3G4250D_RegisterWrite(address, value);
}

uint8_t I3G4250D_Get_Int1ThsXL()
{
	uint8_t value;
	uint8_t address = (uint8_t)I3G4250D_INT1_TSH_XL;
	value = I3G4250D_RegisterRead(address);

	return value;
}


/********************************************************************/
/* Access INT1_THS_YH register */
/********************************************************************/
void I3G4250D_Set_Int1ThsYH(uint8_t value)
{
	uint8_t address = (uint8_t)I3G4250D_INT1_TSH_YH;
	I3G4250D_RegisterWrite(address, value);
}

uint8_t I3G4250D_Get_Int1ThsYH()
{
	uint8_t value;
	uint8_t address = (uint8_t)I3G4250D_INT1_TSH_YH;
	value = I3G4250D_RegisterRead(address);

	return value;
}


/********************************************************************/
/* Access INT1_THS_YL register */
/********************************************************************/
void I3G4250D_Set_Int1ThsYL(uint8_t value)
{
	uint8_t address = (uint8_t)I3G4250D_INT1_TSH_YL;
	I3G4250D_RegisterWrite(address, value);
}

uint8_t I3G4250D_Get_Int1ThsYL()
{
	uint8_t value;
	uint8_t address = (uint8_t)I3G4250D_INT1_TSH_YL;
	value = I3G4250D_RegisterRead(address);

	return value;
}


/********************************************************************/
/* Access INT1_THS_ZH register */
/********************************************************************/
void I3G4250D_Set_Int1ThsZH(uint8_t value)
{
	uint8_t address = (uint8_t)I3G4250D_INT1_TSH_ZH;
	I3G4250D_RegisterWrite(address, value);
}

uint8_t I3G4250D_Get_Int1ThsZH()
{
	uint8_t value;
	uint8_t address = (uint8_t)I3G4250D_INT1_TSH_ZH;
	value = I3G4250D_RegisterRead(address);

	return value;
}


/********************************************************************/
/* Access INT1_THS_ZL register */
/********************************************************************/
void I3G4250D_Set_Int1ThsZL(uint8_t value)
{
	uint8_t address = (uint8_t)I3G4250D_INT1_TSH_ZL;
	I3G4250D_RegisterWrite(address, value);
}

uint8_t I3G4250D_Get_Int1ThsZL()
{
	uint8_t value;
	uint8_t address = (uint8_t)I3G4250D_INT1_TSH_ZL;
	value = I3G4250D_RegisterRead(address);

	return value;
}


/********************************************************************/
/* Access INT1_DURATION register */
/********************************************************************/
void I3G4250D_Set_Int1Duration(uint8_t value)
{
	uint8_t address = (uint8_t)I3G4250D_INT1_DURATION;
	I3G4250D_RegisterWrite(address, value);
}

uint8_t I3G4250D_Get_Int1Duration()
{
	uint8_t value;
	uint8_t address = (uint8_t)I3G4250D_INT1_DURATION;
	value = I3G4250D_RegisterRead(address);

	return value;
}


/********************************************************************/
/* Functional APIs */
/********************************************************************/
void I3G4250D_SetPowerMode(uint8_t mode)
{
	/* TODO: This variable must be read from the register */
	uint8_t value = 0;

	if (mode == I3G4250D_NORMAL)
	{
		value |= (1 << I3G4250D_NORMAL_SHIFT);
	}
	else if (mode == I3G4250D_SLEEP)
	{
		value &= I3G4250D_SLEEP_CLR_MASK;
		value |= (1 << I3G4250D_SLEEP_SHIFT);
	}
	else
	{
		value &= ~(1 << I3G4250D_POWER_DOWN_SHIFT);
	}

	I3G4250D_Set_CtrlReg1(value);
	HAL_Delay(5);
}

/* TODO: This needs to be updated. Will read normal mode even if it is only sleep mode */
I3G4250D_power_mode_status_t I3G4250D_GetPowerMode()
{
	uint8_t retStatus;

	if (((I3G4250D_Get_CtrlReg1() & I3G4250D_NORMAL_MASK) >> I3G4250D_NORMAL_SHIFT) == I3G4250D_NORMAL)
	{
		retStatus = I3G4250D_STATUS_NORMAL;
	}
	else if ((I3G4250D_Get_CtrlReg1() & I3G4250D_SLEEP_MASK) == I3G4250D_SLEEP)
	{
		retStatus = I3G4250D_STATUS_SLEEP;
	}
	else
	{
		retStatus = I3G4250D_STATUS_POWER_DOWN;
	}

	return retStatus;
}


/********************************************************************/
/* Enable gyroscope by configuring CTRL_REG1 */
/********************************************************************/
void I3G4250D_GyroEnable(uint8_t value)
{
	uint8_t temp = 0;
	uint8_t address = 0;

	address = (uint8_t)I3G4250D_CTRL_REG1;
	temp = I3G4250D_Get_CtrlReg1();
	temp |= value;

	I3G4250D_RegisterWrite(address, temp);
}


/********************************************************************/
/* Configure High Pass filter by writing to CTRL_REG2 */
/********************************************************************/
void I3G4250D_HPFilterConfig(uint8_t hpfMode, uint8_t hpfCutOff)
{
	uint8_t address = (uint8_t)I3G4250D_CTRL_REG2;
	uint8_t value = 0;

	value = (hpfMode << I3G4250D_HPM_SHIFT) | hpfCutOff;
	I3G4250D_RegisterWrite(address, value);
}


/********************************************************************/
/* Configure interrupts and output port by writing to CTRL_REG3 */
/********************************************************************/
void I3G4250D_InterruptConfig(uint8_t value)
{
	uint8_t address = (uint8_t)I3G4250D_CTRL_REG3;
	I3G4250D_RegisterWrite(address, value);
}


/********************************************************************/
/* Configure full scale by writing to CTRL_REG4 */
/********************************************************************/
void I3G4250D_SetFullScale(uint8_t value)
{
	uint8_t address = (uint8_t)I3G4250D_CTRL_REG4;
	I3G4250D_RegisterWrite(address, value);
}


/********************************************************************/
/* Configure SPI mode by writing to CTRL_REG4 */
/********************************************************************/
void I3G4250D_SetSPIMode(uint8_t value)
{
	uint8_t address = (uint8_t)I3G4250D_CTRL_REG4;
	I3G4250D_RegisterWrite(address, value);
}


/********************************************************************/
/* Configure endianess writing to CTRL_REG4 */
/********************************************************************/
void I3G4250D_SetEndianess(uint8_t value)
{
	uint8_t address = (uint8_t)I3G4250D_CTRL_REG4;
	I3G4250D_RegisterWrite(address, value);
}


/********************************************************************/
/* Enable self test by writing to CTRL_REG4 */
/********************************************************************/
void I3G4250D_SelfTestEnable(uint8_t value)
{
	uint8_t address = (uint8_t)I3G4250D_CTRL_REG4;
	I3G4250D_RegisterWrite(address, value);
}


/********************************************************************/
/* Reboot memory content by writing to CTRL_REG5 */
/********************************************************************/
void I3G4250D_RebootMem(uint8_t value)
{
	uint8_t address = (uint8_t)I3G4250D_CTRL_REG5;
	I3G4250D_RegisterWrite(address, value);
}


/********************************************************************/
/* Out selection configuration by writing to CTRL_REG5 */
/********************************************************************/
void I3G4250D_OutSelConfig(uint8_t value)
{
	uint8_t address = (uint8_t)I3G4250D_CTRL_REG5;
	I3G4250D_RegisterWrite(address, value);
}


Gyro_XYZ_t I3G4250D_ReadGyroXYZ()
{
	Gyro_XYZ_t xyzRaw;
	uint8_t buff[6];
	uint8_t temp = 0;
	temp = (uint8_t)I3G4250D_ZYXDA;
	uint8_t address = (uint8_t)I3G4250D_OUT_X_L;
	address |= 0xC0;
	int16_t val[3];

	if (I3G4250D_GetStatusReg() & temp)
	{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, &address, 1, 100);
		HAL_SPI_Receive(&hspi1, buff, 6, 100);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

		val[0] = (int16_t)buff[1];
		val[0] = (val[0] * 256) + (int16_t)buff[0];
		val[1] = (int16_t)buff[3];
		val[1] = (val[1] * 256) + (int16_t)buff[2];
		val[2] = (int16_t)buff[5];
		val[2] = (val[2] * 256) + (int16_t)buff[4];

//		xyzRaw.xAxis = (buf[1] << 8) | buf[0];
//		xyzRaw.yAxis = (buf[3] << 8) | buf[2];
//		xyzRaw.zAxis = (buf[3] << 8) | buf[2];
//		xyzRaw.xAxis = (I3G4250D_Get_OutXH() << 8) | I3G4250D_Get_OutXL();
//		xyzRaw.yAxis = (I3G4250D_Get_OutYH() << 8) | I3G4250D_Get_OutYL();
//		xyzRaw.zAxis = (I3G4250D_Get_OutZH() << 8) | I3G4250D_Get_OutZL();
	}

	xyzRaw.xAxis = ((float)val[0] * 8.75f);
	xyzRaw.yAxis = ((float)val[1] * 8.75f);
	xyzRaw.zAxis = ((float)val[2] * 8.75f);

	return xyzRaw;
}













