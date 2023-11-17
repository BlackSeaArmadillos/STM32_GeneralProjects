/*
 * LSM303AGR_Lib.c
 *
 *  Created on: Jul 31, 2023
 *      Author: legionarul
 */

#include "LSM303AGR_Lib.h"


/********************************************************************/
/* Access accelerometer WHO_AM_I register */
/********************************************************************/
uint8_t LSM303AGR_AccWhoIAm()
{
	uint8_t whoIAm = 0;
	uint16_t deviceAddress = LSM303AGR_ACC_ADDR_READ;
	uint16_t registerAddress = LSM303AGR_WHO_AM_I_A;
	HAL_I2C_Mem_Read(&hi2c1, deviceAddress, registerAddress, 1, &whoIAm, 1, 10);

	return whoIAm;
}


/********************************************************************/
/* Access magnetometer WHO_AM_I register */
/********************************************************************/
uint8_t LSM303AGR_MagWhoIAm()
{
	uint8_t whoIAm = 0;
	uint16_t deviceAddress = LSM303AGR_MAG_ADDR_READ;
	uint16_t registerAddress = LSM303AGR_WHO_AM_I_M;
	HAL_I2C_Mem_Read(&hi2c1, deviceAddress, registerAddress, 1, &whoIAm, 1, 10);

	return whoIAm;
}


/********************************************************************/
/* Set block data update field in CTRL_REG4_A register */
/********************************************************************/
LSM303AGR_Status_t LSM303AGR_AccSetBlockDataUpdate(uint8_t value)
{
	LSM303AGR_Status_t retStat = LSM303AGR_ERROR;
	uint16_t deviceAddress = LSM303AGR_ACC_ADDR_WRITE;
	uint8_t regAddress = LSM303AGR_CTRL_REG4_A;
	uint8_t data = value << LSM303AGR_ACC_CTRL_REG4_BDU_SHIFT;

	if (HAL_I2C_Mem_Write(&hi2c1, deviceAddress, regAddress, 1, &data, 1, 10) == HAL_OK)
		retStat = LSM303AGR_OK;

	return retStat;
}


/********************************************************************/
/* Set data rate by changing ODR field in CTRL_REG1_A register */
/********************************************************************/
LSM303AGR_Status_t LSM303AGR_AccSetDataRate(uint8_t value)
{
	LSM303AGR_Status_t retStat = LSM303AGR_ERROR;
	uint16_t deviceAddress = LSM303AGR_ACC_ADDR_WRITE;
	uint8_t regAddress = LSM303AGR_CTRL_REG1_A;
	uint8_t data = value << LSM303AGR_ACC_CTRL_REG1_ODR_SHIFT;

	if (HAL_I2C_Mem_Write(&hi2c1, deviceAddress, regAddress, 1, &data, 1, 10) == HAL_OK)
		retStat = LSM303AGR_OK;

	return retStat;
}


/********************************************************************/
/* Set power mode by setting LPen field in CTRL_REG1_A register */
/********************************************************************/
LSM303AGR_Status_t LSM303AGR_AccSetPowerMode(uint8_t value)
{
	LSM303AGR_Status_t retStat = LSM303AGR_ERROR;
	uint16_t deviceAddress = LSM303AGR_ACC_ADDR_WRITE;
	uint8_t regAddress = LSM303AGR_CTRL_REG1_A;
	uint8_t data = value << LSM303AGR_ACC_CTRL_REG1_LPen_SHIFT;

	if (HAL_I2C_Mem_Write(&hi2c1, deviceAddress, regAddress, 1, &data, 1, 10) == HAL_OK)
		retStat = LSM303AGR_OK;

	return retStat;
}


/********************************************************************/
/* Enable XYZ axes in CTRL_REG1_A register */
/********************************************************************/
LSM303AGR_Status_t LSM303AGR_AccEnableXYZAxes(uint8_t value)
{
	LSM303AGR_Status_t retStat = LSM303AGR_ERROR;
	uint16_t deviceAddress = LSM303AGR_ACC_ADDR_WRITE;
	uint8_t regAddress = LSM303AGR_CTRL_REG1_A;

	if (HAL_I2C_Mem_Write(&hi2c1, deviceAddress, regAddress, 1, &value, 1, 10) == HAL_OK)
		retStat = LSM303AGR_OK;

	return retStat;
}


/********************************************************************/
/* Configure full-scale in CTRL_REG4_A register */
/********************************************************************/
LSM303AGR_Status_t LSM303AGR_AccSetFullScale(uint8_t value)
{
	LSM303AGR_Status_t retStat = LSM303AGR_ERROR;
	uint16_t deviceAddress = LSM303AGR_ACC_ADDR_WRITE;
	uint8_t regAddress = LSM303AGR_CTRL_REG4_A;
	uint8_t data = value << LSM303AGR_ACC_CTRL_REG4_FS_SHIFT;

	if (HAL_I2C_Mem_Write(&hi2c1, deviceAddress, regAddress, 1, &data, 1, 10) == HAL_OK)
		retStat = LSM303AGR_OK;

	return retStat;
}


/********************************************************************/
/* Configure operating mode by seting fields in CTRL_REG1_A and
 * CTRL_REG4_A registers */
/********************************************************************/
LSM303AGR_Status_t LSM303AGR_AccSetOperatingMode(uint8_t value)
{
	LSM303AGR_Status_t retStat = LSM303AGR_ERROR;
	uint16_t deviceAddress = LSM303AGR_ACC_ADDR_WRITE;
	uint8_t reg1Address = LSM303AGR_CTRL_REG1_A;
	uint8_t reg4Address = LSM303AGR_CTRL_REG4_A;
	uint8_t data1 = 0;
	uint8_t data2 = 0;

	HAL_I2C_Mem_Read(&hi2c1, deviceAddress, reg1Address, 1, &data1, 1, 10);
	HAL_I2C_Mem_Read(&hi2c1, deviceAddress, reg4Address, 1, &data2, 1, 10);

	if (value == LSM303AGR_ACC_LOW_POWER_MODE)
	{
		data1 |= 0x01 << LSM303AGR_ACC_CTRL_REG1_LPen_SHIFT;
		data2 &= ~(0x01 << LSM303AGR_ACC_CTRL_REG4_HR_SHIFT);
	}
	else if (value == LSM303AGR_ACC_NORMAL_MODE)
	{
		data1 &= ~(0x01 << LSM303AGR_ACC_CTRL_REG1_LPen_SHIFT);
		data2 &= ~(0x01 << LSM303AGR_ACC_CTRL_REG4_HR_SHIFT);
	}
	else if (value == LSM303AGR_ACC_HIGH_RES_MODE)
	{
		data1 &= ~(0x01 << LSM303AGR_ACC_CTRL_REG1_LPen_SHIFT);
		data2 |= 0x01 << LSM303AGR_ACC_CTRL_REG4_HR_SHIFT;
	}
	else
	{
		retStat = LSM303AGR_ERROR;
	}

	if (HAL_I2C_Mem_Write(&hi2c1, deviceAddress, reg1Address, 1, &data1, 1, 10) == HAL_OK)
		retStat = LSM303AGR_OK;

	if (HAL_I2C_Mem_Write(&hi2c1, deviceAddress, reg4Address, 1, &data2, 1, 10) == HAL_OK)
		retStat = LSM303AGR_OK;

	return retStat;
}


/********************************************************************/
/* Get status from STATUS_REG_A register */
/********************************************************************/
uint8_t LSM303AGR_AccGetStatus()
{
	uint8_t retVal = 0;
	uint16_t deviceAddress = LSM303AGR_ACC_ADDR_READ;
	uint16_t registerAddress = LSM303AGR_STATUS_REG_A;

	HAL_I2C_Mem_Read(&hi2c1, deviceAddress, registerAddress, 1, &retVal, 1, 10);

	return retVal;
}


/********************************************************************/
/* Get accelerometer raw data */
/********************************************************************/
Acc_XYZ_t LSM303AGR_AccReadRawData()
{
	uint8_t buff[6];
	Acc_XYZ_t xyzRaw;
	uint16_t deviceAddress = LSM303AGR_ACC_ADDR_READ;
	uint16_t registerAddress = LSM303AGR_OUT_X_L_A;

	if (LSM303AGR_AccGetStatus() & (1 << LSM303AGR_ACC_ZYXDA_SHIFT) == LSM303AGR_ACC_ZYXDA)
	{
		HAL_I2C_Mem_Read(&hi2c1, deviceAddress, registerAddress, 1, buff, 6, 10);

		val[0] = (int16_t)buff[1];
		val[0] = (val[0] * 256) + (int16_t)buff[0];
		val[1] = (int16_t)buff[3];
		val[1] = (val[1] * 256) + (int16_t)buff[2];
		val[2] = (int16_t)buff[5];
		val[2] = (val[2] * 256) + (int16_t)buff[4];
	}

	xyzRaw.xAxis = (float)val[0];
	xyzRaw.yAxis = (float)val[1];
	xyzRaw.zAxis = (float)val[2];
}







