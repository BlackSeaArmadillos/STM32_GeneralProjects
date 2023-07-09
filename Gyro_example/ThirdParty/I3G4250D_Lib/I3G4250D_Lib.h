/*
 * I3G4250D_Lib.h
 *
 *  Created on: Jun 20, 2023
 *      Author: legionarul
 *
 *      The driver supports SPI protocol
 *      Write and read signle and multiple register supported
 *
 *      TODO: I2C support
 */

#ifndef I3G4250D_LIB_I3G4250D_LIB_H_
#define I3G4250D_LIB_I3G4250D_LIB_H_

// Includes
#include "stm32f4xx_hal.h"
#include "spi.h"
#include "gpio.h"
#include "string.h"


// Registers
#define I3G4250D_ID                      0xD3U
#define I3G4250D_WHO_AM_I                0x0FU
#define I3G4250D_CTRL_REG1               0x20U
#define I3G4250D_CTRL_REG2               0x21U
#define I3G4250D_CTRL_REG3               0x22U
#define I3G4250D_CTRL_REG4               0x23U
#define I3G4250D_CTRL_REG5               0x24U
#define I3G4250D_REFERENCE               0x25U
#define I3G4250D_OUT_TEMP                0x26U
#define I3G4250D_STATUS_REG              0x27U

#define I3G4250D_OUT_X_L                 0x28U
#define I3G4250D_OUT_X_H                 0x29U
#define I3G4250D_OUT_Y_L                 0x2AU
#define I3G4250D_OUT_Y_H                 0x2BU
#define I3G4250D_OUT_Z_L                 0x2CU
#define I3G4250D_OUT_Z_H                 0x2DU

#define I3G4250D_FIFO_CTRL_REG           0x2EU
#define I3G4250D_FIFO_SRC_REG            0x2FU
#define I3G4250D_INT1_CFG                0x30U
#define I3G4250D_INT1_SRC                0x31U
#define I3G4250D_INT1_TSH_XH             0x32U
#define I3G4250D_INT1_TSH_XL             0x33U
#define I3G4250D_INT1_TSH_YH             0x34U
#define I3G4250D_INT1_TSH_YL             0x35U
#define I3G4250D_INT1_TSH_ZH             0x36U
#define I3G4250D_INT1_TSH_ZL             0x37U
#define I3G4250D_INT1_DURATION           0x38U

/* Enable gyroscope macros */
#define I3G4250D_ODR_OFF				 0x00
#define I3G4250D_ODR_SLEEP               0x08
#define I3G4250D_ODR_100Hz				 0x0F
#define I3G4250D_ODR_200Hz				 0x1F
#define I3G4250D_ODR_400Hz				 0x2F
#define I3G4250D_ODR_800Hz				 0x3F

/* Configure High-pass filter */
#define I3G4250D_HP_NORMAL_MODE_WITH_RST 	0
#define I3G4250D_HP_REFERENCE_SIGNAL     	1
#define I3G4250D_HP_NORMAL_MODE          	2
#define I3G4250D_HP_AUTO_RESET_ON_INT    	3

#define I3G4250D_HPM_SHIFT				0x04

#define I3G4250D_HPCF_LEVEL_0   			 0
#define I3G4250D_HPCF_LEVEL_1   			 1
#define I3G4250D_HPCF_LEVEL_2   			 2
#define I3G4250D_HPCF_LEVEL_3   			 3
#define I3G4250D_HPCF_LEVEL_4   		 	 4
#define I3G4250D_HPCF_LEVEL_5   			 5
#define I3G4250D_HPCF_LEVEL_6   			 6
#define I3G4250D_HPCF_LEVEL_7   			 7
#define I3G4250D_HPCF_LEVEL_8   			 8
#define I3G4250D_HPCF_LEVEL_9   			 9


/* Full scale selection macros */
#define I3G4250D_245dps     			 0x00
#define I3G4250D_500dps     			 0x01
#define I3G4250D_2000dps    			 0x02

/* Configure output selection and high-pass filter */
#define I3G4250D_ONLY_LPF1_ON_OUT     	 0
#define I3G4250D_LPF1_HP_ON_OUT       	 1
#define I3G4250D_LPF1_LPF2_ON_OUT     	 2
#define I3G4250D_LPF1_HP_LPF2_ON_OUT  	 6

#define I3G4250D_NORMAL					 0x01
#define I3G4250D_NORMAL_MASK			 0x08
#define I3G4250D_NORMAL_SHIFT			 0x03

#define I3G4250D_POWER_DOWN              0x00
#define I3G4250D_POWER_DOWN_SHIFT		 0x03
#define I3G4250D_POWER_DOWN_MASK		 0x00

#define I3G4250D_SLEEP					 0x08
#define I3G4250D_SLEEP_MASK				 0x08
#define I3G4250D_SLEEP_CLR_MASK		 	 0xF8
#define I3G4250D_SLEEP_SHIFT			 0x03

/* Status register macros */
#define I3G4250D_ZYXDA					 0x08
#define I3G4250D_ZDA					 0x04
#define I3G4250D_YDA					 0x02
#define I3G4250D_XDA					 0x01


typedef enum
{
	I3G4250D_OK,
	I3G4250D_NOT_OK
} I3G4250D_ret_t;

typedef enum
{
	I3G4250D_STATUS_POWER_DOWN = 0,
	I3G4250D_STATUS_NORMAL = 1,
	I3G4250D_STATUS_SLEEP = 2
} I3G4250D_power_mode_status_t;

typedef struct
{
	int16_t xAxis;
	int16_t yAxis;
	int16_t zAxis;
} Gyro_XYZ_t;

void I3G4250D_RegisterWriteMultiple(uint8_t address, uint8_t * pData);
void I3G4250D_RegisterReadMultiple(uint8_t address, uint8_t ret_value);
void I3G4250D_RegisterWrite(uint8_t address, uint8_t value);
uint8_t I3G4250D_RegisterRead(uint8_t address);

uint8_t I3G4250D_WhoIAm();
void I3G4250D_Set_CtrlReg1(uint8_t value);
uint8_t I3G4250D_Get_CtrlReg1();
void I3G4250D_Set_CtrlReg2(uint8_t value);
uint8_t I3G4250D_Get_CtrlReg2();
void I3G4250D_Set_CtrlReg3(uint8_t value);
uint8_t I3G4250D_Get_CtrlReg3();
void I3G4250D_Set_CtrlReg4(uint8_t value);
uint8_t I3G4250D_Get_CtrlReg4();
void I3G4250D_Set_CtrlReg5(uint8_t value);
uint8_t I3G4250D_Get_CtrlReg5();
void I3G4250D_Set_Reference(uint8_t value);
uint8_t I3G4250D_Get_Reference();
uint8_t I3G4250D_Get_OutTemp();
uint8_t I3G4250D_Get_StatusReg();
uint8_t I3G4250D_Get_OutXL();
uint8_t I3G4250D_Get_OutXH();
uint8_t I3G4250D_Get_OutYL();
uint8_t I3G4250D_Get_OutYH();
uint8_t I3G4250D_Get_OutZL();
uint8_t I3G4250D_Get_OutZH();

void I3G4250D_SetPowerMode(uint8_t value);
uint8_t I3G4250D_GetPowerMode();

void I3G4250D_GyroEnable(uint8_t value);
void I3G4250D_HPFilterConfig(uint8_t hpfMode, uint8_t hpfCutOff);
void I3G4250D_InterruptConfig(uint8_t value);
void I3G4250D_SetFullScale(uint8_t value);
void I3G4250D_SetSPIMode(uint8_t value);
void I3G4250D_SetEndianess(uint8_t value);
void I3G4250D_SelfTestEnable(uint8_t value);
void I3G4250D_RebootMem(uint8_t value);
void I3G4250D_OutSelConfig(uint8_t value);

Gyro_XYZ_t I3G4250D_ReadGyroXYZ();

#endif /* I3G4250D_LIB_I3G4250D_LIB_H_ */
