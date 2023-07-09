/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ACC_SENSOR_BUS hi2c1
#define GYRO_SENSOR_BUS hspi1
#include "lsm303agr_reg.h"
#include "i3g4250d_reg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct
{
  void   *hbus;
  uint8_t i2c_address;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
} sensbus_t;

#define BOOT_TIME             5 //ms

static sensbus_t xl_bus  = {&ACC_SENSOR_BUS,
                            LSM303AGR_I2C_ADD_XL,
                            0,
                            0
                           };
static sensbus_t mag_bus = {&ACC_SENSOR_BUS,
                            LSM303AGR_I2C_ADD_MG,
                            0,
                            0
                           };
static sensbus_t gyr_bus = {&GYRO_SENSOR_BUS,
                            0,
                            GPIOE,
                            GPIO_PIN_3
                           };


static int16_t data_raw_acceleration[3];
static int16_t data_raw_magnetic[3];
static int16_t data_raw_temperature;
static float acceleration_mg[3];
static float magnetic_mG[3];
static float temperature_degC;
static int16_t data_raw_angular_rate[3];
static float angular_rate_mdps[3];
static uint8_t whoamI, rst;


static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

void lsm303agr_init(stmdev_ctx_t *dev_ctx_xl, stmdev_ctx_t *dev_ctx_mg);
void lsm303agr_read(stmdev_ctx_t *dev_ctx_xl, stmdev_ctx_t *dev_ctx_mg);
void i3g4250d_init(stmdev_ctx_t *dev_ctx);
void i3g4250d_read(stmdev_ctx_t *dev_ctx);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  stmdev_ctx_t stmdev_ctx_xl;
  stmdev_ctx_t stmdev_ctx_mg;
  stmdev_ctx_t stmdev_ctx_gyr;
  lsm303agr_init(&stmdev_ctx_xl, &stmdev_ctx_mg);
  i3g4250d_init(&stmdev_ctx_gyr);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  lsm303agr_read(&stmdev_ctx_xl, &stmdev_ctx_mg);
	  i3g4250d_read(&stmdev_ctx_gyr);

	  HAL_Delay(100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void lsm303agr_init(stmdev_ctx_t *dev_ctx_xl, stmdev_ctx_t *dev_ctx_mg)
{
	/* Initialize mems driver interface */
	dev_ctx_xl->write_reg = platform_write;
	dev_ctx_xl->read_reg = platform_read;
	dev_ctx_xl->handle = (void *)&xl_bus;

	dev_ctx_mg->write_reg = platform_write;
	dev_ctx_mg->read_reg = platform_read;
	dev_ctx_mg->handle = (void *)&mag_bus;

	HAL_Delay(BOOT_TIME);

	/* Check device ID */
	whoamI = 0;
	lsm303agr_xl_device_id_get(dev_ctx_xl, &whoamI);

	if ( whoamI != LSM303AGR_ID_XL )
		while (1); /*manage here device not found */

	whoamI = 0;
	lsm303agr_mag_device_id_get(dev_ctx_mg, &whoamI);

	if ( whoamI != LSM303AGR_ID_MG )
		while (1); /*manage here device not found */

	/* Restore default configuration for magnetometer */
	lsm303agr_mag_reset_set(dev_ctx_mg, PROPERTY_ENABLE);

	do {
		lsm303agr_mag_reset_get(dev_ctx_mg, &rst);
	} while (rst);

	/* Enable Block Data Update */
	lsm303agr_xl_block_data_update_set(dev_ctx_xl, PROPERTY_ENABLE);
	lsm303agr_mag_block_data_update_set(dev_ctx_mg, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	lsm303agr_xl_data_rate_set(dev_ctx_xl, LSM303AGR_XL_ODR_1Hz);
	lsm303agr_mag_data_rate_set(dev_ctx_mg, LSM303AGR_MG_ODR_10Hz);
	/* Set accelerometer full scale */
	lsm303agr_xl_full_scale_set(dev_ctx_xl, LSM303AGR_2g);
	/* Set / Reset magnetic sensor mode */
	lsm303agr_mag_set_rst_mode_set(dev_ctx_mg, LSM303AGR_SENS_OFF_CANC_EVERY_ODR);
	/* Enable temperature compensation on mag sensor */
	lsm303agr_mag_offset_temp_comp_set(dev_ctx_mg, PROPERTY_ENABLE);
	/* Enable temperature sensor */
	lsm303agr_temperature_meas_set(dev_ctx_xl, LSM303AGR_TEMP_ENABLE);
	/* Set device in continuous mode */
	lsm303agr_xl_operating_mode_set(dev_ctx_xl, LSM303AGR_HR_12bit);
	/* Set magnetometer in continuous mode */
	lsm303agr_mag_operating_mode_set(dev_ctx_mg, LSM303AGR_CONTINUOUS_MODE);
}

void lsm303agr_read(stmdev_ctx_t *dev_ctx_xl, stmdev_ctx_t *dev_ctx_mg)
{
	/* Read output only if new value is available */
	lsm303agr_reg_t reg;
	lsm303agr_xl_status_get(dev_ctx_xl, &reg.status_reg_a);

	if (reg.status_reg_a.zyxda)
	{
	  /* Read accelerometer data */
	  memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
	  lsm303agr_acceleration_raw_get(dev_ctx_xl, data_raw_acceleration);
	  acceleration_mg[0] = lsm303agr_from_fs_2g_hr_to_mg(data_raw_acceleration[0]);
	  acceleration_mg[1] = lsm303agr_from_fs_2g_hr_to_mg(data_raw_acceleration[1]);
	  acceleration_mg[2] = lsm303agr_from_fs_2g_hr_to_mg(data_raw_acceleration[2]);
	}

	lsm303agr_mag_status_get(dev_ctx_mg, &reg.status_reg_m);

	if (reg.status_reg_m.zyxda)
	{
	  /* Read magnetic field data */
	  memset(data_raw_magnetic, 0x00, 3 * sizeof(int16_t));
	  lsm303agr_magnetic_raw_get(dev_ctx_mg, data_raw_magnetic);
	  magnetic_mG[0] = lsm303agr_from_lsb_to_mgauss(data_raw_magnetic[0]);
	  magnetic_mG[1] = lsm303agr_from_lsb_to_mgauss(data_raw_magnetic[1]);
	  magnetic_mG[2] = lsm303agr_from_lsb_to_mgauss(data_raw_magnetic[2]);
	}

	lsm303agr_temp_data_ready_get(dev_ctx_xl, &reg.byte);

	if (reg.byte)
	{
	  /* Read temperature data */
	  memset(&data_raw_temperature, 0x00, sizeof(int16_t));
	  lsm303agr_temperature_raw_get(dev_ctx_xl, &data_raw_temperature);
	  temperature_degC = lsm303agr_from_lsb_hr_to_celsius(data_raw_temperature);
	}
}

void i3g4250d_init(stmdev_ctx_t *dev_ctx)
{
	/* Uncomment to use interrupts on drdy */
	//i3g4250d_int2_route_t int2_reg;
	/* Initialize mems driver interface */
	dev_ctx->write_reg = platform_write;
	dev_ctx->read_reg = platform_read;
	dev_ctx->handle = &gyr_bus;
	/* Initialize platform specific hardware */
	HAL_Delay(BOOT_TIME);
	HAL_Delay(BOOT_TIME);
	/* Check device ID */
	whoamI = 0;
	i3g4250d_device_id_get(dev_ctx, &whoamI);

	if (whoamI != I3G4250D_ID)
	while (1); /*manage here device not found */

	/* Configure filtering chain -  Gyroscope - High Pass */
	i3g4250d_filter_path_set(dev_ctx, I3G4250D_LPF1_HP_ON_OUT);
	i3g4250d_hp_bandwidth_set(dev_ctx, I3G4250D_HP_LEVEL_3);
	/* Uncomment to use interrupts on drdy */
	//i3g4250d_pin_int2_route_get(&dev_ctx, &int2_reg);
	//int2_reg.i2_drdy = PROPERTY_ENABLE;
	//i3g4250d_pin_int2_route_set(&dev_ctx, int2_reg);
	/* Set Output Data Rate */
	i3g4250d_data_rate_set(dev_ctx, I3G4250D_ODR_100Hz);
}

void i3g4250d_read(stmdev_ctx_t *dev_ctx)
{
	uint8_t reg = 0;
	/* Read output only if new value is available */
	i3g4250d_flag_data_ready_get(dev_ctx, &reg);

	if (reg) {
	  /* Read angular rate data */
	  memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
	  i3g4250d_angular_rate_raw_get(dev_ctx, data_raw_angular_rate);
	  angular_rate_mdps[0] = i3g4250d_from_fs245dps_to_mdps(
							   data_raw_angular_rate[0]);
	  angular_rate_mdps[1] = i3g4250d_from_fs245dps_to_mdps(
							   data_raw_angular_rate[1]);
	  angular_rate_mdps[2] = i3g4250d_from_fs245dps_to_mdps(
							   data_raw_angular_rate[2]);
	}
}

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{

	sensbus_t *sensbus = (sensbus_t *)handle;

	if ((sensbus->i2c_address == 0) && (sensbus->cs_port == GPIOE))
	{
		/* Write multiple command */
		reg |= 0x40;
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_SPI_Transmit(handle, &reg, 1, 1000);
		HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
	}
	else
	{
		if (sensbus->i2c_address == LSM303AGR_I2C_ADD_XL) {
		    /* enable auto incremented in multiple read/write commands */
			reg |= 0x80;
		}

		  HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg,
		                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
	}


	return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{

	sensbus_t *sensbus = (sensbus_t *)handle;

	if ((sensbus->i2c_address == 0) && (sensbus->cs_port == GPIOE))
	{
		/* Read multiple command */
//		reg |= 0xC0;
		reg |= 0x80;
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, &reg, 1, 200);
		HAL_SPI_Receive(&hspi1, bufp, len, 200);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
	}
	else
	{
		if (sensbus->i2c_address == LSM303AGR_I2C_ADD_XL) {
		//    /* enable auto incremented in multiple read/write commands */
		    reg |= 0x80;
		}

		  HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg,
		                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	}

	return 0;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
