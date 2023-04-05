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
#include "MPU6050.h"
#include "math.h"
#define _USE_MATH_DEFINES
#include "micros.h"
#include <stdio.h>
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

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /*__GNUC__*/
PUTCHAR_PROTOTYPE
{

	HAL_UART_Transmit(&huart2,(uint8_t*)&ch, 1 ,0xFFFF);

	return ch;

}
int16_t Ac_X0, Ac_Y0, Ac_Z0, Gy_X0, Gy_Y0, Gy_Z0;
uint8_t MPU6050 = 0;
//#define MPU6050  MPU6050_DEFAULT_ADDRESS

void MPU6050_Write(uint8_t Address, uint8_t data){
  HAL_I2C_Mem_Write(&hi2c1, MPU6050, Address, 1, (uint8_t *)&data, 1, 10);
}

void MPU6050_Write_bits(uint8_t Address, uint8_t bitStart, uint8_t length, uint8_t data){
  uint8_t tmp = 0;
  HAL_I2C_Mem_Read(&hi2c1, MPU6050, Address, 1, (uint8_t *)&tmp, 1, 10);
  uint8_t mask = 0;
  switch(length){
    case 1: mask = 0x01; break;
    case 2: mask = 0x03; break;
    case 3: mask = 0x07; break;
    case 4: mask = 0x0F; break;
    case 5: mask = 0x1F; break;
    case 6: mask = 0x3F; break;
    case 7: mask = 0x7F; break;
    case 8: mask = 0xFF; break;
  }
  tmp &= ~(mask << bitStart);
  tmp |= (data << bitStart);
  HAL_I2C_Mem_Write(&hi2c1, MPU6050, Address, 1, (uint8_t *)&tmp, 1, 10);
}

uint8_t MPU6050_Read(uint8_t Address){
  uint8_t data;
  HAL_I2C_Mem_Read(&hi2c1, MPU6050, Address, 1, (uint8_t *)&data, 1, 10);
  return data;
}

void init_MPU6050(void){
  while(HAL_I2C_IsDeviceReady(&hi2c1, MPU6050, 10, 1000)!=HAL_OK) {
    MPU6050++;
  }
  printf("MPU6050 I2C Address is 0x%02X(7bit value)\r\n", MPU6050>>1);

  uint8_t temp = MPU6050_Read(MPU6050_RA_WHO_AM_I);
  printf("Who am I = 0x%02X\r\n", temp);
  printf("MPU6050 Initialize..... \r\n");
  printf("--------------------------------------------------------\r\n");

  HAL_Delay(100);
  /* Power Management 1, SLEEP Diasble*/
  MPU6050_Write_bits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, MPU6050_PWR1_SLEEP_LENGTH, DISABLE);
  HAL_Delay(10);
  /* Power Management 1, Internal 8MHz oscillator */
  MPU6050_Write_bits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_INTERNAL);
  /* Gyroscope Configuration, ± 250 °/s, 131 LSB/°/s */
  MPU6050_Write_bits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_GYRO_FS_250);
  /* Accelerometer Configuration, ± 2g, 16384 LSB/g */
  MPU6050_Write_bits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, MPU6050_ACCEL_FS_2);
}

void read_MPU6050_data(void){
  Ac_X0 = (MPU6050_Read(MPU6050_RA_ACCEL_XOUT_H)<<8) | MPU6050_Read(MPU6050_RA_ACCEL_XOUT_L);
  Ac_Y0 = (MPU6050_Read(MPU6050_RA_ACCEL_YOUT_H)<<8) | MPU6050_Read(MPU6050_RA_ACCEL_YOUT_L);
  Ac_Z0 = (MPU6050_Read(MPU6050_RA_ACCEL_ZOUT_H)<<8) | MPU6050_Read(MPU6050_RA_ACCEL_ZOUT_L);
  Gy_X0 = (MPU6050_Read(MPU6050_RA_GYRO_XOUT_H)<<8) | MPU6050_Read(MPU6050_RA_GYRO_XOUT_L);
  Gy_Y0 = (MPU6050_Read(MPU6050_RA_GYRO_YOUT_H)<<8) | MPU6050_Read(MPU6050_RA_GYRO_YOUT_L);
  Gy_Z0 = (MPU6050_Read(MPU6050_RA_GYRO_ZOUT_H)<<8) | MPU6050_Read(MPU6050_RA_GYRO_ZOUT_L);
}

int _write(int32_t file, uint8_t *ptr, int32_t len){
  HAL_UART_Transmit(&huart2, ptr, len, 10);
  return len;
}
//칼만 필터 관련 변수 및 함수
double Q_angle, Q_gyro, R_measure ;
double angle, bias ;
double P[2][2], K[2] ;
double deg,dgy_y;
double dt;
double val;
double getkalman(double acc, double gyro, double dt)
{
	//project the state ahead

	      angle += dt * (gyro - bias) ;

	      //Project the error covariance ahead

	      P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle) ;

	      P[0][1] -= dt * P[1][1] ;

	      P[1][0] -= dt * P[1][1] ;

	      P[1][1] += Q_gyro * dt ;



	      //Compute the Kalman gain

	      double S = P[0][0] + R_measure ;

	      K[0] = P[0][0] / S ;

	      K[1] = P[1][0] / S ;



	      //Update estimate with measurement z

	      double y = acc - angle ;

	      angle += K[0] * y ;

	      bias += K[1] * y ;



	      //Update the error covariance

	      double P_temp[2] = {P[0][0], P[0][1]} ;

	      P[0][0] -= K[0] * P_temp[0] ;

	      P[0][1] -= K[0] * P_temp[1] ;

	      P[1][0] -= K[1] * P_temp[0] ;

	      P[1][1] -= K[1] * P_temp[1] ;



	      return angle ;
}
double getvar(int num)
{
    switch (num) {
      case 0 :
    	  return Q_angle ;
    	  break ;

      case 1 :
    	  return Q_gyro ;
    	  break ;

      case 2 :
    	  return R_measure ;
    	  break ;

    }
}
void kal_init(double angle, double gyro, double measure)
{
	Q_angle = angle ;
	Q_gyro = gyro ;
	R_measure = measure ;

	angle = 0 ;
	bias = 0 ;

	P[0][0] = 0 ;
	P[0][1] = 0 ;
	P[1][0] = 0 ;
	P[1][1] = 0 ;
}



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
  uint32_t pasttime = 0;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  init_MPU6050();
  DWT_Init();
  kal_init(0.001, 0.003, 0.03);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  	 static uint16_t cnt=1;
	     HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
	     read_MPU6050_data();
	     deg = atan2(Ac_X0,Ac_Z0)*180/M_PI;// acc data to degree data
	     dgy_y = Gy_Y0/131.;				   // gyro output to

	     dt = (double)(micros()-pasttime)/1000000;
	     pasttime = micros();
	     val = getkalman(deg, dgy_y, dt);//get kalman data
	     printf("kalman degree : %5.2f , dt = %5.4f\r\n",val,dt);

	     HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
	     HAL_Delay(5);
	     if(++cnt>20) cnt=20;
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
