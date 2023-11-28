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
#include "stdio.h"
#include "stdint.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	uint16_t voltage_src;
	uint16_t shunt_voltage;
	uint16_t bus_voltage;
	uint16_t current;
	uint16_t power; //Performance (hieu suat)
}INA219_Measurement;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_CONV_RANGE 3.6 //(Conversion voltage range of ADC) Callib value based on which STM32 used
#define MAX_ADC_RESOLUTION_VAL 4095 //STM32F103C8T6 ADC 12 bits
#define ACCURACY_TEMPERATURE_CELCIUS 100 //10mv/Celcius based on LM35 datasheet
//Temperature Sensor Datasheet: extension://bfdogplmndidlpjfhoijckpakkdjkkil/pdf/viewer.html?file=https%3A%2F%2Fwww.ti.com%2Flit%2Fds%2Fsymlink%2Flm35.pdf

//////////// INA219 ////////////
//Define device address
#define INA219_SLAVE_ADDRESS 0x40<<1
//Define i2c protocol sensor register address - BEGIN
#define INA219_CONFIG_REG 0x007
#define INA219_SHUNTVOLTAGE_REG 0x01
#define INA219_BUSVOLTAGE_REG 0x02
#define INA219_POWER_REG 0x03
#define INA219_CURRENT_REG 0x04
#define INA219_CALLIB_REG 0x05
//Define i2c protocol sensor register address - END

///////////////////////////////
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t raw_temperature_val = 0; //digital value output value from adc
float voltage_temperature_convert = 0; //Voltage analog value convert
float temperature = 0; //temperature after processing
uint8_t Tx_Buffer[100]; //UART buffer

INA219_Measurement ina_basic_measure;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void Ina219_Init(void);
uint16_t eightbit_to_16bit_conv(uint8_t num1,uint8_t num2);
void Ina219_Read_Value(INA219_Measurement *pTarget);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Ina219_Init(void)
{
	uint8_t buffer[3];
	buffer[0] = INA219_CALLIB_REG;
	buffer[1] = 0x01; //MSB callib reg
	buffer[2] = 0x9A; //LSB callib reg
	//init ina219 callibration
	//28ms is time out value based on ina219 datasheet corresponding to SMBus
	if(HAL_I2C_Master_Transmit(&hi2c1, INA219_SLAVE_ADDRESS,buffer,3,28) != HAL_OK) //that ra neu de noi ma init successfully thi phai check gia tri duoc ghi co that sư ghi chua
	{
		 memset(Tx_Buffer,0,sizeof(Tx_Buffer)); //clear buffer before write
		 sprintf((char*)Tx_Buffer,"INA219: I2C TRANSMIT CALLIB SUCCESSFULLY!\n");
		 HAL_Delay(100);
		 memset(Tx_Buffer,0,sizeof(Tx_Buffer)); //clear buffer before write
		 sprintf((char*)Tx_Buffer,"Ready to read data...\n");
		 HAL_Delay(100);
	}
	else {
		memset(Tx_Buffer,0,sizeof(Tx_Buffer)); //clear buffer before write
		sprintf((char*)Tx_Buffer,"INA219: TRANSMIT CALLIB FAILED!\n");
		HAL_Delay(100);
	}
}

void Ina219_Read_Value(INA219_Measurement *pTarget)
{
	uint8_t current_val_buffer[2];
	uint8_t bus_voltage_val_buffer[2];
	uint8_t shuntvoltage_val_buffer[2];
	uint8_t power_val_buffer[2];

	//bus voltage handle
	HAL_I2C_Master_Transmit(&hi2c1, INA219_SLAVE_ADDRESS,(uint8_t*)INA219_CURRENT_REG,1,28);
	HAL_I2C_Master_Receive(&hi2c1, INA219_SLAVE_ADDRESS,current_val_buffer,2,28);

	//bus voltage handle
	HAL_I2C_Master_Transmit(&hi2c1, INA219_SLAVE_ADDRESS,(uint8_t*)INA219_BUSVOLTAGE_REG,1,28);
	HAL_I2C_Master_Receive(&hi2c1, INA219_SLAVE_ADDRESS,bus_voltage_val_buffer,2,28);

	//power handle
	HAL_I2C_Master_Transmit(&hi2c1, INA219_SLAVE_ADDRESS,(uint8_t*)INA219_POWER_REG,1,28);
	HAL_I2C_Master_Receive(&hi2c1, INA219_SLAVE_ADDRESS,power_val_buffer,2,28);

	//shunt voltage handle - may need to be calculated more to get accurate result with 2's complement
	HAL_I2C_Master_Transmit(&hi2c1, INA219_SLAVE_ADDRESS,(uint8_t*)INA219_SHUNTVOLTAGE_REG,1,28);
	HAL_I2C_Master_Receive(&hi2c1, INA219_SLAVE_ADDRESS,shuntvoltage_val_buffer,2,28);

	pTarget->current = eightbit_to_16bit_conv(current_val_buffer[0], current_val_buffer[1]);
	pTarget->bus_voltage = eightbit_to_16bit_conv(bus_voltage_val_buffer[0], bus_voltage_val_buffer[1]);
	pTarget->shunt_voltage = eightbit_to_16bit_conv(shuntvoltage_val_buffer[0], shuntvoltage_val_buffer[1]);
	pTarget->power = eightbit_to_16bit_conv(power_val_buffer[0], power_val_buffer[1]);

	pTarget->voltage_src = pTarget->shunt_voltage + pTarget->bus_voltage;
}

uint16_t eightbit_to_16bit_conv(uint8_t num1,uint8_t num2)
{
	uint16_t result = (num1 << 8) | num2;
	return result;
}




void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
 {
 	if(hadc->Instance == hadc1.Instance)
 	{
 		raw_temperature_val = HAL_ADC_GetValue(&hadc1);
 	}
 	//if the continous ADC read disable we will start again here
 	//HAL_ADC_Start_IT(&hadc1);
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
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_IT(&hadc1);
  Ina219_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  ////////////LM35 - TEMPERATURE SENSOR //////////////
	  //calculation to get temperature
	  //voltage_temperature_convert = (float)raw_temperature_val*MAX_CONV_RANGE/MAX_ADC_RESOLUTION_VAL;
	  //temperature = voltage_temperature_convert*100;
	  ////////////LM35 - TEMPERATURE SENSOR //////////////

	  ////////////INA219 - TEMPERATURE SENSOR //////////////
	  //ina219_Read_Value(&ina_basic_measure);
	  ////////////INA219 - TEMPERATURE SENSOR //////////////

	  ////////////DEBUG OLED SSD1306 - ESP32 THROUGH UART //////////////
	  //memset(Tx_Buffer,0,sizeof(Tx_Buffer)); //clear buffer before write
	  //sprintf((char*)Tx_Buffer,"Temp: %.2f C",temperature);
	  //HAL_UART_Transmit(&huart1,Tx_Buffer,sizeof(Tx_Buffer), 10);
	  //HAL_Delay(100); //each 500ms display temperature once
	  ////////////DEBUG OLED SSD1306 - ESP32 THROUGH UART //////////////
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
	  memset(Tx_Buffer,0,sizeof(Tx_Buffer)); //clear buffer before write
	  sprintf((char*)Tx_Buffer,"System init error\n");
	  HAL_Delay(500);
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
