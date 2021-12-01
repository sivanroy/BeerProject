/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_mems.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "iks01a3_motion_sensors.h"
#include "iks01a3_env_sensors.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct displayFloatToInt_s {
  int8_t sign; /* 0 means positive, 1 means negative*/
  uint32_t  out_int;
  uint32_t  out_dec;
} displayFloatToInt_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_BUF_SIZE	256
#define BUFFER_SIZE		FLASH_PAGE_SIZE/4

// Analog loopback variables
#define DELTA_MAX       0.1
#define VDD             3.3


#ifdef MULTI_SENSOR_ENABLE
#define DATA_SIZE				16
#define FLOAT_CONVERSION		0
#define LIS2DW12_ODR			5       // Output data rate (HP = High-perf., LP = Low-power) (0 power down, 1 HP 12.5Hz/LP 1.6Hz, 2 for 12.5Hz, 3 for 25Hz, 4 for 50Hz, 5 for 100Hz, 6 for 200Hz, 7 for HP 400Hz/LP 200Hz, 8 for HP 800Hz/LP 200Hz, 9 for HP 1600Hz/LP 200Hz)
#define LIS2DW12_FS         	2      	// Full-scale +-(2, 4, 8 or 16 g)
#define LIS2DW12_BW_FILT        0       // Filter bandwidth (0 for ODR/2, 1 for ODR/4, 2 for ODR/10, 3 for ODR/20)
#define LIS2DW12_MODE           0       // Mode (0 for low-power, 1 for high-performance, 2 for single data conversion)
#define LIS2DW12_LP_MODE        4       // Low-power modes 1 to 4 (1 gives the max. rms noise, 4 gives the min. rms noise)
#define LIS2DW12_LOW_NOISE      0       // Low-noise (0 disabled, 1 enabled)
#define LIS2DW12_CTRL1_VAL		(LIS2DW12_ODR << 4) + (LIS2DW12_MODE << 2) + LIS2DW12_LP_MODE
#define LIS2DW12_CTRL6_VAL     	(LIS2DW12_LOW_NOISE << 2)
#define LIS2DW12_CTRL6_MASK		0xFB
#endif
//#define DATA_SIZE_WORD			DATA_SIZE/4
#define DATA_SIZE_WORD		DATA_SIZE/2


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
// Print and read buffer
static char dataOut[MAX_BUF_SIZE];
char dataIn[2];
// Timer IRQ
extern int TIM2_IRQ_FLAG;
// Sensors
extern void *EnvCompObj[IKS01A3_ENV_INSTANCES_NBR]; /* This "redundant" line is here to fulfil MISRA C-2012 rule 8.4 */
void *EnvCompObj[IKS01A3_ENV_INSTANCES_NBR];
extern void *MotionCompObj[IKS01A3_MOTION_INSTANCES_NBR]; /* This "redundant" line is here to fulfil MISRA C-2012 rule 8.4 */
void *MotionCompObj[IKS01A3_MOTION_INSTANCES_NBR];
// Button handling
static volatile uint8_t ButtonPressed;
static volatile uint8_t SaveData;
uint32_t data_ind;
uint32_t data_buffer[BUFFER_SIZE];
uint32_t Flash_addr;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */


static void read_task(void);



#ifdef MULTI_SENSOR_ENABLE
static void Multi_Sensor_Config(void);
static void Multi_Sensor_Handler(int save, int verbose);
#endif

void print_flash_info();
int erase_flash(int verbose);
int write_flash(uint32_t Flash_addr, uint32_t* Flash_wdata, int32_t n_words, int verbose);
void read_flash(uint32_t Flash_addr, uint32_t* Flash_rdata, uint32_t n_bytes);

static void floatToInt(float in, displayFloatToInt_t *out_value, int32_t dec_prec);
uint32_t FloatToUint(float n);
float UintToFloat(uint32_t n);
float pressureToAltitude(float pressure);

int _read(int file, char *result, size_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */


// CHANGE PARAMETERS
// USER PARAMETERS CAN BE SET
#define PASSED 				10
//#define MARGE 				50 //in percent
#define NUMBER_OF_SAMPLES	10
#define NUMBER_OF_BEERS		3
#define ZMARGE				3000

//GLOBAL VARIABLES for states
int Dormancy    = -1;
int WTBR		= 0;
//int save = 0; already set further

//X passed AccZ data
uint32_t PassedAccZ[DATA_SIZE_WORD*PASSED];

//Counters
int CountPassed		= 0;
int CountSamples	= 0;
int CountBeers		= 0;


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
  MX_ADC_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_MEMS_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  /* Turn off 1kB buffer for printf/scanf */
  setvbuf(stdin, NULL, _IONBF, 0);

  /* Timer */
  HAL_TIM_Base_Start_IT(&htim2);
  TIM2_IRQ_FLAG = 0;

  /* Button */
  ButtonPressed = 0;
  SaveData = 0;

  /* Flash Memory info */
  print_flash_info();

  /* Sensor configuration */
  #ifdef MULTI_SENSOR_ENABLE
    Multi_Sensor_Config();
  #endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //printf("%d\t%d\r\n",CountBeers,CountSamples);
    //printf("%d\t%d\t%d\r\n",Dormancy,SaveData,WTBR);

    /* Button handling */
    if (ButtonPressed) {
    	ButtonPressed = 0;
    	if(Dormancy == -1) {
			ButtonPressed = 0;
			SaveData = 0;Dormancy = 1;WTBR = 0;

			CountPassed = 0;
			CountBeers = 0;
			CountSamples = 0;

			erase_flash(0);
			printf("Acquisition started...\r\n");
			printf("Press blue button to stop data acquisition\r\n");
			data_ind = 0;
			Flash_addr = FLASH_BANK2_BASE;
    	}
    	else {
            SaveData = 0;Dormancy = -1;WTBR = 0; //
        	printf("Acquisition stopped...\r\n");
        	printf("Press 'R' to read the data\r\n");
        	if (data_ind != 0) {
        	  write_flash(Flash_addr, &data_buffer[0], data_ind, 0);
        	}
        }

    }


    /* Read task */
    if (Dormancy == -1) {
      if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != 0) {
        scanf("%s", dataIn);
      }

      if ((strcmp(dataIn, "r") == 0) || (strcmp(dataIn, "R") == 0)) {
        read_task();
      }

      /* Reset user command buffer */
      memset(dataIn, '\0', sizeof(dataIn));
    }

    /* Acquisition task */
    if (TIM2_IRQ_FLAG && (Dormancy != -1) && (Flash_addr <= FLASH_BANK2_END-DATA_SIZE+1)) {
      /* Release timer flag */
      TIM2_IRQ_FLAG = 0;

      /* LPS22HH pressure sensor */

      /* Multiple sensors */
	  #ifdef MULTI_SENSOR_ENABLE
        Multi_Sensor_Handler(1, 0);
	  #endif


    }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1; //change adc value?

  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3200;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1; //change to have PA0
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB10 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
/* Analog loopback task */

/* Read task */
static void read_task(void)
{
  uint32_t Flash_rdata[DATA_SIZE_WORD];
  uint8_t flash_empty;
  displayFloatToInt_t int_data[DATA_SIZE_WORD];
  uint8_t count = 0;

  /* Data labels */
  #ifdef MULTI_SENSOR_ENABLE
    printf("accz\tvout\r\n");
  #endif

  /* Read 1st Flash data */
  uint32_t Flash_addr_temp = FLASH_BANK2_BASE;
  read_flash(Flash_addr_temp, &Flash_rdata[0], DATA_SIZE);

  /* Read Flash data */
  flash_empty = 0;
  while ((Flash_addr_temp <= FLASH_BANK2_END-DATA_SIZE+1) && !flash_empty) {
	if (FLOAT_CONVERSION) {
	  for (int i=0; i<DATA_SIZE_WORD; i++) {
        floatToInt(UintToFloat(Flash_rdata[i]), &int_data[i], 6);
      }
	}

    /* Print data to console */

    #ifdef MULTI_SENSOR_ENABLE
      /* Convert ADC data to int */
      //floatToInt(UintToFloat(Flash_rdata[3]), &int_data[0], 6);
      //snprintf(dataOut, MAX_BUF_SIZE, "%d\t%d\t%d\t%d.%06d\r\n", (int) Flash_rdata[0], (int) Flash_rdata[1], (int) Flash_rdata[2], (int) int_data[0].out_int, (int) int_data[0].out_dec); //valeures stockées
      floatToInt(UintToFloat(Flash_rdata[1]), &int_data[0], 6);
      snprintf(dataOut, MAX_BUF_SIZE, "%d\t%d.%06d\r\n", (int) Flash_rdata[0], (int) int_data[0].out_int, (int) int_data[0].out_dec); //valeures stockées
    #endif

    printf("%s", dataOut);

    /* Increment Flash address */
    Flash_addr_temp += DATA_SIZE;

    /* Check if the next address is not empty (erased Flash only contains 0) */
    if (Flash_addr_temp <= FLASH_BANK2_END-DATA_SIZE+1) {
      read_flash(Flash_addr_temp, &Flash_rdata[0], DATA_SIZE);
      count = 0;
      for (int i=0; i<DATA_SIZE_WORD; i++) {
        if (Flash_rdata[i] == 0) {
          count++;
        }
      }
      if (count == DATA_SIZE_WORD) {
        flash_empty = 1;
      }
    }
  }
}

/* LPS22HH pressure/temperature sensor */


/* LIS2MDL magnetometer sensor */


/* LIS2DW12 accelerometer sensor */


/* Custom sensor */


/* Multiple sensors */
#ifdef MULTI_SENSOR_ENABLE
static void Multi_Sensor_Config(void)
{
  uint8_t ReadData, WriteData;
  LIS2DW12_ACC_SetFullScale(MotionCompObj[IKS01A3_LIS2DW12_0], LIS2DW12_FS);
  LIS2DW12_ACC_Set_Filter_Mode(MotionCompObj[IKS01A3_LIS2DW12_0], LIS2DW12_BW_FILT);

  // CTRL 1 register
  LIS2DW12_Write_Reg(MotionCompObj[IKS01A3_LIS2DW12_0], (uint8_t) 0x20, LIS2DW12_CTRL1_VAL);
  LIS2DW12_Read_Reg(MotionCompObj[IKS01A3_LIS2DW12_0], (uint8_t) 0x20, &ReadData);
  printf("LIS2DW12 reg 0x%x (read): 0x%x\r\n", (uint8_t) 0x20, ReadData);

  // CTRL 6 register
  LIS2DW12_Read_Reg(MotionCompObj[IKS01A3_LIS2DW12_0], (uint8_t) 0x25, &ReadData);
  WriteData = (ReadData & LIS2DW12_CTRL6_MASK) + LIS2DW12_CTRL6_VAL;
  LIS2DW12_Write_Reg(MotionCompObj[IKS01A3_LIS2DW12_0], (uint8_t) 0x25, WriteData);
  LIS2DW12_Read_Reg(MotionCompObj[IKS01A3_LIS2DW12_0], (uint8_t) 0x25, &ReadData);
  printf("LIS2DW12 reg 0x%x (read): 0x%x\r\n", (uint8_t) 0x25, ReadData);
}

static void Multi_Sensor_Handler(int save, int verbose)
{
  /* Initialize variables */
  int32_t int_data[DATA_SIZE_WORD];
  LIS2DW12_AxesRaw_t acc_axes;
  uint32_t adcValue;
  float in_value;
  displayFloatToInt_t float_in;

  /* Read sensors */
  if (LIS2DW12_ACC_GetAxesRaw(MotionCompObj[IKS01A3_LIS2DW12_0], &acc_axes) != LIS2DW12_ERROR) {
	//int_data[0] = acc_axes.x;
	//int_data[1] = acc_axes.y;
	//int_data[2] = acc_axes.z;
	  int_data[0] = acc_axes.z;
  }

  /* Read value on ADC1/0 */
  HAL_ADC_Start(&hadc);
  if (HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY) == HAL_OK)
  {
    adcValue = HAL_ADC_GetValue(&hadc);
  }
  else
  {
    adcValue = 0;
  }
  in_value = ((float) adcValue)/4095 * VDD;
  //int_data[3] = FloatToUint(in_value);
  int_data[1] = FloatToUint(in_value);

  //add sample to passed samples
  if (CountPassed<PASSED){
	  CountPassed ++;
  }
  for (int i=CountPassed-1; i>0; i--) {
	  PassedAccZ[i] = PassedAccZ[i-1];
  }
  PassedAccZ[0] = int_data[0];

  int mean = 0;
  //compute mean of passedAccZ
  if(CountPassed == PASSED) {
	  for (int i = 0 ; i<PASSED; i++) {
		  mean += PassedAccZ[i];
	  }
	  mean = mean/PASSED;
  }

  if(WTBR == 1) {
	  if(CountBeers == NUMBER_OF_BEERS) {
		  Dormancy = -1;
		  SaveData = 0;
		  WTBR = 0;
	  }
	  else if (mean > ZMARGE) {// MAKE ON DORMANCY
		  printf("Make on dormancy \r\n");
		  Dormancy = 1;
		  SaveData = 0;
		  WTBR = 0;
	  }
  }

  else if(Dormancy==1) {
	  //printf('in dormancy');
	  if (mean < -ZMARGE) {// MAKE OFF DORMANCY
		  printf("Make off dormancy \r\n");
		  Dormancy = 0;
		  SaveData = 1;
		  WTBR = 0;
		  CountBeers++;
		  printf("CountBeers = %d\n\r",CountBeers);
	  }
  }

  else {
	  if (SaveData) {
		write_flash(Flash_addr, (uint32_t*) &int_data[0], DATA_SIZE_WORD, 0);
		Flash_addr += DATA_SIZE;

		CountSamples ++;
		if (CountSamples == NUMBER_OF_SAMPLES) { //WTBR ON
			printf("WTBR is on\r\n");
			Dormancy = 0;
			WTBR = 1;
			SaveData = 0;

			CountSamples = 0;
		}
	  }
	  //printf("Here is an error \; \r\n");
  }
  /* Print data */
  if (verbose) {
    /* Data conversion to float representation */
    floatToInt(in_value, &float_in, 3);
	sprintf(dataOut, "%d\t%d\t%d\t%d.%03d\r\n", (int) acc_axes.x, (int) acc_axes.y, (int) acc_axes.z, (int) float_in.out_int, (int) float_in.out_dec);
    printf("%s ;;; not to date", dataOut);
  }
}
#endif

/* Print Flash memory info */
void print_flash_info()
{
    printf("**************************************************\n\r");
    printf("/***** Flash memory info *****/\r\n");
    printf("Flash size: %d [B]\r\n", (int) FLASH_SIZE);
    printf("Flash page size: %d [B]\r\n", (int) FLASH_PAGE_SIZE);
    printf("Flash nb of pages: %d \r\n", (int) round(FLASH_SIZE/FLASH_PAGE_SIZE));
    printf("Flash bank 1 base address: 0x%X\r\n", (int) FLASH_BASE);
    printf("Flash bank 1 end address: 0x%X\r\n", (int) FLASH_BANK1_END);
    printf("Flash bank 2 base address: 0x%X\r\n", (int) FLASH_BANK2_BASE);
    printf("Flash bank 2 end address: 0x%X\r\n", (int) FLASH_BANK2_END);
    printf("**************************************************\n\r");
}

/* Erase content of Flash memory */
int erase_flash(int verbose)
{
    printf("Erasing Flash memory...\r\n");

    /* Unlock Flash memory */
    HAL_FLASH_Unlock();

    /* Erase Flash memory */
    FLASH_EraseInitTypeDef eraser;
    uint32_t Flash_addr = FLASH_BANK2_BASE;
    uint32_t page_error = 0;
    int32_t page = 1;

    while (Flash_addr < FLASH_BANK2_END) {
        eraser.TypeErase = FLASH_TYPEERASE_PAGES;
        eraser.PageAddress = Flash_addr;
        eraser.NbPages = 1;
        if(HAL_OK != HAL_FLASHEx_Erase(&eraser, &page_error)) {
            if (verbose) {printf("Flash erase failed!\r\n");}
            printf("Error 0x%X\r\n", (int) page_error);
            HAL_FLASH_Lock();
            return 0;
        }
        if (verbose) {printf("Erased page %d at address: 0x%X\r\n", (int) page, (int) Flash_addr);}
        Flash_addr += FLASH_PAGE_SIZE;
        page++;
    }

    if (verbose) {printf("Flash erase succesful!\r\n");}
    return 1;
}

/* Write Flash memory */
int write_flash(uint32_t Flash_addr, uint32_t* Flash_wdata, int32_t n_words, int verbose)
{
    /* Unlock Flash memory */
    HAL_FLASH_Unlock();

    /* Write Flash memory */
    for (int i=0; i<n_words; i++) {
        if (HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Flash_addr, Flash_wdata[i])) {
            if (verbose) {printf("Flash write failed!\r\n");}
            HAL_FLASH_Lock();
            return 0;
        }
        Flash_addr += 4;
    }
    if (verbose) {printf("Flash write successful!\r\n");}

    HAL_FLASH_Lock();

    return 1;
}

/* Read Flash memory */
void read_flash(uint32_t Flash_addr, uint32_t* Flash_rdata, uint32_t n_bytes)
{
    memcpy(Flash_rdata, (uint32_t*) Flash_addr, n_bytes);
}

/**
  * @brief  BSP Push Button callback
  * @param  Button Specifies the pin connected EXTI line
  * @retval None.
  */
void BSP_PB_Callback(Button_TypeDef Button)
{
  ButtonPressed = 1;
}

/**
  * @brief  Splits a float into two integer values.
  * @param  in the float value as input
  * @param  out_value the pointer to the output integer structure
  * @param  dec_prec the decimal precision to be used
  * @retval None
  */
static void floatToInt(float in, displayFloatToInt_t *out_value, int32_t dec_prec)
{
  if(in >= 0.0f)
  {
    out_value->sign = 0;
  }else
  {
    out_value->sign = 1;
    in = -in;
  }

  in = in + (0.5f / pow(10, dec_prec));
  out_value->out_int = (int32_t)in;
  in = in - (float)(out_value->out_int);
  out_value->out_dec = (int32_t)trunc(in * pow(10, dec_prec));
}

uint32_t FloatToUint(float n)
{
   return (uint32_t)(*(uint32_t*)&n);
}

float UintToFloat(uint32_t n)
{
   return (float)(*(float*)&n);
}

/* Pressure to altitude conversion */
float pressureToAltitude(float pressure)
{
    return 44330.77 * (1-pow(pressure/1013.26, 0.1902632));
}

/* Read function for scanf */
int _read(int file, char *result, size_t len)
{
  HAL_StatusTypeDef status;
  int retcode = 0;

  /* Read data from USART */
  if (len != 0) {
    status = HAL_UART_Receive_IT(&huart2, (uint8_t *) result, len);
    while(HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY){};
    if (status == HAL_OK) {
        retcode = len;
      } else {
        retcode = -1;
      }
  }

  /* Write back data to USART */
  HAL_UART_Transmit_IT(&huart2, (uint8_t *) result, len);
  while(HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY);

  return retcode;
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

  /* USER CODE END Error_Handler_Debug */
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
