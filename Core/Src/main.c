/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "lsm303agr.h"
#include <math.h>
#include <stdio.h>
extern I2C_HandleTypeDef hi2c1; // Use your I2C handle

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NORTH_HEADING_TOLERANCE    20.0f

#define TILT_THRESHOLD_DEG         10.0f

#define MAG_X_OFFSET               324
#define MAG_Y_OFFSET               -35

#define LCD_ADDR (0x27 << 1) // Shifted for HAL
#define LCD_BACKLIGHT 0x08
#define LCD_ENABLE    0x04
#define LCD_COMMAND   0
#define LCD_DATA      1
// Task handles
osThreadId_t sensorTaskHandle;    // Task that reads sensor data
osThreadId_t ledTaskHandle;       // Task that updates LEDs

// Mutex, Semaphore, and Queue handles
osMutexId_t       i2cMutexHandle;
osSemaphoreId_t   dataReadySemHandle;   // Signaled when new heading is in queue
osMessageQueueId_t headingQueueHandle;  // Queue to pass heading (float) from sensorTask to ledTask
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

I2S_HandleTypeDef hi2s3;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
// Task handles
osThreadId_t sensorTaskHandle;    // Task that reads sensor data
osThreadId_t ledTaskHandle;       // Task that updates LEDs

// Mutex, Semaphore, and Queue handles
osMutexId_t       i2cMutexHandle;
osSemaphoreId_t   dataReadySemHandle;   // Signaled when new heading is in queue
osMessageQueueId_t headingQueueHandle;  // Queue to pass heading (float) from sensorTask to ledTask

// Task attributes
const osThreadAttr_t sensorTask_attributes = {
  .name = "sensorTask",
  .priority = (osPriority_t)osPriorityNormal,
  .stack_size = 128 * 4
};

const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .priority = (osPriority_t)osPriorityNormal,
  .stack_size = 128 * 4
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2S3_Init(void);

/* USER CODE BEGIN PFP */
void StartSensorTask(void* argument);
void StartLedTask(void* argument);

// Accelerometer initialization routine
static void LSM303AGR_Accelerometer_Startup(void);
static void LSM303AGR_Magnetometer_Startup(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief Initializes the LSM303AGR magnetometer.
  */
static void LSM303AGR_Magnetometer_Startup(void)
{
  LSM303AGRMag_InitTypeDef magInitStruct;

  magInitStruct.Temperature_Sensor = LSM303AGR_TEMPSENSOR_DISABLE;
  magInitStruct.MagOutput_DataRate = LSM303AGR_ODR_15_HZ;
  magInitStruct.MagFull_Scale      = LSM303AGR_FS_4_0_GA;
  magInitStruct.Working_Mode       = LSM303AGR_CONTINUOS_CONVERSION;

  COMPASSACCELERO_IO_Init();
  LSM303AGR_MagInit(&magInitStruct);
}

/**
  * @brief Initializes the LSM303AGR accelerometer.
  */
static void LSM303AGR_Accelerometer_Startup(void)
{
  LSM303AGRAcc_InitTypeDef accInitStruct;

  accInitStruct.Power_Mode_Output_DataRate =
        (uint8_t)LSM303AGR_NORMAL_MODE | (uint8_t)LSM303AGR_ODR_50_HZ;
  accInitStruct.Axes_Enable       = LSM303AGR_AXES_ENABLE;
  accInitStruct.Decimation        = LSM303AGR_DEC_NO;
  accInitStruct.Full_Scale        = LSM303AGR_FULLSCALE_2G;
  accInitStruct.High_Resolution   = LSM303AGR_HR_DISABLE;
  accInitStruct.Fifo_Mode         = LSM303AGR_FIFO_DISABLE;
  accInitStruct.Fifo_Threshold    = 0;

  COMPASSACCELERO_IO_Init();
  LSM303AGR_AccInitStruct(&accInitStruct);

  LSM303AGR_AccITConfigTypeDef accITConfig;
  accITConfig.Dataready_Interrupt = LSM303AGR_DATA_READY_INTERRUPT_DISABLE;
  accITConfig.InterruptSignal     = LSM303AGR_INTERRUPT_SIGNAL_LOW;
  accITConfig.InterruptType       = LSM303AGR_INTERRUPT_REQUEST_PULSED;
  LSM303AGR_AccITConfig(&accITConfig);
}
void LCD_Init(void);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_SendString(char *str);
void LCD_SendInternal(uint8_t data, uint8_t flags) {
    uint8_t up = data & 0xF0;
    uint8_t lo = (data << 4) & 0xF0;
    uint8_t data_arr[4];
    data_arr[0] = up | flags | LCD_BACKLIGHT | LCD_ENABLE;
    data_arr[1] = up | flags | LCD_BACKLIGHT;
    data_arr[2] = lo | flags | LCD_BACKLIGHT | LCD_ENABLE;
    data_arr[3] = lo | flags | LCD_BACKLIGHT;
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, data_arr, 4, HAL_MAX_DELAY);
    HAL_Delay(1);
}

void LCD_SendCommand(uint8_t cmd) {
    LCD_SendInternal(cmd, LCD_COMMAND);
}

void LCD_SendData(uint8_t data) {
    LCD_SendInternal(data, LCD_DATA);
}

void LCD_Init(void) {
    HAL_Delay(50);
    LCD_SendCommand(0x30); HAL_Delay(5);
    LCD_SendCommand(0x30); HAL_Delay(1);
    LCD_SendCommand(0x30); HAL_Delay(10);
    LCD_SendCommand(0x20); HAL_Delay(10);
    LCD_SendCommand(0x28); // 4-bit, 2 line, 5x8 dots
    LCD_SendCommand(0x08); // Display off
    LCD_SendCommand(0x01); // Clear display
    HAL_Delay(2);
    LCD_SendCommand(0x06); // Entry mode
    LCD_SendCommand(0x0C); // Display on, cursor off
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    LCD_SendCommand(0x80 | (col + row_offsets[row]));
}

void LCD_SendString(char *str) {
    while(*str) LCD_SendData((uint8_t)(*str++));
}

void LCD_Clear(void) {
   LCD_SendCommand(0x01); // 0x01 is the clear display command for most HD44780-compatible LCDs
   HAL_Delay(2);          // Wait for the command to complete (typically 1.53ms minimum)
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_RTC_Init();
  MX_I2C2_Init();
  MX_I2S3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  // Create mutex

  i2cMutexHandle = osMutexNew(NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  dataReadySemHandle = osSemaphoreNew(1, 0, NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */

  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  headingQueueHandle = osMessageQueueNew(5, sizeof(float), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */

  /* USER CODE BEGIN RTOS_THREADS */
  sensorTaskHandle = osThreadNew(StartSensorTask, NULL, &sensorTask_attributes);
  ledTaskHandle = osThreadNew(StartLedTask, NULL, &ledTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */

  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_8K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD10_Pin|LD11_Pin|LD12_Pin
                          |LD13_Pin|LD14_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD7_Pin|LD8_Pin|LD9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DATA_Ready_Pin */
  GPIO_InitStruct.Pin = DATA_Ready_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DATA_Ready_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD10_Pin LD11_Pin LD12_Pin
                           LD13_Pin LD14_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD10_Pin|LD11_Pin|LD12_Pin
                          |LD13_Pin|LD14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_Pin INT2_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = INT1_Pin|INT2_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD7_Pin LD8_Pin LD9_Pin */
  GPIO_InitStruct.Pin = LD7_Pin|LD8_Pin|LD9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin       = GPIO_PIN_10 | GPIO_PIN_11;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void StartSensorTask(void* argument)
{
    // Initialize the magnetometer and accelerometer
    osMutexAcquire(i2cMutexHandle, osWaitForever);
    LSM303AGR_Magnetometer_Startup();
    LSM303AGR_Accelerometer_Startup();
    LCD_Init();
    LCD_Clear();
    osMutexRelease(i2cMutexHandle);

    int16_t magData[3], accData[3];
    float headingDeg = -1.0f, xi = 0, yi = 0, hdg = 0;
    char buf[32];

    for (;;)
    {

        // Acquire I2C bus before reading
        osMutexAcquire(i2cMutexHandle, osWaitForever);

        // Read the raw accelerometer data
        LSM303AGR_AccReadXYZ(accData);
        float Ax = (float)accData[0];
        float Ay = (float)accData[1];
        float Az = (float)accData[2];

        // Compute pitch and roll
        float pitchDeg = (180.0f / M_PI) * atanf(-Ax / sqrtf(Ay*Ay + Az*Az));
        float rollDeg  = (180.0f / M_PI) * atanf( Ay / Az);

        // Default: if not flat, set heading to negative sentinel
        headingDeg = -1.0f;

        // Check if board is flat
        if ((fabsf(pitchDeg) < TILT_THRESHOLD_DEG) && (fabsf(rollDeg) < TILT_THRESHOLD_DEG))
        {
            // Read magnetometer
            LSM303AGR_MagReadXYZ(magData);

            float x = (float)magData[0] - (float)MAG_X_OFFSET;
            float y = (float)magData[1] - (float)MAG_Y_OFFSET;

            headingDeg = atan2f(y, x) * (180.0f / M_PI);
            if (headingDeg < 0.0f)
            {
                headingDeg += 360.0f;
            }
            if (headingDeg >= 360.0f)
            {
                headingDeg = 0.0f;
            }
            if ((headingDeg > 45.0f) && (headingDeg < 245.0f))
            {
                headingDeg -= 90.0f;
            }
            if (((headingDeg > 0.0f) || (headingDeg < 0.0f)) && ((headingDeg < 5.0f) || (headingDeg < 15.0f)))
            {
                headingDeg = 0;
                headingDeg += 330.0f;
            }
            if ((headingDeg > 100.0f) && (headingDeg < 225.0f))
            {
                headingDeg += 45.0f;
            }

            xi = x;                     /* signed integers fit on line      */
            yi = y;
            hdg = headingDeg;
        }

        LCD_Clear();
        LCD_SetCursor(0, 0);
        LCD_SendString("Current Location");
        LCD_SetCursor(1, 0);
        sprintf(buf, "X:%.4f", xi);
        LCD_SendString(buf);
        LCD_SetCursor(2, 0);
        sprintf(buf, "Y:%.4f", yi);
        LCD_SendString(buf);
        LCD_SetCursor(3, 0);
        sprintf(buf, "Dir: %.2f", hdg);
        LCD_SendString(buf);

        // Release the mutex after sensor read
        osMutexRelease(i2cMutexHandle);

        // Send heading to the queue
        osMessageQueuePut(headingQueueHandle, &headingDeg, 0, 0);

        // Signal that new data is ready
        osSemaphoreRelease(dataReadySemHandle);

        osDelay(200);
    }
}

/**
  * @brief Task to read heading from queue and update LEDs.
  */
void StartLedTask(void* argument)
{
    float headingDeg;

    for (;;)
    {
        // Wait until the sensor task signals new data
        osSemaphoreAcquire(dataReadySemHandle, osWaitForever);

        // Receive heading from the queue
        if (osMessageQueueGet(headingQueueHandle, &headingDeg, NULL, 0) == osOK)
        {
            // Turn off all LEDs first
            HAL_GPIO_WritePin(GPIOB, LD7_Pin | LD8_Pin | LD9_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOE, LD10_Pin | LD11_Pin | LD12_Pin | LD13_Pin | LD14_Pin, GPIO_PIN_RESET);

            if (headingDeg < 0.0f)
            {
                // Not flat => keep all LEDs off
                continue;
            }

            /*
             * For 8 LEDs, we’ll break the circle into 8 octants of 45°:
             *   0   to  45 -> PB0  (LD7)
             *  45   to  90 -> PB1  (LD8)
             *  90   to 135 -> PB2  (LD9)
             * 135   to 180 -> PE7  (LD10)
             * 180   to 225 -> PE8  (LD11)
             * 225   to 270 -> PE9  (LD12)
             * 270   to 315 -> PE10 (LD13)
             * 315   to 360 -> PE11 (LD14)
             */

            if (headingDeg < 45.0f)
            {
                // 0° - 44.999... -> LD7
                HAL_GPIO_WritePin(GPIOB, LD7_Pin, GPIO_PIN_SET);
            }
            else if (headingDeg < 90.0f)
            {
                // 45° - 89.999... -> LD8
                HAL_GPIO_WritePin(GPIOB, LD8_Pin, GPIO_PIN_SET);
            }
            else if (headingDeg < 135.0f)
            {
                // 90° - 134.999... -> LD9
                HAL_GPIO_WritePin(GPIOB, LD9_Pin, GPIO_PIN_SET);
            }
            else if (headingDeg < 180.0f)
            {
                // 135° - 179.999... -> LD10
                HAL_GPIO_WritePin(GPIOE, LD10_Pin, GPIO_PIN_SET);
            }
            else if (headingDeg < 225.0f)
            {
                // 180° - 224.999... -> LD11
                HAL_GPIO_WritePin(GPIOE, LD11_Pin, GPIO_PIN_SET);
            }
            else if (headingDeg < 270.0f)
            {
                // 225° - 269.999... -> LD12
                HAL_GPIO_WritePin(GPIOE, LD12_Pin, GPIO_PIN_SET);
            }
            else if (headingDeg < 315.0f)
            {
                // 270° - 314.999... -> LD13
                HAL_GPIO_WritePin(GPIOE, LD13_Pin, GPIO_PIN_SET);
            }
            else
            {
                // 315° - 359.999... -> LD14
                HAL_GPIO_WritePin(GPIOE, LD14_Pin, GPIO_PIN_SET);
            }
        }
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
//void StartDefaultTask(void *argument)
//{
//  /* init code for USB_HOST */
//  MX_USB_HOST_Init();
//  /* USER CODE BEGIN 5 */
//  /* Infinite loop */
//  for(;;)
//  {
//    osDelay(1);
//  }
//  /* USER CODE END 5 */
//}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
