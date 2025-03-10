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
//#include "arm_math.h"
#include <stdio.h>
#include "stdio.h"
#include <string.h>

//#define NUM_TAPS 32       // Number of filter coefficients
#define BLOCK_SIZE 16     // Number of samples processed at once
#define SIGNAL_LENGTH 60  // Total number of input samples
#define ARM_MATH_CM4
#define MAX 120
#define MU 0.01
//#include "arm_math.h"


  char buf1[100];
  char buf2[100];

  // Input signal (Example: Simulated Sine Wave)
  float desired[SIGNAL_LENGTH] = {
       0.0,  0.31,  0.59,  0.81,  0.95,  1.00,  0.95,  0.81,
       0.59, 0.31,  0.0,  -0.31, -0.59, -0.81, -0.95, -1.00,
      -0.95, -0.81, -0.59, -0.31,  0.0,  0.31,  0.59,  0.81,
       0.95,  1.00,  0.95,  0.81,  0.59,  0.31,  0.0, -0.31,
      -0.59, -0.81, -0.95, -1.00, -0.95, -0.81, -0.59, -0.31,
       0.0,  0.31,  0.59,  0.81,  0.95,  1.00,  0.95,  0.81,
       0.59, 0.31,  0.0, -0.31, -0.59, -0.81, -0.95, -1.00,
      -0.95, -0.81, -0.59, -0.31
  };// Input signal (Example: Simulated Sine Wave)
  float reference[SIGNAL_LENGTH] = {
       0.0,  0.31,  0.59,  0.81,  0.95,  1.00,  0.95,  0.81,
       0.59, 0.31,  0.0,  -0.31, -0.59, -0.81, -0.95, -1.00,
      -0.95, -0.81, -0.59, -0.31,  0.0,  0.31,  0.59,  0.81,
       0.95,  1.00,  0.95,  0.81,  0.59,  0.31,  0.0, -0.31,
      -0.59, -0.81, -0.95, -1.00, -0.95, -0.81, -0.59, -0.31,
       0.0,  0.31,  0.59,  0.81,  0.95,  1.00,  0.95,  0.81,
       0.59, 0.31,  0.0, -0.31, -0.59, -0.81, -0.95, -1.00,
      -0.95, -0.81, -0.59, -0.31
  };
  float output[SIGNAL_LENGTH]={};
// FIR filter coefficients (Generated using MATLAB/Python)
//float32_t firCoeffsF32[NUM_TAPS] = {
//    -0.001822, -0.003007, -0.004453, -0.006050, -0.007660, -0.009119, -0.010246,
//    -0.010863, -0.010806, -0.009929, -0.008110, -0.005253, -0.001308, 0.003708,
//    0.009707, 0.016574, 0.024152, 0.032243, 0.040622, 0.049049, 0.057282, 0.065085,
//    0.072234, 0.078523, 0.083769, 0.087814, 0.090530, 0.091825, 0.091639, 0.089947,
//    0.086759, 0.082122
//};



// Output signal (filtered result)
float output[SIGNAL_LENGTH];
// Adaptive LMS Filter
void lms_filter(float* reference,float* desired,float* output, int numSamples) {
    float w = 0.0;  // Filter weight
    float error, y;

    for (int i = 0; i < numSamples; i++) {
        y = w * reference[i];   // Filtered output
        error = desired[i] - y; // Error signal
        w += MU * error * reference[i]; // Weight update
        output[i] = (float) error;
    }
}

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

int main(void)
{

  /* USER CODE BEGIN 1 */
#define FRAME_SIZE 1024
//#define MU 0.0001  // Learning rate

// WAV header structure
typedef struct {
    char chunkID[4];
    int chunkSize;
    char format[4];
    char subchunk1ID[4];
    int subchunk1Size;
    short audioFormat;
    short numChannels;
    int sampleRate;
    int byteRate;
    short blockAlign;
    short bitsPerSample;
    char subchunk2ID[4];
    int subchunk2Size;
} WAVHeader;


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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // Initialize FIR filter
  //FIR_Init();
  uint8_t buf[120];



  // Apply FIR filter to input signal
  //FIR_Process(inputSignal, outputSignal, SIGNAL_LENGTH);
  lms_filter(&desired, &reference, &output, SIGNAL_LENGTH);
  strcpy((char*)buf,"Hello world");
  HAL_Delay(400);
  // Print results (for debugging)
  for (int i = 0; i < SIGNAL_LENGTH; i++) {
      //printf("Input: %f -> Output: %f\n", inputSignal[i], outputSignal[i]);
	  gcvt(desired[i],6,buf1);
	  gcvt(output[i],6,buf2);
	  sprintf(buf,"Input: %s --> output: %s \n",buf1,buf2);
	  HAL_Delay(1000);
	  HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen(buf), HAL_MAX_DELAY);
	  HAL_Delay(1000);
  }

  while (1); // Infinite loop to prevent exit
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
