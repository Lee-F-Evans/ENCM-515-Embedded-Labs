/**
  ******************************************************************************
  * @file    BSP/Src/main.c
  * @author  MCD Application Team
  * @brief   This example code shows how to use the STM324xG BSP Drivers
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  ******************************************************************************
  ******************************************************************************
  * This code was modified for use in ENCM 515 in 2022
  * B. Tan
  * Note: DO NOT REGENERATE CODE/MODIFY THE IOC
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define ITM_Port32(n) (*((volatile unsigned long *)(0xE0000000+4*n)))

//Change this value to control frame/block process size
#define BLOCK_NUM 3

/*Uncomment these for testing or utilizing different filtering methods*/
#define PROCESS_SAMPLE 1
//#define PROCESS_BLOCK 1
//#define UNFOLDED_PROCESS_BLOCK 1
//#define ASSEMBLY_PROCESS_BLOCK 1

#define NUMBER_OF_TAPS	256
#define BUFFER_SIZE 32
#define FUNCTIONAL_TEST 1 	// uncomment this flag if we want to test the code without the interrupt
#define OUTPUT_SAMPLES 500 	// number of samples in the output buffer
#define INPUT_SAMPLES 44100

uint16_t blockIndex = 0; //for iterating through the required amount of samples. 3 or 16

#ifndef PROCESS_SAMPLE
int16_t inputBuffer[BLOCK_NUM];
int16_t outputBuffer[OUTPUT_SAMPLES];
static int outputSampleCounter = 0;
#endif

__IO uint8_t UserPressButton = 0;

/* Wave Player Pause/Resume Status. Defined as external in waveplayer.c file */
__IO uint32_t PauseResumeStatus = IDLE_STATUS;

/* Counter for User button presses */
__IO uint32_t PressCount = 0;

TIM_HandleTypeDef    TimHandle;
TIM_OC_InitTypeDef   sConfig;
uint32_t uwPrescalerValue = 0;
uint32_t uwCapturedValue = 0;

volatile int32_t *raw_audio = 0x802002C; // ignore first 44 bytes of header
int16_t history_l[NUMBER_OF_TAPS];
int16_t history_r[NUMBER_OF_TAPS];
volatile int overflow_count = 0;
volatile int underflow_count = 0;

/* 256 */
int16_t filter_coeffs[NUMBER_OF_TAPS] = {-3, -8, -8, -12, -13, -13, -12, -9, -4, 1, 6, 10, 11, 9, 5, 0, -6, -11, -13, -13, -9, -2, 6, 13, 17, 17, 13, 5, -5, -14, -21, -23, -19, -10, 2, 15, 25, 30, 27, 17, 2, -15, -29, -37, -36, -26, -8, 13, 33, 45, 47, 37, 17, -9, -35, -53, -59, -50, -28, 3, 36, 61, 73, 66, 43, 6, -34, -69, -88, -86, -61, -20, 30, 75, 104, 108, 85, 38, -22, -80, -122, -135, -114, -63, 9, 83, 142, 167, 152, 96, 11, -83, -163, -207, -200, -142, -41, 78, 187, 257, 266, 206, 87, -66, -217, -327, -362, -306, -163, 41, 259, 436, 522, 481, 303, 14, -331, -654, -866, -886, -661, -174, 543, 1416, 2336, 3176, 3818, 4164, 4164, 3818, 3176, 2336, 1416, 543, -174, -661, -886, -866, -654, -331, 14, 303, 481, 522, 436, 259, 41, -163, -306, -362, -327, -217, -66, 87, 206, 266, 257, 187, 78, -41, -142, -200, -207, -163, -83, 11, 96, 152, 167, 142, 83, 9, -63, -114, -135, -122, -80, -22, 38, 85, 108, 104, 75, 30, -20, -61, -86, -88, -69, -34, 6, 43, 66, 73, 61, 36, 3, -28, -50, -59, -53, -35, -9, 17, 37, 47, 45, 33, 13, -8, -26, -36, -37, -29, -15, 2, 17, 27, 30, 25, 15, 2, -10, -19, -23, -21, -14, -5, 5, 13, 17, 17, 13, 6, -2, -9, -13, -13, -11, -6, 0, 5, 9, 11, 10, 6, 1, -4, -9, -12, -13, -13, -12, -8, -8, -3};

volatile int new_sample_flag = 0;
static int32_t sample_count = 0;
int16_t newSampleL = 0;
int16_t newSampleR = 0;
int16_t filteredSampleL;
int16_t filteredSampleR;

static volatile int32_t filteredOutBufferA[BUFFER_SIZE];
//static volatile int32_t filteredOutBufferB[BUFFER_SIZE];
static volatile int bufchoice = 0;

extern I2S_HandleTypeDef       hAudioOutI2s;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void GPIOA_Init(void);
static int16_t ProcessSample(int16_t newsample, int16_t* history);

void ProcessBlock(int16_t* sampleBlock, int16_t* history);
void UnfoldedProcessBlock(int16_t* sampleBlock, int16_t* history);
void AssemblyProcessBlock(int16_t* sampleBlock, int16_t* history);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
 /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();

  /* Configure LED3, LED4, LED5 and LED6 */
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);
  BSP_LED_Init(LED5);
  BSP_LED_Init(LED6);

  /* Configure the system clock to 100 MHz */
  SystemClock_Config();

  /* Configure GPIO so that we can probe PB2 with an Oscilloscope */
  GPIOA_Init();

  /* Configure the User Button in GPIO Mode */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  /* Set TIMx instance */
  TimHandle.Instance = TIMx;

  /* Initialize TIM3 peripheral to toggle with a frequency of ~ 8 kHz
   * System clock is 100 MHz and TIM3 is counting at the rate of the system clock
   * so 100 M / 8 k is 12500
   */
  TimHandle.Init.Period = 12499;
  TimHandle.Init.Prescaler = 0;
  TimHandle.Init.ClockDivision = 0;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if(HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
	  /* Initialization Error */
	  Error_Handler();
  }

  ITM_Port32(30) = 0;
  if(HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
  {
	  /* Starting Error */
	  Error_Handler();
  }
  /******************************************************************************
   ******************************************************************************
   ******************************************************************************
   * Init Complete
   * BEGIN LAB 2 CODE HERE
   ******************************************************************************
   ******************************************************************************
   ******************************************************************************
   */

  static uint16_t i = 0;
  static uint16_t k = 0;
  static uint16_t start = 0;

  while (1) {

//#ifdef FUNCTIONAL_TEST
//		if (sample_count < 64000) {
//			  newSampleL = (int16_t)raw_audio[sample_count];
//			  newSampleR = (int16_t)(raw_audio[sample_count] >> 16);
//			  sample_count++;
//		  } else {
//			  sample_count = 0;
//		  }
//#endif

#ifndef FUNCTIONAL_TEST
	if (new_sample_flag == 1) {
#endif

	/* circular buffer FIR */
	#ifdef PROCESS_SAMPLE
		ITM_Port32(31) = 1;
		filteredSampleL = ProcessSample(newSampleL,history_l);
		ITM_Port32(31) = 2;

		new_sample_flag = 0;
		if (i < NUMBER_OF_TAPS-1) {
			filteredSampleL = 0;
			i++;
		} else {
			if (bufchoice == 0) {
				filteredOutBufferA[k] = ((int32_t)filteredSampleL << 16) + (int32_t)filteredSampleL; // copy the filtered output to both channels
			} else {
//				filteredOutBufferB[k] = ((int32_t)filteredSampleL << 16) + (int32_t)filteredSampleL;
			}

			k++;
		}
	#endif

	/* basic block process */
	#ifdef PROCESS_BLOCK
		//Gather the required samples for block processing. 3 or 16
		for(int j = 0; j<BLOCK_NUM; j++){
			inputBuffer[j] = (int16_t)raw_audio[sample_count]; // Type casting to int16 gets only left channel data
			if(sample_count<INPUT_SAMPLES){
				sample_count++;
			} else {
				sample_count = 0;
			}
		}

		ITM_Port32(31) = 1;
		ProcessBlock(inputBuffer,history_l);
		ITM_Port32(31) = 2;
	#endif

	/* block process with unfold */
	#ifdef UNFOLDED_PROCESS_BLOCK
		//Gather the required samples for block processing. 3 or 16
		for(int j = 0; j<BLOCK_NUM; j++){
			inputBuffer[j] = (int16_t)raw_audio[sample_count]; // Type casting to int16 gets only left channel data
			if(sample_count<INPUT_SAMPLES){
				sample_count++;
			} else {
				sample_count = 0;
			}
		}

		ITM_Port32(31) = 1;
		UnfoldedProcessBlock(inputBuffer,history_l);
		ITM_Port32(31) = 2;
		#endif

	/* block processing unfolded with special instructions */
	#ifdef ASSEMBLY_PROCESS_BLOCK
		//Gather the required samples for block processing. 3 or 16
		for(int j = 0; j<BLOCK_NUM; j++){
			inputBuffer[j] = (int16_t)raw_audio[sample_count]; // Type casting to int16 gets only left channel data
			if(sample_count<INPUT_SAMPLES){
				sample_count++;
			} else {
				sample_count = 0;
			}
		}

		ITM_Port32(31) = 1;
		AssemblyProcessBlock(inputBuffer,history_l);
		ITM_Port32(31) = 2;
	#endif

#ifndef FUNCTIONAL_TEST
	}
#endif

	// once a buffer is full, we can swap to fill up the other buffer
	// this is probably not going to be used in Lab2
	if (k == BUFFER_SIZE) {
		k = 0;
//		bufchoice = bufchoice == 0 ? 1 : 0;
	}

//    if(UserPressButton == 1) {
//    	AudioPlay_Test();
//    	UserPressButton = 0;
//    }
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 100000000
  *            HCLK(Hz)                       = 100000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_M                          = 16
  *            PLL_N                          = 400
  *            PLL_P                          = 4
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 3
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (KEY_BUTTON_PIN == GPIO_Pin)
  {
    while (BSP_PB_GetState(BUTTON_KEY) != RESET);
    UserPressButton = 1;
  }
}

/**
  * @brief  Toggle LEDs
  * @param  None
  * @retval None
  */
void Toggle_Leds(void)
{
  BSP_LED_Toggle(LED3);
  HAL_Delay(100);
//  BSP_LED_Toggle(LED4);
//  HAL_Delay(100);
  BSP_LED_Toggle(LED5);
  HAL_Delay(100);
  BSP_LED_Toggle(LED6);
  HAL_Delay(100);
}

// This timer callback should trigger every 1/8000 Hz, and it emulates
// the idea of receiving a new sample peridiocally
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

//  BSP_LED_Toggle(LED4);
//  HAL_GPIO_TogglePin(SCOPE_CHECK_GPIO_Port, SCOPE_CHECK_Pin);

	// If we "miss" processing a sample, the new_sample_flag will still be
	// high on the trigger of the interrupt
	if (new_sample_flag == 1) {
		ITM_Port32(30) = 10;
	}

	// Otherwise, go to the raw audio in memory and "retrieve" a new sample every timer period
	// set the new_sample_flag high
#ifndef FUNCTIONAL_TEST
	if (sample_count < 64000) {
		newSampleL = (int16_t)raw_audio[sample_count];
		newSampleR = (int16_t)(raw_audio[sample_count] >> 16);
		sample_count++;

		if (sample_count >= 64000) sample_count = 0;
		new_sample_flag = 1;
  }
#endif
}

int _write(int file, char* ptr, int len) {
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++) {
		ITM_SendChar(*ptr++);
	}
	return len;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* Turn LED5 on */
  BSP_LED_On(LED5);
  while(1)
  {
  }
}

static void GPIOA_Init(void){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOB_CLK_ENABLE();
	/*Configure GPIO pin : SCOPE_CHECK_Pin */
	  GPIO_InitStruct.Pin = SCOPE_CHECK_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(SCOPE_CHECK_GPIO_Port, &GPIO_InitStruct);

}

#ifdef PROCESS_SAMPLE
uint16_t processIndex = 0;
static int16_t ProcessSample(int16_t newsample, int16_t* history) {
	//create value to store filtered sample
	int32_t accumulator = 0;

	//store the new sample in the buffer overwriting the oldest sample
	history[processIndex] = newsample;

	//Perform FIR convolution
	for(int16_t tap = 0; tap<NUMBER_OF_TAPS; tap++){
		accumulator += history[(processIndex + tap) % BUFFER_SIZE] * filter_coeffs[tap];
	}

	//advance the circular buffer 1 position forward
	processIndex = (processIndex + 1) % BUFFER_SIZE;

	//Check and correct if we have overflow, value is for the max and min value of a 16bit signed int.
	if(accumulator > 0x3FFFFFFF){
		accumulator = 0x3FFFFFFF;
	} else if (accumulator < -0x40000000){
		accumulator = -0x40000000;
	}

	return (int16_t)(accumulator>>15);
}
#endif

#ifdef PROCESS_BLOCK
//processes data in blocks of 3
void ProcessBlock(int16_t* sampleBlock, int16_t* history){

	//itterate through new frame samples
	for(int blockNum = 0; blockNum < BLOCK_NUM; blockNum++){

		int32_t accumulator = 0;
		// replace oldest value in history with new frame value
		history[blockIndex] = sampleBlock[blockNum];

		//Perform FIR convolution
		for(int16_t tap = 0; tap<NUMBER_OF_TAPS; tap++){
			accumulator += (int32_t)history[(blockIndex + tap) % NUMBER_OF_TAPS] * (int32_t)filter_coeffs[tap];
		}

		//advance the circular buffer 1 position forward
		blockIndex = (blockIndex + 1) % NUMBER_OF_TAPS;

		//Check and correct if we have overflow, value is for the max and min value of a 16bit signed int.
		if(accumulator > 0x3FFFFFFF){
			accumulator = 0x3FFFFFFF;
		} else if (accumulator < -0x40000000){
			accumulator = -0x40000000;
		}

		//add the new sample to the output buffer and update sample counter if out buffer is full
		outputBuffer[outputSampleCounter] = (int16_t) (accumulator >> 15);
		outputSampleCounter++;
		if(outputSampleCounter == OUTPUT_SAMPLES){
			outputSampleCounter = 0;
		}
	}
}
#endif

#ifdef UNFOLDED_PROCESS_BLOCK
void UnfoldedProcessBlock(int16_t* sampleBlock, int16_t* history){

	//itterate through new frame samples
	for(int blockNum = 0; blockNum < BLOCK_NUM; blockNum++){

		int32_t accumulator = 0;
		// replace oldest value in history with new frame value
		history[blockIndex] = sampleBlock[blockNum];

		//Perform FIR convolution with 4 operations unfolded
		for(int16_t tap = 0; tap<NUMBER_OF_TAPS; tap +=2){
			accumulator += (int32_t)history[(blockIndex + tap) % NUMBER_OF_TAPS] * (int32_t)filter_coeffs[tap];
			accumulator += (int32_t)history[(blockIndex + tap + 1) % NUMBER_OF_TAPS] * (int32_t)filter_coeffs[tap + 1];
		}

		//advance the circular buffer 1 position forward
		blockIndex = (blockIndex + 1) % NUMBER_OF_TAPS;

		//Check and correct if we have overflow, value is for the max and min value of a 16bit signed int.
		if(accumulator > 0x3FFFFFFF){
			accumulator = 0x3FFFFFFF;
		} else if (accumulator < -0x40000000){
			accumulator = -0x40000000;
		}

		//add the new sample to the output buffer and update sample counter if out buffer is full
		outputBuffer[outputSampleCounter] = (int16_t) (accumulator >> 15);
		if(outputSampleCounter == (OUTPUT_SAMPLES - 1)){
			outputSampleCounter = 0;
		} else {
			outputSampleCounter++;
		}
	}
}
#endif

#ifdef ASSEMBLY_PROCESS_BLOCK
void AssemblyProcessBlock(int16_t* sampleBlock, int16_t* history){
	//itterate through new frame samples
	for(int blockNum = 0; blockNum < BLOCK_NUM; blockNum++){

		int32_t accumulator = 0;
		// replace oldest value in history with new frame value
		history[blockIndex] = sampleBlock[blockNum];

		//Perform FIR convolution with 4 operations unfolded
		for(int16_t tap = 0; tap<NUMBER_OF_TAPS; tap +=2){

			//Multiple and accumulate accelerator SMLABB instruction
			__asm volatile ("SMLABB %[result], %[op1], %[op2], %[acc]"
			: [result] "=r" (accumulator)
			: [op1] "r" (history[(blockIndex + tap) % NUMBER_OF_TAPS]), [op2] "r" (filter_coeffs[tap]), [acc] "r" (accumulator)
			);

			__asm volatile ("SMLABB %[result], %[op1], %[op2], %[acc]"
			: [result] "=r" (accumulator)
			: [op1] "r" (history[(blockIndex + tap + 1) % NUMBER_OF_TAPS]), [op2] "r" (filter_coeffs[tap + 1]), [acc] "r" (accumulator)
			);
		}

		//advance the circular buffer 1 position forward
		blockIndex = (blockIndex + 1) % NUMBER_OF_TAPS;

		//Check and correct if we have overflow, value is for the max and min value of a 16bit signed int.
		if(accumulator > 0x3FFFFFFF){
			accumulator = 0x3FFFFFFF;
		} else if (accumulator < -0x40000000){
			accumulator = -0x40000000;
		}

		//add the new sample to the output buffer and update sample counter if out buffer is full
		outputBuffer[outputSampleCounter] = (int16_t) (accumulator >> 15);
		if(outputSampleCounter == (OUTPUT_SAMPLES - 1)){
			outputSampleCounter = 0;
		} else {
			outputSampleCounter++;
		}
	}

}

#endif

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
