#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>

UART_HandleTypeDef huart2;
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

volatile int calculateDistance = 0;

void pin_config(){
  //enable GPIOB on bus
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

  //Trigger pi PB5, set the pin B5 to alternate function and map it to alternate function 2
  GPIOB->MODER |= (GPIOB->MODER & ~GPIO_MODER_MODER5) | (0b0010 << GPIO_MODER_MODER5_Pos);
  GPIOB->AFR[0] |= (GPIOB->AFR[0] & ~GPIO_AFRL_AFRL5) | (0b0010 << GPIO_AFRL_AFRL5_Pos);


  //Echo pin PB6, set up pin B6 to alternate function and map it to alternate function 2
  GPIOB->MODER |= (GPIOB->MODER & ~GPIO_MODER_MODER6) | (0b0010  << GPIO_MODER_MODER6_Pos);
  GPIOB->AFR[0] |= ( GPIOB->AFR[0] & ~GPIO_AFRL_AFRL6) | (0b0010 << GPIO_AFRL_AFRL6_Pos);

}


void tim_config(){
  //////////////////////////////// ~ TIMER 3 CONFIG  (TRIGG OUTPUT) ~ ////////////////////////////////
  //enable timer 3 on the bus
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

  //set up the up counting to max value
  TIM3->PSC = 16-1;	//slowing down clock to 1 microsecond cc
  TIM3->ARR = 0xFFFF;

  //set up duty cycle to 10 microseconds
  TIM3->CCR2 = 10;

  //set up output of timer to PWM mode 1
  TIM3->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
  TIM3->CCER |= TIM_CCER_CC2E; //enable channel 2

  //reset and enable counter
  TIM3->CNT = 0;
  TIM3->CR1 |= TIM_CR1_CEN;
  //////////////////////////////////////////////////////////////////////////////////////////////////


  //////////////////////////////// ~ TIMER 4 CONFIG  (ECHO INPUT) ~ ////////////////////////////////
  //enable timer 4 on the bus
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

  //set up the up counting to max value
  TIM4->PSC = 16-1; //slowing down clock to 1 microsecond
  TIM4->ARR = 0xFFFF;

  //set up the timer for input on IC1
  TIM4->CCMR1 |= 0b0001; //setting up the timer to be an input timer
  TIM4->CCER |= TIM_CCER_CC1NP | TIM_CCER_CC1P; // channel set as input on both edges

  TIM4->CCER |= TIM_CCER_CC1E;    // enable channel 1

  //Enable interrupt for tim4 and attach it to NVIC
  TIM4->DIER = TIM_DIER_CC1IE;
  NVIC_EnableIRQ(TIM4_IRQn);

  //clear interrupt flag for the timer
  TIM4->SR = ~TIM_SR_CC1IF;

  //reset and enable the counter
  TIM4->CNT = 0;
  TIM4->CR1 |= TIM_CR1_CEN;
  //////////////////////////////////////////////////////////////////////////////////////////////////
}

int main(void)
{
  /* Initialize all configured peripherals */
  SystemClock_Config();
  MX_USART2_UART_Init();
  pin_config();
  tim_config();

  HAL_Init();

  while (1)
  {

    //print the distance to user
    const int MSGBUFSIZE = 80;
    char msgBuf[MSGBUFSIZE];
    snprintf(msgBuf, MSGBUFSIZE, "Distance in cm: %d\n", calculateDistance);
    HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);

  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM17)
  {
    HAL_IncTick();
  }

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

#ifdef USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
