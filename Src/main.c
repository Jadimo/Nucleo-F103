/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern uint8_t uSynchroMode;
extern uint8_t GPO_Low;
uint16_t counter;
uint16_t last_count;
uint16_t led_effect;
sURI_Info URI;
sAARInfo App;
sSMSInfo sms;
sGeoInfo geo;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void M24SR_I2CInit(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void M24SR_Program(uint16_t counter);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);

  while (TT4_Init() != SUCCESS);

  counter=0;
  last_count=-1;
  led_effect=0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(counter!=last_count){
		  HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
		  last_count=counter;
		  M24SR_Program(counter);
	  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR2;
  if (HAL_TIM_SlaveConfigSynchronization(&htim3, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|RF_DIS_Pin|LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin RF_DIS_Pin LED4_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|RF_DIS_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GPO_Pin */
  GPIO_InitStruct.Pin = GPO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void M24SR_I2CInit (){

	if( hi2c1.Instance == M24SR_I2C)
		  HAL_I2C_DeInit(&hi2c1);

	I2C1_FORCE_RESET();
	HAL_Delay(1000);
	I2C1_RELEASE_RESET();
	HAL_GPIO_DeInit(SCL_GPIO_Port, SCL_Pin);
	HAL_GPIO_DeInit(SDA_GPIO_Port, SDA_Pin);
	__HAL_AFIO_REMAP_I2C1_DISABLE();

	GPIO_InitTypeDef  GPIO_InitStruct;
	I2Cx_SCL_GPIO_CLK_ENABLE();
	I2Cx_SDA_GPIO_CLK_ENABLE();
	__HAL_RCC_AFIO_CLK_ENABLE();
	I2Cx_CLK_ENABLE();
	GPIO_InitStruct.Pin       = SCL_Pin;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull      = GPIO_PULLUP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(SCL_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin		  = SDA_Pin;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull      = GPIO_PULLUP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(SDA_GPIO_Port, &GPIO_InitStruct);
	__HAL_AFIO_REMAP_I2C1_ENABLE();

	hi2c1.Instance 	     = M24SR_I2C;
	hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
	#if defined (STM32F302x8)
	  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	  hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
	#elif defined (STM32F401xE)
	  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
	  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
	  hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLED;
	#endif
	  hi2c1.Init.OwnAddress1     = 0;
	  hi2c1.Init.OwnAddress2     = 0;
	#if (defined USE_STM32F4XX_NUCLEO) || (defined USE_STM32L1XX_NUCLEO) || \
		  (defined USE_STM32F1XX_NUCLEO)
	  hi2c1.Init.ClockSpeed      = M24SR_I2C_SPEED;
	  hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
	#elif (defined USE_STM32F0XX_NUCLEO) || (defined USE_STM32L0XX_NUCLEO) || \
	      (defined USE_STM32F3XX_NUCLEO) || (defined USE_STM32L4XX_NUCLEO)
	  hi2c1.Init.Timing          = M24SR_I2C_SPEED;
	#endif
	if(HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
	    /* Initialization Error */
	    Error_Handler();
	}
	HAL_Delay(100);

}
/**
  * @brief  This function retrieve current tick
  * @param	ptickstart: pointer on a variable to store current tick value
  */

void M24SR_WaitMs(uint32_t time_ms)
{
	wait_ms(time_ms);
}

void M24SR_GetTick( uint32_t *ptickstart )
{
	*ptickstart = HAL_GetTick();
}
/**
  * @brief  This function read the state of the M24SR GPO
	* @param	none
  * @retval GPIO_PinState : state of the M24SR GPO
  */
void M24SR_GPO_ReadPin( GPIO_PinState * pPinState)
{
	*pPinState = HAL_GPIO_ReadPin(M24SR_GPO_PIN_PORT,M24SR_GPO_PIN);
}

/**
  * @brief  This function set the state of the M24SR RF disable pin
	* @param	PinState: put RF disable pin of M24SR in PinState (1 or 0)
  */
void M24SR_RFDIS_WritePin( GPIO_PinState PinState)
{
	HAL_GPIO_WritePin(M24SR_RFDIS_PIN_PORT,M24SR_RFDIS_PIN,PinState);
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == M24SR_GPO_PIN)
	{
		if( uSynchroMode == M24SR_INTERRUPT_GPO)
			GPO_Low = 1;
	}
	if(GPIO_Pin == Button_Pin){
		counter++;
		if(counter >= 4){
			counter = 0;
		}
	}
}

void M24SR_Program(uint16_t counter){
	switch(counter){
	case 0:
		strcpy(URI.protocol,URI_ID_0x01_STRING);
		strcpy(URI.URI_Message,"pssite.com");
		strcpy(URI.Information,"\0");
		while (TT4_WriteURI(&URI) != SUCCESS);
		break;
	case 1:
		strcpy(App.PakageName,"com.sonyericsson.music");
		while (TT4_AddAAR(&App)!=SUCCESS);
		break;
	case 2:
		strcpy(geo.Latitude,"50.026596");
		strcpy(geo.Longitude,"21.984825");
		strcpy(geo.Information,"\0");
		while(TT4_WriteGeo(&geo) != SUCCESS);
		break;
	case 3:
		strcpy(sms.Message,"Hello World");
		strcpy(sms.PhoneNumber,"792 240 994");
		strcpy(sms.Information,"\0");
		while(TT4_WriteSMS(&sms) != SUCCESS);
		break;
	default:
		counter=0;
		break;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM3){
		switch(led_effect){
		case 0:
			HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
			HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
			led_effect++;
			break;
		case 1:
			HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
			HAL_GPIO_TogglePin(LED4_GPIO_Port,LED4_Pin);
			led_effect++;
			break;
		case 2:
			HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
			HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
			led_effect++;
			break;
		case 3:
			HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
			HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
			HAL_GPIO_TogglePin(LED4_GPIO_Port,LED4_Pin);
			led_effect++;
			break;
		case 4:
			HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
			HAL_GPIO_TogglePin(LED4_GPIO_Port,LED4_Pin);
			led_effect++;
			break;
		case 5:
			HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
			HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
			HAL_GPIO_TogglePin(LED4_GPIO_Port,LED4_Pin);
			led_effect=0;
			break;
		}
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
