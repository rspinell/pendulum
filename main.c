
#include "main.h"
#include "stm32l4xx_hal.h"
#include "Board_LED.h"
#include "stm32l4xx.h"
#include <stdio.h>
#include "stdlib.h"
#include <stdint.h>
#include <math.h>
#define periodValue (int)(45561 - 1)
	
//(PC0) = UserCtrl Input 	= &hadc1
//(PC1) = Pendulum Input 	= &hadc2
//(PA8) = PWM Output 		= &tim1

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_ChannelConfTypeDef sConfig;

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;
TIM_OC_InitTypeDef sConfigOC;

float duty;
int Vreference, Vsystem, err, y, *pDuty, pulseValue;

//PID Declaration & Control Parameters
float ts = 0.0001; 		//time step in seconds
float P = 800; 
float I = 1;
float D = 0.005;
int tol = 10;			//error tolerance in [mV]

float Integrate, Ppart, Ipart, Dpart;
int PIDpart = 0;
int errOld = 0;
int num = 0;

void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);		//possibly remove, we initialize below main
int main(void){

HAL_Init();
SystemClock_Config();				//Configure the system clock

//Initialize parameters 
pDuty = &pulseValue;			//pDuty points to PWM pulse value
duty = 0.75;				//Initialize duty 
pulseValue = periodValue*0.50;		//Initialize pulse value
	
Integrate = 0.0; 
Ppart = 0.0;
Ipart = 0.0;
Dpart = 0.0;

//Initialize all configured peripherals
MX_GPIO_Init();
MX_ADC1_Init();
MX_ADC2_Init();
MX_TIM1_Init();
	LED_Initialize();	

	//PWM Generation Error
	if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK){
		Error_Handler();							
	}
	
	LED_Off(num);
	
  while(1){
				
	//Configure and Start both ADC channels
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	HAL_ADC_Start(&hadc1);
		
	HAL_ADC_ConfigChannel(&hadc2, &sConfig);
	HAL_ADC_Start(&hadc2);
		
	//Read from both ADC channels
	if(HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK){		
		Vreference = HAL_ADC_GetValue(&hadc1);			//Voltage read from User Potentiometer [mV]
			
		if(HAL_ADC_PollForConversion(&hadc2, 100) == HAL_OK){	
			Vsystem = HAL_ADC_GetValue(&hadc2);			//Voltage read from Pendulum Potentiometer [mV]
				
			err = (int)(Vreference - Vsystem);			//Error defined as the difference between Voltage measurements of Potentiometers
				
			if(err < tol){ 
				LED_On(num);
			}else{ 
				LED_Off(num);
			     }	//Turn on LED if error is less than tolerance	
				
			//Nonlinear steady state mapping.
			duty = (float) (0.07*sin(0.0015*Vreference - 0.8936) + 0.37);
				
			y = (int)(periodValue*duty);							//Write steady-state response pulse value to PWM Register
	
			//PID Control Begin
			
			Ipart = Integrate*I;
			if (abs(err) < tol ){
				Integrate = Integrate;
			}		//Deals with Integral Windup
			else {
				Integrate = Integrate + (((err + errOld)*0.5)*ts);
			}
			Ppart = P*(sin(0.0015*err - 0.8936));			//Nonlinear proportional component, follows steady-state function
				
			Dpart = D*((err - errOld)/ts);
				
			PIDpart = (int)(Ipart + Ppart + Dpart);
				
			y = y + (PIDpart);												//Output includes Steady-State and PID components
				
			if(y > (0.64*(periodValue))){
				y = (int) (0.64*(periodValue));
			}	
			errOld = err;
			//PID END

			*pDuty = y;							
	
			//Update PWM
			sConfigOC.OCMode = TIM_OCMODE_PWM1;	
			sConfigOC.Pulse = pulseValue;
			HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			}
		}
	}	
}

//System Clock Configuration
void SystemClock_Config(void){

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  //Initializes the CPU, AHB and APB busses clocks 
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    Error_Handler();
  }

  //Initializes the CPU, AHB and APB busses clocks 
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK){
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK){
    Error_Handler();
  }

  //Configure the main internal regulator output voltage 
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK){
    Error_Handler();
  }

  //Configure the Systick interrupt time 
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  //Configure the Systick 
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  //SysTick_IRQn interrupt configuration
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

//ADC1 initialization function
static void MX_ADC1_Init(void){

  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

  //Common configuration 
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  
	if (HAL_ADC_Init(&hadc1) != HAL_OK){
    Error_Handler();
  }

  //Configure the ADC multi-mode 
  multimode.Mode = ADC_MODE_INDEPENDENT;
  
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK){
    Error_Handler();
  }

  //Configure Regular Channel 
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){
    Error_Handler();
  }
}

//ADC2 initialization function
static void MX_ADC2_Init(void){

  ADC_ChannelConfTypeDef sConfig;

  //Common config 
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.NbrOfDiscConversion = 1;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  
	if (HAL_ADC_Init(&hadc2) != HAL_OK){
    Error_Handler();
  }

  //Configure Regular Channel 
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK){
    Error_Handler();
  }
}

//TIM1 initialization function
static void MX_TIM1_Init(void){
	
	int prescalerValue = (SystemCoreClock / 16000000) - 1;		//Prescaler Value: Constant - set for TIM1 Counter clock = 16MHz
	
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = prescalerValue;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = periodValue;													//PWM Period: Constant - set for the external servo controller (motor controller)
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
	
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK){
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK){
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pulseValue;														//PWM Pulse: Variable - adjusted for PWM duty cycle
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK){
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK){
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim1);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA2   ------> USART2_TX
     PA3   ------> USART2_RX
*/
static void MX_GPIO_Init(void){

  GPIO_InitTypeDef GPIO_InitStruct;

  //GPIO Ports Clock Enable
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  //Configure GPIO pin Output Level
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  //Configure GPIO pin : B1_Pin
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  //Configure GPIO pins : USART_TX_Pin USART_RX_Pin
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  //Configure GPIO pin : LD2_Pin
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void){
  /* User can add his own implementation to report the HAL error return state */
  while(1){}
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line){
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 
