/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
CAN_TxHeaderTypeDef   TxHeader,TxHeader1;
uint8_t               TxData[8];
uint32_t              TxMailbox;
uint16_t OwnID=0x123;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
char BT_Buff[200];
float T_Vout,T_Vout1, cs_voltage;
float Bat_voltage;
float current;
float R_NTC,R_NTC1;
uint8_t w,x,y,z,n,b,c,d;
uint16_t R_10k = 9990;
uint16_t B_param = 3977;
float T0 = 298.15;
float Temp_K,Temp1_K;
float Temp_C,Temp1_C;
uint16_t thermistor_adc_value,thermistor_adc_value1;
float CS_adc_value;
float volt_adc_value;

TaskHandle_t T1_Handle;
TaskHandle_t T2_Handle;
TaskHandle_t T3_Handle;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */
#define DWT_CYCCNT ((volatile uint32_t *) 0xE0001000)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void CAN_TxHeaderInit(void);

void ADC_Select_CH0 (void)
{
	 /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	ADC_ChannelConfTypeDef sConfig = {0};

	  sConfig.Channel = ADC_CHANNEL_0;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_CH1 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	 sConfig.Channel = ADC_CHANNEL_1;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_CH2 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	 sConfig.Channel = ADC_CHANNEL_2;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_CH3 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	 sConfig.Channel = ADC_CHANNEL_3;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_CH4 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	 sConfig.Channel = ADC_CHANNEL_4;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void delay()
{
	uint16_t g,k;

	for(g=1000;g!=0;g--) {

		for(k=1000;k!=0;k--);
	}
}
void BMS_transmitter(void)
{

	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
			HAL_Delay(1000);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);
	    }
}

void Task1(void *a)


{

	for(;;) {
				ADC_Select_CH1();
			  	HAL_ADC_Start(&hadc1);
			  	HAL_ADC_PollForConversion(&hadc1, 1000);
			  	thermistor_adc_value = HAL_ADC_GetValue(&hadc1);
			    HAL_ADC_Stop(&hadc1);
			  	T_Vout = (thermistor_adc_value* 3.3/4095);//*(3.3 /3);// Calculate NTC thermistor resistance
			  	R_NTC = (T_Vout * R_10k) / (3.3 - T_Vout);
			  	Temp_K = (T0 * B_param) / (T0 * log(R_NTC / R_10k) + B_param);   // Calculate temperature in Kelvin
			  	Temp_C = Temp_K - 273.15;    // Convert temperature to Celsius
			  //	TxData[0] = Temp_C;
			  	 delay();


			  	ADC_Select_CH2();
			  	HAL_ADC_Start(&hadc1);
			  	HAL_ADC_PollForConversion(&hadc1, 1000);
			  	thermistor_adc_value1 = HAL_ADC_GetValue(&hadc1);
			  	HAL_ADC_Stop(&hadc1);
			  	T_Vout1 = (thermistor_adc_value1* 3.3/4095);//*(3.3 /3);// Calculate NTC thermistor resistance
			  	R_NTC1 = (T_Vout1 * R_10k) / (3.3 - T_Vout1);
			  	Temp1_K = (T0 * B_param) / (T0 * log(R_NTC1 / R_10k) + B_param);   // Calculate temperature in Kelvin
			  	Temp1_C = Temp1_K - 273.15;    // Convert temperature to Celsius
			  	//	TxData[0] = Temp_C;
			  	 delay();

			  	 ADC_Select_CH3();
			  	 HAL_ADC_Start(&hadc1);
			  	 HAL_ADC_PollForConversion(&hadc1, 1000);
			  	 CS_adc_value = HAL_ADC_GetValue(&hadc1);
			  	 HAL_ADC_Stop(&hadc1);
			  	 cs_voltage= ((CS_adc_value*3.3*2)/4095)*0.92;  //0.96 perfect
			  	 current=(cs_voltage-2.5)/0.066;							 //30A 0.066
			  	 w=cs_voltage;
			  	 x=(cs_voltage-w)*100;
			  	 delay();

			  	 ADC_Select_CH4();
			  	 HAL_ADC_Start(&hadc1);
			  	 HAL_ADC_PollForConversion(&hadc1, 100);
			  	 volt_adc_value = HAL_ADC_GetValue(&hadc1);
			  	HAL_ADC_Stop(&hadc1);
			  	Bat_voltage = (volt_adc_value/4095)*12;   //13.3
			  	y=Bat_voltage;
			  	z=(Bat_voltage-y)*100;
			  	 delay();

			           if(Temp_C > 34 || Temp1_C > 34 || Bat_voltage < 9 || current > 2) {
			           			/* when the Batter  Bank Exceeds the Temp > 37 c and Bat_voltasge is below 9V & Bat_current > 2A */
			           			/*Relay Switch off tghe Battery supply */
			           				//   delay();
			           				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, RESET);
			           			   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, SET);
			           			}
			           			else if(Temp_C < 32 && Temp1_C < 34 && Bat_voltage > 9 && current < 2)
			           			{
			           						/* when the Batter  Bank Exceeds the Temp > 37 c and Bat_voltasge is below 9V & Bat_current > 2A */
			           						/*Relay Switch off tghe Battery supply */
			           			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, SET);
			           			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, RESET);
			          			}
			           vTaskSuspend(T1_Handle);

			 	}
}

void Task2 (void *a) {


	for(;;) {

						  // TxData[0] = (uint8_t)Temp_C;

						   //TxData[1] =(uint8_t) Temp1_C;
	 	           TxData[0] =n;
	 	           TxData[1] =b;
	 	           TxData[2] =c;
	 	           TxData[3] =d;
						   TxData[4] =w;
						   TxData[5] =x;
						   TxData[6] =y;
						   TxData[7] =z;
				       BMS_transmitter();
		           vTaskResume(T3_Handle);
		           vTaskResume(T1_Handle);
		           vTaskSuspend(T2_Handle);
	}
}

void Task3 (void *a) {

	for(;;) {
		sprintf(BT_Buff,"Temperature_T1=%.2f C\nTemperature_T2=%.2f C\nBattery_Current=%.2f A\nBattery_Voltage=%.2f V\n\n",Temp_C,Temp1_C,current,Bat_voltage);
		HAL_UART_Transmit(&huart5, (void *)BT_Buff, strlen(BT_Buff), 100);
					vTaskResume(T1_Handle);
					vTaskResume(T2_Handle);
					vTaskSuspend(T3_Handle);
	}

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
  MX_CAN1_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */

  HAL_CAN_Start(&hcan1);
  CAN_TxHeaderInit();
  	  	*DWT_CYCCNT = *DWT_CYCCNT | (1 << 0);
        SEGGER_SYSVIEW_Conf();
       // SEGGER_SYSVIEW_Start();

       xTaskCreate(Task1, "Battery_Sensosr_Parameters", 200,NULL, 2,&T1_Handle);
       xTaskCreate(Task2, "Data Transmit",200,NULL, 1,&T2_Handle);
       xTaskCreate(Task3, "BLE TX ",200,NULL,1,&T3_Handle);
       vTaskStartScheduler();


  /* USER CODE END 2 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

 // ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Channel = ADC_CHANNEL_0;
//  sConfig.Rank = 1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Channel = ADC_CHANNEL_1;
//  sConfig.Rank = 2;
//  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Channel = ADC_CHANNEL_2;
//  sConfig.Rank = 3;
//  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Channel = ADC_CHANNEL_3;
//  sConfig.Rank = 4;
//  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Channel = ADC_CHANNEL_4;
//  sConfig.Rank = 5;
//  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void CAN_TxHeaderInit(void)
{

	  TxHeader.StdId =OwnID;
	  TxHeader.IDE = CAN_ID_STD;
	  TxHeader.RTR = CAN_RTR_DATA;
	  TxHeader.DLC = 8;

}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
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
