/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

#include "Modbus.h"
#include "ModbusConfig.h"

#include "Linealizacion.c"
#include "Linealizacion.h"

#include "control.c"
#include "control.h"

#include "INA219.h"
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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* Definitions for Speed */
osThreadId_t SpeedHandle;
const osThreadAttr_t Speed_attributes = {
		.name = "Speed",
		.stack_size = 256 * 4,
		.priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Modbus */
osThreadId_t ModbusHandle;
const osThreadAttr_t Modbus_attributes = {
		.name = "Modbus",
		.stack_size = 256 * 4,
		.priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for CheckVelocidad */
osThreadId_t CheckVelocidadHandle;
const osThreadAttr_t CheckVelocidad_attributes = {
		.name = "CheckVelocidad",
		.stack_size = 256 * 4,
		.priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for TaskControl */
osThreadId_t TaskControlHandle;
const osThreadAttr_t TaskControl_attributes = {
		.name = "TaskControl",
		.stack_size = 256 * 4,
		.priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for SpeedSemaphore */
osSemaphoreId_t SpeedSemaphoreHandle;
const osSemaphoreAttr_t SpeedSemaphore_attributes = {
		.name = "SpeedSemaphore"
};
/* USER CODE BEGIN PV */

INA219_t ina219;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
void StartSpeed(void *argument);
void StartModbus(void *argument);
void StartCheckVelocidad(void *argument);
void StartTaskControl(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//---------------->  Modbus
modbusHandler_t ModbusH;
uint16_t ModbusDATA[32]={0,0,0,0,0,0,0,0,0,'\0'}; // Mapa modbus!
//---------------->

float velocidad = 0;
float velocidad_prima1,velocidad_prima2;

uint32_t ticksPrev = 0;
uint32_t ticksNow = 0;
uint32_t ticksAux = 0;
uint32_t deltaTicks = 0;


uint16_t overflow = 0; // Cantidad de desbordes del timer


void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin){
	if (GPIO_Pin == D02_Encoder_Pin){
		ticksAux = ticksPrev;
		ticksPrev = ticksNow;
		ticksNow = __HAL_TIM_GetCounter(&htim2);
		osSemaphoreRelease(SpeedSemaphoreHandle);
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
	MX_TIM2_Init();
	MX_TIM1_Init();
	MX_USART3_UART_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	Linealizacion_initialize();
	control_initialize();


	// Definiciones para la biblioteca de modbus
	ModbusH.uModbusType = MB_SLAVE;
	ModbusH.port =  &huart3;
	ModbusH.u8id = 1; //Modbus slave ID
	ModbusH.u16timeOut = 1000;
	ModbusH.EN_Port = NULL;
	ModbusH.u16regs = ModbusDATA;
	ModbusH.u16regsize= sizeof(ModbusDATA)/sizeof(ModbusDATA[0]);
	ModbusH.xTypeHW = USART_HW;

	//Initialize Modbus library
	ModbusInit(&ModbusH);
	//Start capturing traffic on serial Port
	ModbusStart(&ModbusH);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	//  	HAL_UART_Transmit(&huart3, (uint8_t*)"V\n", 3*sizeof(char), HAL_MAX_DELAY);
	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* creation of SpeedSemaphore */
	SpeedSemaphoreHandle = osSemaphoreNew(1, 1, &SpeedSemaphore_attributes);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of Speed */
	SpeedHandle = osThreadNew(StartSpeed, NULL, &Speed_attributes);

	/* creation of Modbus */
	ModbusHandle = osThreadNew(StartModbus, NULL, &Modbus_attributes);

	/* creation of CheckVelocidad */
	CheckVelocidadHandle = osThreadNew(StartCheckVelocidad, NULL, &CheckVelocidad_attributes);

	/* creation of TaskControl */
	TaskControlHandle = osThreadNew(StartTaskControl, NULL, &TaskControl_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
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
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 72-1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 10000-1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

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
	htim2.Init.Prescaler = 1440-1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 20000-1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

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
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, IN2_1_Pin|IN2_2_Pin|IN1_2_Pin|IN1_1_Pin
			|IN3_2_Pin|IN3_1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, IN4_1_Pin|IN4_2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : Led_Pin */
	GPIO_InitStruct.Pin = Led_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Led_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : D01_Encoder_Pin */
	GPIO_InitStruct.Pin = D01_Encoder_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(D01_Encoder_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : D02_Encoder_Pin D03_Encoder_Pin */
	GPIO_InitStruct.Pin = D02_Encoder_Pin|D03_Encoder_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : IN2_1_Pin IN2_2_Pin IN1_2_Pin IN1_1_Pin
                           IN3_2_Pin IN3_1_Pin */
	GPIO_InitStruct.Pin = IN2_1_Pin|IN2_2_Pin|IN1_2_Pin|IN1_1_Pin
			|IN3_2_Pin|IN3_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : IN4_1_Pin IN4_2_Pin */
	GPIO_InitStruct.Pin = IN4_1_Pin|IN4_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartSpeed */
/**
 * @brief  Function implementing the Speed thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSpeed */
void StartSpeed(void *argument)
{
	/* USER CODE BEGIN 5 */

	uint32_t ranuras = 50;
	uint32_t cantTicksTmr2 = 20000;
	uint64_t fsTmr2= 50000;
	uint64_t tickFilter = 600; // Parametro delicado, puede hacer cagadas en la medicion de la velocidad, OJO

	uint32_t ticksPrev_l = 0;
	uint32_t ticksNow_l = 0;
	uint32_t ticksAux_l = 0;
	uint32_t deltaTicks_l = 0;
	uint16_t overflow_l =0;
	float velocidad_prima2_l = 0;
	float velocidad_prima1_l = 0;
	float velocidad_l = 0;
	/* Infinite loop */
	for(;;)
	{
		osSemaphoreAcquire(SpeedSemaphoreHandle, osWaitForever);
		taskENTER_CRITICAL();

		ticksPrev_l = ticksPrev;
		ticksNow_l = ticksNow;
		ticksAux_l = ticksAux;
		overflow_l = overflow;
		velocidad_l = velocidad;
		velocidad_prima1_l = velocidad_prima1;
		velocidad_prima2_l = velocidad_prima2;

		taskEXIT_CRITICAL();

		//		HAL_NVIC_EnableIRQ(EXTI1_IRQn);

		if (overflow_l == 0){
			// Todo cool, calculo normal
			deltaTicks_l = ticksNow_l - ticksPrev_l;
			if (deltaTicks_l > tickFilter){
				velocidad_l = ((1/(float)ranuras)/((float)deltaTicks_l/(float)fsTmr2));
				//Filtro IIR
				velocidad_prima2_l = velocidad_prima1_l;
				velocidad_prima1_l = 0.7*velocidad_prima2_l + 0.3*velocidad_l;

				taskENTER_CRITICAL();
				deltaTicks = deltaTicks_l;
				velocidad = velocidad_l;
				velocidad_prima1 = velocidad_prima1_l;
				velocidad_prima2 = velocidad_prima2_l;
				taskEXIT_CRITICAL();
			}
			else{
				taskENTER_CRITICAL();
				ticksNow = ticksPrev_l;
				ticksPrev = ticksAux_l;
				taskEXIT_CRITICAL();
			}
		} else{
			// Tuve algun desborde y tengo que tenerlo en cuenta
			deltaTicks_l = (ticksNow_l + overflow_l * cantTicksTmr2)- ticksPrev_l;/////////////////////////////////////////
			if (deltaTicks_l > tickFilter){
				velocidad_l = ((1/(float)ranuras)/((float)deltaTicks_l/(float)fsTmr2));
				//Filtro IIR
				velocidad_prima2_l = velocidad_prima1_l;
				velocidad_prima1_l = 0.7*velocidad_prima2_l + 0.3*velocidad_l;

				taskENTER_CRITICAL();
				overflow = 0;
				deltaTicks = deltaTicks_l;
				velocidad = velocidad_l;
				velocidad_prima1 = velocidad_prima1_l;
				velocidad_prima2 = velocidad_prima2_l;
				taskEXIT_CRITICAL();
			}
			else{
				taskENTER_CRITICAL();
				ticksNow = ticksPrev_l;
				ticksPrev = ticksAux_l;
				taskEXIT_CRITICAL();
			}
		}
		osDelay(1);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartModbus */
/**
 * @brief Function implementing the Modbus thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartModbus */
void StartModbus(void *argument)
{
	/* USER CODE BEGIN StartModbus */
	uint16_t delta[2];// para mandar los deltaticks
	uint16_t delta1[2];
	uint16_t deltaticks[2];
	uint16_t delta2[2];
	uint16_t delta3[2];


	float velocidad_l = 0;
	float velocidad_prima1_l = 0;
	uint32_t deltaTicks_l = 0;

	uint16_t ModbusDATA_l[32] = {'\0'};
	/* Infinite loop */
	for(;;)
	{
		//HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
		taskENTER_CRITICAL();
		deltaTicks_l = deltaTicks;
		velocidad_l = velocidad;
		velocidad_prima1_l = velocidad_prima1;
		ModbusDATA_l[0] = ModbusDATA[0];
		taskEXIT_CRITICAL();

		HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_SET);


		memcpy(deltaticks, &deltaTicks_l, sizeof(deltaTicks_l));
		ModbusDATA_l[1]=deltaticks[0];
		ModbusDATA_l[2]=deltaticks[1];

		memcpy(delta, &velocidad_l, sizeof(velocidad_l));
		ModbusDATA_l[3]=delta[0];
		ModbusDATA_l[4]=delta[1];

		memcpy(delta1, &velocidad_prima1_l, sizeof(velocidad_prima1_l));
		ModbusDATA_l[5]=delta1[0];
		ModbusDATA_l[6]=delta1[1];

		memcpy(delta2, &rtSalida_Linealizacion, sizeof(rtSalida_Linealizacion));
		ModbusDATA_l[7]=delta2[0];
		ModbusDATA_l[8]=delta2[1];


		memcpy(delta3, &rtEntrada_Control, sizeof(rtEntrada_Control));
		ModbusDATA_l[9]=delta3[0];
		ModbusDATA_l[10]=delta3[1];


		taskENTER_CRITICAL();
		ModbusDATA[0] = ModbusDATA_l[0];
		ModbusDATA[1] = ModbusDATA_l[1];
		ModbusDATA[2] = ModbusDATA_l[2];
		ModbusDATA[3] = ModbusDATA_l[3];
		ModbusDATA[4] = ModbusDATA_l[4];
		ModbusDATA[5] = ModbusDATA_l[5];
		ModbusDATA[6] = ModbusDATA_l[6];
		ModbusDATA[7] = ModbusDATA_l[7];
		ModbusDATA[8] = ModbusDATA_l[8];
		ModbusDATA[9] = ModbusDATA_l[9];
		ModbusDATA[10] = ModbusDATA_l[10];


		taskEXIT_CRITICAL();

		osDelay(10);
	}
	/* USER CODE END StartModbus */
}

/* USER CODE BEGIN Header_StartCheckVelocidad */
/**
 * @brief Function implementing the CheckVelocidad thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCheckVelocidad */
void StartCheckVelocidad(void *argument)
{
	/* USER CODE BEGIN StartCheckVelocidad */

	uint16_t overflow_l =0;

	///////////////////////////

	uint16_t rawCurrent = 0;
	float current = 0;
	while(!INA219_Init(&ina219, &hi2c1, INA219_ADDRESS));
	uint16_t delta[2];

	////////////////////////////////////



	/* Infinite loop */
	for(;;)
	{
		taskENTER_CRITICAL();
		overflow_l = overflow;
		taskEXIT_CRITICAL();

		if(overflow_l >= 3){
			overflow_l = 0;
			taskENTER_CRITICAL();
			overflow = 0;
			velocidad_prima2 = 0;
			velocidad_prima1 = 0;
			velocidad = 0;
			taskEXIT_CRITICAL();

			/////////////////////////////////////////////
			rawCurrent = INA219_ReadCurrent_raw(&ina219); // mA?
			if(rawCurrent > 32767){
				//hago complemento a 2
				uint16_t complementCurrent = (0xFFFF - rawCurrent)+1;
				current = - complementCurrent / 10.0;

			} else {
				current = rawCurrent/10.0; // mA
			}

//			memcpy(delta, &current, sizeof(current));
//			ModbusDATA[13]=delta[0];
//			ModbusDATA[14]=delta[1];
			//////////////////////////////////////////////
		}
		osDelay(5);
	}
	/* USER CODE END StartCheckVelocidad */
}

/* USER CODE BEGIN Header_StartTaskControl */
/**
 * @brief Function implementing the TaskControl thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskControl */
void StartTaskControl(void *argument)
{
	/* USER CODE BEGIN StartTaskControl */
	float velocidad_l=0;
	float Setpoint = 0;
	uint16_t delta4[2];
	float error=0;

	/* Infinite loop */
	for(;;)
	{
		//		HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
		taskENTER_CRITICAL();
		velocidad_l = velocidad;
		Setpoint = ModbusDATA[0]/1000.0;

		memcpy(delta4, &Setpoint, sizeof(Setpoint));
		ModbusDATA[11]=delta4[0];
		ModbusDATA[12]=delta4[1];

		taskEXIT_CRITICAL();

		rtEntrada_Control = Setpoint - velocidad_l; //Error para PID
		control_step(); //Ejecutamos control

		rtEntrada_Linealizacion = rtSalida_Control;	//Salida PID asignada a entrada de planta linealizadora
		Linealizacion_step();	//Ejecutamos planta linealizadora

		htim1.Instance->CCR2 = (uint32_t)rtSalida_Linealizacion;	//Salida linealizada asignada a CCR



		//		Codigo para validar control
		//		taskENTER_CRITICAL();
		//		Setpoint = ModbusDATA[0]/1000.0;
		//		taskEXIT_CRITICAL();
		//
		//		rtEntrada_Control = Setpoint - velocidad_l; //Error para PID
		//		control_step(); //Ejecutamos control
		//
		//		htim1.Instance->CCR2 = rtSalida_Control;	//Salida PID asignada a entrada de planta linealizadora

		//		Codigo para validar linealización
		//		taskENTER_CRITICAL();
		//		Setpoint = ModbusDATA[0]/1000.0; // Setpoint de velocidad (vueltas/seg*1000)
		//		taskEXIT_CRITICAL();
		//
		//		rtEntrada_Linealizacion = Setpoint;
		//		Linealizacion_step();
		//		htim1.Instance->CCR2 = rtSalida_Linealizacion;

		//		taskENTER_CRITICAL();
		//		Setpoint = ModbusDATA[0]; // Setpoint de velocidad (vueltas/seg*1000)
		//		taskEXIT_CRITICAL();
		//
		//		htim1.Instance->CCR2 = Setpoint;

		osDelay(5);

	}

	/* USER CODE END StartTaskControl */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM4 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */
	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM4) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */
	if(htim->Instance == TIM2){
		overflow += 1;

	}
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

