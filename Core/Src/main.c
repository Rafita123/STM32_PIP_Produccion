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

//#include "INA219.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define INA_219_ADDR_M1 (0x41)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* Definitions for Speed1 */
osThreadId_t Speed1Handle;
const osThreadAttr_t Speed1_attributes = {
  .name = "Speed1",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Modbus */
osThreadId_t ModbusHandle;
const osThreadAttr_t Modbus_attributes = {
  .name = "Modbus",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for CheckVelocidad */
osThreadId_t CheckVelocidadHandle;
const osThreadAttr_t CheckVelocidad_attributes = {
  .name = "CheckVelocidad",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal1,
};
/* Definitions for TaskControl */
osThreadId_t TaskControlHandle;
const osThreadAttr_t TaskControl_attributes = {
  .name = "TaskControl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Corriente */
osThreadId_t CorrienteHandle;
const osThreadAttr_t Corriente_attributes = {
  .name = "Corriente",
  .stack_size = 200 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Semaforo1 */
osSemaphoreId_t Semaforo1Handle;
const osSemaphoreAttr_t Semaforo1_attributes = {
  .name = "Semaforo1"
};
/* USER CODE BEGIN PV */

//INA219_t ina219;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
void StartSpeed1(void *argument);
void StartModbus(void *argument);
void StartCheckVelocidad(void *argument);
void StartTaskControl(void *argument);
void StartCorriente(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
HAL_StatusTypeDef ret; // Con ret veo los estados retornados por HAL
//---------------->  Modbus
modbusHandler_t ModbusH;
uint16_t ModbusDATA[24]={'\0'}; // Mapa modbus!
//---------------->
uint32_t ranuras = 50;
uint32_t cantTicksTmr2 = 20000;
uint64_t fsTmr2= 50000;
uint64_t tickFilter = 600; // Parametro delicado, puede hacer cagadas en la medicion de la velocidad, OJO

float velocidad[4];
float velocidad_prima1[4];
float velocidad_prima2[4];

uint8_t flags_motores[4];

uint32_t ticksPrev[4];
uint32_t ticksNow[4];
uint32_t ticksAux[4];
uint32_t deltaTicks[4];

uint16_t overflow[4] = {'\0'}; // Cantidad de desbordes del timer


float current = 0;


void Sentido(uint8_t valor,uint8_t motor){
	//Motor gira en un sentido
	if(motor == 1){
		if(valor == 0){
			HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, SET);
			HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, RESET);
		}
		//Motor gira en otro sentido
		else if(valor == 1){
			HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, RESET);
			HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, SET);

		}
		else{ // Break
			HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, RESET);
			HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, RESET);
		}
	}
	if(motor == 2){
		if(valor == 0){
			HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, SET);
			HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, RESET);
		}
		//Motor gira en otro sentido
		else if(valor == 1){
			HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, RESET);
			HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, SET);

		}
		else{ // Break
			HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, RESET);
			HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, RESET);
		}

	}
	if(motor == 3){
		if(valor == 0){
			HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, SET);
			HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, RESET);
		}
		//Motor gira en otro sentido
		else if(valor == 1){
			HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, RESET);
			HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, SET);

		}
		else{ // Break
			HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, RESET);
			HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, RESET);
		}
	}
	if(motor == 4){
		if(valor == 0){
			HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, SET);
			HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, RESET);
		}
		//Motor gira en otro sentido
		else if(valor == 1){
			HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, RESET);
			HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN4_2_Pin, SET);

		}
		else{ // Break
			HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, RESET);
			HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, RESET);
		}
	}

}


void configIna219(uint8_t address,uint16_t to){
	//	setCalibration_32V_1A();
	uint32_t ina219_calValue = 10240;
	HAL_I2C_DeInit(&hi2c1);
	HAL_I2C_Init(&hi2c1);
	// Set multipliers to convert raw current/power values
	//		uint32_t ina219_currentDivider_mA = 25;      // Current LSB = 40uA per bit (1000/40 = 25)
	//		uint32_t ina219_powerMultiplier_mW = 1;         // Power LSB = 800mW per bit

	// Set Calibration register to 'Cal' calculated above
	//	   wireWriteRegister(INA219_REG_CALIBRATION, ina219_calValue); // reg value
	uint8_t reg = 0x05;
	uint16_t value = ina219_calValue;
	uint8_t i2c_temp[2];
	i2c_temp[0] = value>>8;
	i2c_temp[1] = value;
	ret = HAL_I2C_Mem_Write(&hi2c1, address<<1, (uint16_t)reg, 1, i2c_temp, 2, to);
	//I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout
	osDelay(1);
		if(ret != HAL_OK){
//			current = 12345;
	//		sprintf((char*)buf,"Se rompio en config\r\n");
	//		HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), 1000);
		}

	// Set Config register to take into account the settings above
	uint16_t config = 8192 | 6144 | 384 | 120 | 7;
	//			  INA219_CONFIG_BVOLTAGERANGE_32V |
	//	                    INA219_CONFIG_GAIN_8_320MV |
	//	                    INA219_CONFIG_BADCRES_12BIT |
	//	                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
	//	                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

	//	wireWriteRegister(INA219_REG_CONFIG, config);// reg value
	reg = 0x00;
	value = config;
	i2c_temp[0] = value>>8;
	i2c_temp[1] = value;
	ret = HAL_I2C_Mem_Write(&hi2c1, address<<1, (uint16_t)reg, 1, i2c_temp, 2, to);
	//I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout
	osDelay(1);
		if(ret != HAL_OK){
	//			sprintf((char*)buf,"Se rompio en config\r\n");
	//			HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), 1000);
//			current = 23456;
			}
}

float getCurrent(uint8_t address, uint16_t to){
	//		current = getCurrent_mA();
	//		current = getCurrent_raw();
	float current;

	uint32_t ina219_calValue = 10240;
	// Set multipliers to convert raw current/power values
	uint32_t ina219_currentDivider_mA = 25;      // Current LSB = 40uA per bit (1000/40 = 25)
	//	uint32_t ina219_powerMultiplier_mW = 1;         // Power LSB = 800mW per bit

	//		wireWriteRegister(INA219_REG_CALIBRATION, ina219_calValue); // reg value
	uint8_t reg = 0x05;
	uint16_t value = ina219_calValue;
	uint8_t i2c_temp[2];
//	i2c_temp[0] = value>>8;
//	i2c_temp[1] = value;
//	ret = HAL_I2C_Mem_Write(&hi2c1, address<<1, (uint16_t)reg, 1, i2c_temp, 2, to);
//	if(ret != HAL_OK){
//			//			sprintf((char*)buf,"Se rompio en config\r\n");
//			//			HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), 1000);
//	//				current = 34567;
////					configIna219(INA_219_ADDR_M1,1); // Config segun el address
//					}
	osDelay(1);

	//		wireReadRegister(INA219_REG_CURRENT, &value); // reg *value
	reg = 0x04;
	uint16_t *valuee = &value;

	ret = HAL_I2C_Mem_Read(&hi2c1, address<<1, (uint16_t)reg, 1,i2c_temp, 2, to);
	if(ret != HAL_OK){
		//			sprintf((char*)buf,"Se rompio en config\r\n");
		//			HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), 1000);
//				current = 34567;
				configIna219(INA_219_ADDR_M1,100); // Config segun el address
				}
	osDelay(1);
	*valuee = ((uint16_t)i2c_temp[0]<<8 )|(uint16_t)i2c_temp[1];
	current = (int16_t)value;

	return current /= ina219_currentDivider_mA;
}





void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin){
	if (GPIO_Pin == D01_Encoder_Pin){
		ticksAux[0] = ticksPrev[0];
		ticksPrev[0] = ticksNow[0];
		ticksNow[0] = __HAL_TIM_GetCounter(&htim2);
		osSemaphoreRelease(Semaforo1Handle);
		flags_motores[0]=1;
	}

	if (GPIO_Pin == D02_Encoder_Pin){
		ticksAux[1] = ticksPrev[1];
		ticksPrev[1] = ticksNow[1];
		ticksNow[1] = __HAL_TIM_GetCounter(&htim2);
		osSemaphoreRelease(Semaforo1Handle);
		flags_motores[1]=1;
	}

	if (GPIO_Pin == D03_Encoder_Pin){
		ticksAux[2] = ticksPrev[2];
		ticksPrev[2] = ticksNow[2];
		ticksNow[2] = __HAL_TIM_GetCounter(&htim2);
		osSemaphoreRelease(Semaforo1Handle);
		flags_motores[2]=1;
	}

	if (GPIO_Pin == D04_Encoder_Pin){
		ticksAux[3] = ticksPrev[3];
		ticksPrev[3] = ticksNow[3];
		ticksNow[3] = __HAL_TIM_GetCounter(&htim2);
		osSemaphoreRelease(Semaforo1Handle);
		flags_motores[3]=1;
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
  MX_TIM1_Init();
  MX_TIM2_Init();
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

	HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_SET);
	//  	HAL_UART_Transmit(&huart3, (uint8_t*)"V\n", 3*sizeof(char), HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of Semaforo1 */
  Semaforo1Handle = osSemaphoreNew(1, 1, &Semaforo1_attributes);

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
  /* creation of Speed1 */
  Speed1Handle = osThreadNew(StartSpeed1, NULL, &Speed1_attributes);

  /* creation of Modbus */
  ModbusHandle = osThreadNew(StartModbus, NULL, &Modbus_attributes);

  /* creation of CheckVelocidad */
  CheckVelocidadHandle = osThreadNew(StartCheckVelocidad, NULL, &CheckVelocidad_attributes);

  /* creation of TaskControl */
  TaskControlHandle = osThreadNew(StartTaskControl, NULL, &TaskControl_attributes);

  /* creation of Corriente */
  CorrienteHandle = osThreadNew(StartCorriente, NULL, &Corriente_attributes);

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

  /*Configure GPIO pins : D01_Encoder_Pin D02_Encoder_Pin D03_Encoder_Pin D04_Encoder_Pin */
  GPIO_InitStruct.Pin = D01_Encoder_Pin|D02_Encoder_Pin|D03_Encoder_Pin|D04_Encoder_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
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

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartSpeed1 */
/**
 * @brief  Function implementing the Speed1 thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSpeed1 */
void StartSpeed1(void *argument)
{
  /* USER CODE BEGIN 5 */

	uint32_t ticksPrev_l = 0;
	uint32_t ticksNow_l = 0;
	uint32_t ticksAux_l = 0;
	uint32_t deltaTicks_l = 0;
	uint16_t overflow_l =0;
	float velocidad_prima2_l = 0;
	float velocidad_prima1_l = 0;
	float velocidad_l = 0;
	uint8_t flags_motores_l[4];
	uint8_t valor = 0;
	/* Infinite loop */
	for(;;)
	{
		osSemaphoreAcquire(Semaforo1Handle, osWaitForever);
		taskENTER_CRITICAL();
		flags_motores_l[0] = flags_motores[0];
		flags_motores_l[1] = flags_motores[1];
		flags_motores_l[2] = flags_motores[2];
		flags_motores_l[3] = flags_motores[3];
		taskEXIT_CRITICAL();

		if (flags_motores_l[0] == 1)
		{
			valor = 0;
		}
		else if (flags_motores_l[1] == 1)
		{
			valor = 1;
		}
		else if (flags_motores_l[2] == 1)
		{
			valor = 2;
		}
		else if (flags_motores_l[3] == 1)
		{
			valor = 3;
		}




		taskENTER_CRITICAL();
		ticksPrev_l = ticksPrev[valor];
		ticksNow_l = ticksNow[valor];
		ticksAux_l = ticksAux[valor];
		overflow_l = overflow[valor];
		velocidad_l = velocidad[valor];
		velocidad_prima1_l = velocidad_prima1[valor];
		velocidad_prima2_l = velocidad_prima2[valor];
		taskEXIT_CRITICAL();

		//		HAL_NVIC_EnableIRQ(EXTI1_IRQn);

		if (overflow_l == 0){
			// Todo cool, calculo normal
			deltaTicks_l = ticksNow_l - ticksPrev_l;
			if (deltaTicks_l > tickFilter){
				velocidad_l = ((1/(float)ranuras)/((float)deltaTicks_l/(float)fsTmr2));
				//Filtro IIR
				velocidad_prima2_l = velocidad_prima1_l;
				velocidad_prima1_l = 0.6*velocidad_prima2_l + 0.4*velocidad_l;

				taskENTER_CRITICAL();
				deltaTicks[valor] = deltaTicks_l;
				velocidad[valor] = velocidad_l;
				velocidad_prima1[valor]= velocidad_prima1_l;
				velocidad_prima2[valor] = velocidad_prima2_l;
				taskEXIT_CRITICAL();
			}
			else{
				taskENTER_CRITICAL();
				ticksNow[valor] = ticksPrev_l;
				ticksPrev[valor] = ticksAux_l;
				taskEXIT_CRITICAL();
			}
		} else{
			// Tuve algun desborde y tengo que tenerlo en cuenta
			deltaTicks_l = (ticksNow_l + overflow_l * cantTicksTmr2)- ticksPrev_l;/////////////////////////////////////////
			if (deltaTicks_l > tickFilter){
				velocidad_l = ((1/(float)ranuras)/((float)deltaTicks_l/(float)fsTmr2));
				//Filtro IIR
				velocidad_prima2_l = velocidad_prima1_l;
				velocidad_prima1_l = 0.6*velocidad_prima2_l + 0.4*velocidad_l;

				taskENTER_CRITICAL();
				overflow[valor] = 0;
				deltaTicks[valor] = deltaTicks_l;
				velocidad[valor] = velocidad_l;
				velocidad_prima1[valor] = velocidad_prima1_l;
				velocidad_prima2[valor] = velocidad_prima2_l;
				taskEXIT_CRITICAL();
			}
			else{
				taskENTER_CRITICAL();
				ticksNow[valor] = ticksPrev_l;
				ticksPrev[valor] = ticksAux_l;
				taskEXIT_CRITICAL();
			}
		}

		if (valor == 2 && current > 0 ){
			taskENTER_CRITICAL();
			velocidad[valor] = -velocidad[valor];
			taskEXIT_CRITICAL();
		}

		taskENTER_CRITICAL();
		flags_motores_l[valor] = 0;
		flags_motores[valor] = 0;
		taskEXIT_CRITICAL();
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
	uint16_t delta1[2];// para mandar los deltaticks
	uint16_t delta2[2];
	uint16_t delta3[2];
	uint16_t delta4[2];
	uint16_t delta5[2];
	//	uint16_t delta3[2];
	//	uint16_t delta4[2];

	float velocidad[4]={'\0'};
	uint16_t ModbusDATA_l[24] = {'\0'};
	/* Infinite loop */
	for(;;)
	{
		taskENTER_CRITICAL();
		velocidad[0] = velocidad_prima1[0];
		velocidad[1] = velocidad_prima1[1];
		velocidad[2] = velocidad_prima1[2];
		velocidad[3] = velocidad_prima1[3];
		taskEXIT_CRITICAL();


		memcpy(delta1, &velocidad[0], sizeof(velocidad[0]));
		ModbusDATA_l[2]=delta1[0];
		ModbusDATA_l[3]=delta1[1];

		memcpy(delta2, &velocidad[1], sizeof(velocidad[1]));
		ModbusDATA_l[8]=delta2[0];
		ModbusDATA_l[9]=delta2[1];

		memcpy(delta3, &velocidad[2], sizeof(velocidad[2]));
		ModbusDATA_l[14]=delta3[0];
		ModbusDATA_l[15]=delta3[1];

		memcpy(delta4, &velocidad[3], sizeof(velocidad[3]));
		ModbusDATA_l[20]=delta4[0];
		ModbusDATA_l[21]=delta4[1];


		//Corriente
		memcpy(delta5, &current, sizeof(current));
		ModbusDATA_l[10]=delta5[0];
		ModbusDATA_l[11]=delta5[1];


		taskENTER_CRITICAL();

		ModbusDATA[2] = ModbusDATA_l[2];
		ModbusDATA[3] = ModbusDATA_l[3];
		ModbusDATA[4] = ModbusDATA_l[4];
		ModbusDATA[5] = ModbusDATA_l[5];

		ModbusDATA[8] = ModbusDATA_l[8];
		ModbusDATA[9] = ModbusDATA_l[9];
		ModbusDATA[10] = ModbusDATA_l[10];
		ModbusDATA[11] = ModbusDATA_l[11];

		ModbusDATA[14] = ModbusDATA_l[14];
		ModbusDATA[15] = ModbusDATA_l[15];
		ModbusDATA[16] = ModbusDATA_l[16];
		ModbusDATA[17] = ModbusDATA_l[17];

		ModbusDATA[20] = ModbusDATA_l[20];
		ModbusDATA[21] = ModbusDATA_l[21];
		ModbusDATA[22] = ModbusDATA_l[22];
		ModbusDATA[23] = ModbusDATA_l[23];
		taskEXIT_CRITICAL();

		osDelay(50);
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

	uint16_t overflow_l[4] ={'\0'};

	/* Infinite loop */
	for(;;)
	{
		taskENTER_CRITICAL();
		overflow_l[0] = overflow[0];
		overflow_l[1] = overflow[1];
		overflow_l[2] = overflow[2];
		overflow_l[3] = overflow[3];
		taskEXIT_CRITICAL();

		if(overflow_l[0] >= 3){
			overflow_l[0] = 0;
			taskENTER_CRITICAL();
			overflow[0] = 0;
			velocidad_prima2[0] = 0;
			velocidad_prima1[0] = 0;
			velocidad[0] = 0;
			taskEXIT_CRITICAL();
		}
		if(overflow_l[1] >= 3){
			overflow_l[1] = 0;
			taskENTER_CRITICAL();
			overflow[1] = 0;
			velocidad_prima2[1] = 0;
			velocidad_prima1[1] = 0;
			velocidad[1] = 0;
			taskEXIT_CRITICAL();
		}
		if(overflow_l[2] >= 3){
			overflow_l[2] = 0;
			taskENTER_CRITICAL();
			overflow[2] = 0;
			velocidad_prima2[2] = 0;
			velocidad_prima1[2] = 0;
			velocidad[2] = 0;
			taskEXIT_CRITICAL();
		}
		if(overflow_l[3] >= 3){
			overflow_l[3] = 0;
			taskENTER_CRITICAL();
			overflow[3] = 0;
			velocidad_prima2[3] = 0;
			velocidad_prima1[3] = 0;
			velocidad[3] = 0;
			taskEXIT_CRITICAL();
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
	float velocidad_l[4]={'\0'};
	float Setpoint[4] = {'\0'};
	uint16_t Sentido_l[4]={'\0'};

	//	float error=0;

	/* Infinite loop */
	for(;;)
	{
		taskENTER_CRITICAL();
		velocidad_l[0] = velocidad[0];
		velocidad_l[1] = velocidad[1];
		velocidad_l[2] = velocidad[2];
		velocidad_l[3] = velocidad[3];

		Setpoint[0] = (float)ModbusDATA[0]/1000.0;
		Setpoint[1] = (float)ModbusDATA[6]/1000.0;
		Setpoint[2] = (float)ModbusDATA[12]/1000.0;
		Setpoint[3] = (float)ModbusDATA[18]/1000.0;

		Sentido_l[0] = ModbusDATA[1];
		Sentido_l[1] = ModbusDATA[7];
		Sentido_l[2] = ModbusDATA[13];
		Sentido_l[3] = ModbusDATA[19];
		taskEXIT_CRITICAL();

		if(Sentido_l[1] == 1 && current > 5){
			Sentido(0, 2);
			Setpoint[1] = -Setpoint[1];
		}

		if(Sentido_l[1] == 1 && current <= 5){
			Sentido(1, 2);
			Setpoint[1] = Setpoint[1];
		}

		if(Sentido_l[1] == 0 && current <= -5){
			Sentido(1, 2);
			Setpoint[1] = -Setpoint[1];
		}

		if(Sentido_l[1] == 0 && current > -5){
			Sentido(0, 2);
			Setpoint[1] = Setpoint[1];
		}

//		if (current <= 0){
//			velocidad_l[1] = -velocidad_l[1];
//		}


		rtEntrada_Control1 = Setpoint[0] - velocidad_l[0];
		rtEntrada_Control2 = Setpoint[1] - velocidad_l[1];
		rtEntrada_Control3 = Setpoint[2] - velocidad_l[2];
		rtEntrada_Control4 = Setpoint[3] - velocidad_l[3];
		control_step(); //Ejecutamos control


//		if (rtSalida_Control1 < 0){
//			Sentido(1, 2);
//		}

		rtEntrada_Linealizacion1 = rtSalida_Control1;	//Salida PID asignada a entrada de planta linealizadora
		rtEntrada_Linealizacion2 = rtSalida_Control2;
		rtEntrada_Linealizacion3 = rtSalida_Control3;
		rtEntrada_Linealizacion4 = rtSalida_Control4;

		Linealizacion_step();	//Ejecutamos planta linealizadora

		htim1.Instance->CCR1 = (uint32_t)rtSalida_Linealizacion1;	//Salida linealizada asignada a CCR
		htim1.Instance->CCR2 = (uint32_t)rtSalida_Linealizacion2;
		htim1.Instance->CCR3 = (uint32_t)rtSalida_Linealizacion3;
		htim1.Instance->CCR4 = (uint32_t)rtSalida_Linealizacion4;

		//		if (htim1.Instance->CCR1 != ModbusDATA[0]){
		//			htim1.Instance->CCR1 = ModbusDATA[0];
		//		}
		//		if (htim1.Instance->CCR2 != ModbusDATA[6]){
		//			htim1.Instance->CCR2 = ModbusDATA[6];
		//		}
		//		if (htim1.Instance->CCR3 != ModbusDATA[12]){
		//			htim1.Instance->CCR3 = ModbusDATA[12];
		//		}
		//		if (htim1.Instance->CCR4 != ModbusDATA[18]){
		//			htim1.Instance->CCR4 = ModbusDATA[18];
		//		}



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

/* USER CODE BEGIN Header_StartCorriente */
/**
 * @brief Function implementing the Corriente thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCorriente */
void StartCorriente(void *argument)
{
  /* USER CODE BEGIN StartCorriente */
	/* Infinite loop */
	float current_l = 0;
	configIna219(INA_219_ADDR_M1,100); // Config segun el address
		for(;;)
		{
			current_l = getCurrent(INA_219_ADDR_M1,100);
			taskENTER_CRITICAL();
			current = current_l;
			taskEXIT_CRITICAL();



			osDelay(70);

		}
  /* USER CODE END StartCorriente */
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
			overflow[0] += 1;
			overflow[1] += 1;
			overflow[2] += 1;
			overflow[3] += 1;


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

