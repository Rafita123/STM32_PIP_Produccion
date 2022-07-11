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

#include <math.h>

//#include "INA219.h"
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
		.stack_size = 1024 * 4,
		.priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Semaforo1 */
osSemaphoreId_t Semaforo1Handle;
const osSemaphoreAttr_t Semaforo1_attributes = {
		.name = "Semaforo1"
};
/* USER CODE BEGIN PV */

//	Registers
//
#define	INA219_REG_CONFIG						(0x00)
#define	INA219_REG_SHUNTVOLTAGE					(0x01)
#define	INA219_REG_BUSVOLTAGE					(0x02)
#define	INA219_REG_POWER						(0x03)
#define	INA219_REG_CURRENT						(0x04)
#define	INA219_REG_CALIBRATION					(0x05)

#define INA219_CONFIG_BVOLTAGERANGE_32V			(0x2000) // 0-32V Range
#define	INA219_CONFIG_GAIN_8_320MV				(0x1800) // Gain 8, 320mV Range
#define	INA219_CONFIG_BADCRES_12BIT				(0x0180) // 12-bit bus res = 0..4097

#define	INA219_CONFIG_SADCRES_9BIT_1S_84US		(0x0000) // 1 x 9-bit shunt sample
#define	INA219_CONFIG_SADCRES_10BIT_1S_148US	(0x0008) // 1 x 10-bit shunt sample
#define	INA219_CONFIG_SADCRES_11BIT_1S_276US	(0x0010) // 1 x 11-bit shunt sample
#define	INA219_CONFIG_SADCRES_12BIT_1S_532US	(0x0018) // 1 x 12-bit shunt sample
#define	INA219_CONFIG_SADCRES_12BIT_2S_1060US	(0x0048) // 2 x 12-bit shunt samples averaged together
#define	INA219_CONFIG_SADCRES_12BIT_4S_2130US	(0x0050) // 4 x 12-bit shunt samples averaged together
#define	INA219_CONFIG_SADCRES_12BIT_8S_4260US	(0x0058) // 8 x 12-bit shunt samples averaged together
#define	INA219_CONFIG_SADCRES_12BIT_16S_8510US	(0x0060) // 16 x 12-bit shunt samples averaged together
#define	INA219_CONFIG_SADCRES_12BIT_32S_17MS	(0x0068) // 32 x 12-bit shunt samples averaged together
#define	INA219_CONFIG_SADCRES_12BIT_64S_34MS	(0x0070) // 64 x 12-bit shunt samples averaged together
#define	INA219_CONFIG_SADCRES_12BIT_128S_69MS	(0x0078) // 128 x 12-bit shunt samples averaged together

#define	INA219_CONFIG_MODE_SVOLT_CONTINUOUS		(0x05) /**< shunt voltage continuous */

//
//	Addresses
//
#define INA219_ADDRESS_1 (0x41) // A0 puenteado
#define INA219_ADDRESS_2 (0x44) // A1 puenteado
#define INA219_ADDRESS_3 (0x45) // A0 y A1 puenteados
#define INA219_ADDRESS_4 (0x40) // Nada puenteado

#define INA219_ADDRESS_5 (0x4C) // Corriente placa de control
#define INA219_ADDRESS_6 (0x42) // Corriente sobre R conocida para tension de bus

#define  N_motores       (0x04)
#define  N_corriente     (0x06)
#define  N_Modbus        (0x20) //32
#define  N_dato          (0x02)
#define  N_tasks		 (0x06)

#define  SPEED_TASK 	 (0X00)
#define  MODBUS_TASK 	 (0X01)
#define  ZERO_SPEED_TASK (0X02)
#define  CONTROL_TASK 	 (0X03)
#define  CURRENT_TASK 	 (0X04)

HAL_StatusTypeDef ret; // Con ret veo los estados retornados por HAL

//---------------->  Modbus
modbusHandler_t ModbusH;
uint16_t ModbusDATA[N_Modbus];  // Mapa modbus!

//---------------->
uint32_t ranuras = 50;
uint32_t cantTicksTmr2 = 20000;
uint64_t fsTmr2= 50000;
uint64_t tickFilter = 600; // Parametro delicado, puede hacer cagadas en la medicion de la velocidad, OJO

//Datos referidos a la velocidad
float velocidad[N_motores]; //Velocidad cruda
float velocidad_prima1[N_motores]; //Velocidad filtrada
float velocidad_prima2[N_motores];
uint8_t flags_motores[N_motores];// Bandera para saber con que motor estamos trabajando
uint8_t sentidoDeGiro[N_motores];
uint8_t bufferPing[N_tasks-1];

// Datos utilizados para el calculo de la velocidad
uint32_t ticksPrev[N_motores];
uint32_t ticksNow[N_motores];
uint32_t ticksAux[N_motores];
uint32_t deltaTicks[N_motores];

uint16_t overflow[N_motores]; // Cantidad de desbordes del timer

//float current = 0;
float current[N_corriente]; // Corriente de motores, bus y corriente de placa de control


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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


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
			HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, SET);

		}
		else{ // Break
			HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, RESET);
			HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, RESET);
		}
	}
	sentidoDeGiro[motor-1]= valor;

}



void configIna219(uint8_t address, uint16_t TO){

	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
			INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
			INA219_CONFIG_SADCRES_12BIT_128S_69MS |
			INA219_CONFIG_MODE_SVOLT_CONTINUOUS;

	uint16_t ina219_calibrationValue = 4096;

	//	INA219_setCalibration(ina219, ina219_calibrationValue);//(&hi2c1,ina219_calibrationValue)
	uint8_t addr[2];
	addr[0] = (ina219_calibrationValue >> 8) & 0xff;  // upper byte
	addr[1] = (ina219_calibrationValue >> 0) & 0xff; // lower byte
	HAL_I2C_Mem_Write(&hi2c1, (address<<1), INA219_REG_CALIBRATION, 1, (uint8_t*)addr, 2, TO);



	//	INA219_setConfig(ina219, config);
	addr[0] = (config >> 8) & 0xff;  // upper byte
	addr[1] = (config >> 0) & 0xff; // lower byte
	HAL_I2C_Mem_Write(&hi2c1, (address<<1), INA219_REG_CONFIG, 1, (uint8_t*)addr, 2, TO);
}

float getCurrent(uint8_t address, uint16_t TO){

	float current = 65.535;
	uint8_t Value[2];
	int16_t result = 0;
	int16_t ina219_currentDivider_mA = 10;
	//	int16_t result = INA219_ReadCurrent_raw(ina219); // read en INA219_REG_CURRENT
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Mem_Read(&hi2c1, (address<<1), INA219_REG_CURRENT, 1, Value, 2, TO);
	if(ret == HAL_OK){
		result = ((Value[0] << 8) | Value[1]); // RawCurrent
		current = ((float)result / (float)ina219_currentDivider_mA);
	}
	return current;
}

float readMotor(uint8_t address, uint16_t TO){
	HAL_StatusTypeDef ret1;
	float current;
	ret1 = HAL_I2C_IsDeviceReady(&hi2c1, address<<1, 1, TO);
	if(ret1 == HAL_OK){
		configIna219(address,TO);
		current = getCurrent(address,TO);
	}else{
		// Error control
		HAL_I2C_DeInit(&hi2c1);
		HAL_I2C_Init(&hi2c1);
		configIna219(address,TO);
		current = 65.535; // Estado que indica que la corriente no pudo ser leida
	}
	return current;
}

float deteccionCero(uint8_t n_motor, uint8_t sentido_, uint8_t cota, float velocidad, float referencia, uint8_t sentidoActual){
	if(sentido_ == 1 && sentido_!= sentidoActual){//velocidad > cota
		Sentido(1, n_motor);
		referencia = -referencia;
	}else if(sentido_ == 1 && sentido_== sentidoActual ){//corriente <= cota
		Sentido(1, n_motor);
		referencia = referencia;
	}
	else if(sentido_ == 0 && sentido_!= sentidoActual){//velocidad <= -cota
		Sentido(0, n_motor);
		referencia = -referencia;
	}else if(sentido_ == 0 && sentido_== sentidoActual){// corriente > -cota
		Sentido(0, n_motor);
		referencia= referencia;
	}
	return referencia;
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
	//  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, IN2_1_Pin|IN2_2_Pin|IN1_2_Pin|IN1_1_Pin
			|IN3_2_Pin|IN3_1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, IN4_1_Pin|IN4_2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_Pin */
	//  GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	//  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

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

	for(int i=0; i<N_motores; i++){
		sentidoDeGiro[i] = 0;
	}
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
			// Tod o cool, calculo normal
			deltaTicks_l = ticksNow_l - ticksPrev_l;
			if (deltaTicks_l > tickFilter){

//				if(sentidoDeGiro[valor] == 1) deltaTicks_l=-de7ltaTicks_l;

				velocidad_l = ((1/(float)ranuras)/((float)deltaTicks_l/(float)fsTmr2));
				//Filtro IIR
				velocidad_prima2_l = velocidad_prima1_l;
				velocidad_prima1_l = 0.95*velocidad_prima2_l + 0.05*velocidad_l;

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
			deltaTicks_l = (ticksNow_l + overflow_l * cantTicksTmr2)- ticksPrev_l;
			if (deltaTicks_l > tickFilter){
//				if(sentidoDeGiro[valor] == 1) deltaTicks_l=-deltaTicks_l;

				velocidad_l = ((1/(float)ranuras)/((float)deltaTicks_l/(float)fsTmr2));
				//Filtro IIR
				velocidad_prima2_l = velocidad_prima1_l;
				velocidad_prima1_l = 0.95*velocidad_prima2_l + 0.05*velocidad_l;

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

		for(int i=0 ; i < N_motores ; i++){
			taskENTER_CRITICAL();
			flags_motores_l[valor] = 0;
			flags_motores[valor] = 0;
			taskEXIT_CRITICAL();
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
	uint16_t delta1[2];// para mandar los deltaticks
	uint16_t delta2[2];
	uint16_t delta3[2];
	uint16_t delta4[2];
	uint16_t delta5[2];
	//	uint16_t delta3[2];
	//	uint16_t delta4[2];

	float velocidad[4]={'\0'};
	uint16_t ModbusDATA_l[N_Modbus] = {'\0'};
	/* Infinite loop */
	for(;;)
	{
		bufferPing[MODBUS_TASK] = 1; //pin para watchdog

		taskENTER_CRITICAL();
		velocidad[0] = velocidad_prima1[0];
		velocidad[1] = velocidad_prima1[1];
		velocidad[2] = velocidad_prima1[2];
		velocidad[3] = velocidad_prima1[3];
		taskEXIT_CRITICAL();

		for(int i = 0; i < N_motores; i++){
			if(current[i] < 0) velocidad[i] = -velocidad[i];
		}

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

		memcpy(delta5, &current[0], sizeof(current[0])); // Motor 1
		ModbusDATA_l[4]=delta5[0];
		ModbusDATA_l[5]=delta5[1];

		memcpy(delta5, &current[1], sizeof(current[1])); // Motor 2
		ModbusDATA_l[10]=delta5[0];
		ModbusDATA_l[11]=delta5[1];

		memcpy(delta5, &current[2], sizeof(current[2])); // Motor 3
		ModbusDATA_l[16]=delta5[0];
		ModbusDATA_l[17]=delta5[1];

		memcpy(delta5, &current[3], sizeof(current[3])); // Motor 4
		ModbusDATA_l[22]=delta5[0];
		ModbusDATA_l[23]=delta5[1];

		memcpy(delta5, &current[4], sizeof(current[4])); // Placa Control
		ModbusDATA_l[24]=delta5[0];
		ModbusDATA_l[25]=delta5[1];

		memcpy(delta5, &current[5], sizeof(current[5])); // Vbus
		ModbusDATA_l[26]=delta5[0];
		ModbusDATA_l[27]=delta5[1];

		//		ModbusDATA_l[28] // Armado del sistema


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

		ModbusDATA[24] = ModbusDATA_l[24];
		ModbusDATA[25] = ModbusDATA_l[25];
		ModbusDATA[26] = ModbusDATA_l[26];
		ModbusDATA[27] = ModbusDATA_l[27];

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
		bufferPing[ZERO_SPEED_TASK] = 1; //pin para watchdog
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
	uint8_t parada=0;
	uint8_t cota = 50;


	//	float error=0;

	/* Infinite loop */
	for(;;)
	{
		bufferPing[CONTROL_TASK] = 1; //pin para watchdog

		taskENTER_CRITICAL();
//		velocidad_l[0] = velocidad[0]; //MOTOR1
//		velocidad_l[1] = velocidad[1]; //MOTOR2
//		velocidad_l[2] = velocidad[2]; //MOTOR3
//		velocidad_l[3] = velocidad[3]; //MOTOR4

		velocidad_l[0] = velocidad_prima1[0]; //MOTOR1
		velocidad_l[1] = velocidad_prima1[1]; //MOTOR2
		velocidad_l[2] = velocidad_prima1[2]; //MOTOR3
		velocidad_l[3] = velocidad_prima1[3]; //MOTOR4


		Setpoint[0] = (float)ModbusDATA[0]/1000.0; //SETPOINT MOTOR1
		Setpoint[1] = (float)ModbusDATA[6]/1000.0; //SETPOINT MOTOR2
		Setpoint[2] = (float)ModbusDATA[12]/1000.0; //SETPOINT MOTOR3
		Setpoint[3] = (float)ModbusDATA[18]/1000.0; //SETPOINT MOTOR4

		Sentido_l[0] = ModbusDATA[1]; //SENTIDO MOTOR 1
		Sentido_l[1] = ModbusDATA[7]; //SENTIDO MOTOR 2
		Sentido_l[2] = ModbusDATA[13]; //SENTIDO MOTOR 3
		Sentido_l[3] = ModbusDATA[19]; //SENTIDO MOTOR 4

		/////////////// FUNCION PARA PARADA GENERAL /////////////////////////////////
		parada = ModbusDATA[28];
		taskEXIT_CRITICAL();

		if(parada == 0){
			for(int i=0;i<N_motores;i++){
				Setpoint[i]=0;
			}
		}

		/////////////////////////////////////////////////////////////////////////////

		Setpoint[0] = deteccionCero(1, Sentido_l[0], cota, current[0], Setpoint[0], sentidoDeGiro[0]);
		Setpoint[1] = deteccionCero(2, Sentido_l[1], cota, current[1], Setpoint[1], sentidoDeGiro[1]);
		Setpoint[2] = deteccionCero(4, Sentido_l[2], cota, current[2], Setpoint[2], sentidoDeGiro[3]);// Referencia cruzada de motores y CCR
		Setpoint[3] = deteccionCero(3, Sentido_l[3], cota, current[3], Setpoint[3], sentidoDeGiro[2]);// Referencia cruzada de motores y CCR


		rtEntrada_Control1 = Setpoint[0] - velocidad_l[0];
		rtEntrada_Control2 = Setpoint[1] - velocidad_l[1];
		rtEntrada_Control3 = Setpoint[2] - velocidad_l[2];
		rtEntrada_Control4 = Setpoint[3] - velocidad_l[3];
		control_step(); //Ejecutamos control


		rtEntrada_Linealizacion1 = rtSalida_Control1;	//Salida PID asignada a entrada de planta linealizadora
		rtEntrada_Linealizacion2 = rtSalida_Control2;
		rtEntrada_Linealizacion3 = rtSalida_Control3;
		rtEntrada_Linealizacion4 = rtSalida_Control4;

		Linealizacion_step();	//Ejecutamos planta linealizadora

		htim1.Instance->CCR1 = (uint32_t)rtSalida_Linealizacion1;	//Salida linealizada asignada a CCR
		htim1.Instance->CCR2 = (uint32_t)rtSalida_Linealizacion2;
		htim1.Instance->CCR4 = (uint32_t)rtSalida_Linealizacion3;// Referencia cruzada de motores y CCR
		htim1.Instance->CCR3 = (uint32_t)rtSalida_Linealizacion4;// Referencia cruzada de motores y CCR


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
	float current_l[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	float current_prima1_l[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	float current_prima2_l[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	uint16_t timeOut = 2;
	configIna219(INA219_ADDRESS_1,timeOut); // Config segun el address
	configIna219(INA219_ADDRESS_2,timeOut); // Config segun el address
	configIna219(INA219_ADDRESS_3,timeOut); // Config segun el address
	configIna219(INA219_ADDRESS_4,timeOut); // Config segun el address

	for(;;)
	{
		bufferPing[CURRENT_TASK] = 1; //pin para watchdog

		current_l[0] = readMotor(INA219_ADDRESS_1,timeOut)*2.0;
		if(current_l[0] != 65.535){	// 65535 Es un estado que significa que el I2c estaba ocupado o no pudo leer la corriente
			// por lo que dejo el valor anterior
			current_prima2_l[0] = current_prima1_l[0];
			current_prima1_l[0] = 0.85*current_prima2_l[0] + 0.15*current_l[0];
			taskENTER_CRITICAL();
			//			current[0] = current_l[0];
			current[0] = current_prima1_l[0];
			taskEXIT_CRITICAL();
		}

		current_l[1] = readMotor(INA219_ADDRESS_2,timeOut)*2.0;
		if(current_l[1] != 65.535){	// 65535 Es un estado que significa que el I2c estaba ocupado o no pudo leer la corriente
			// por lo que dejo el valor anterior
			current_prima2_l[1] = current_prima1_l[1];
			current_prima1_l[1] = 0.85*current_prima2_l[1] + 0.15*current_l[1];
			taskENTER_CRITICAL();
			//			current[1] = current_l[1];
			current[1] = current_prima1_l[1];
			taskEXIT_CRITICAL();
		}

		current_l[2] = readMotor(INA219_ADDRESS_3,timeOut)*2.0;
		if(current_l[2] != 65.535){	// 65535 Es un estado que significa que el I2c estaba ocupado o no pudo leer la corriente
			// por lo que dejo el valor anterior
			current_prima2_l[2] = current_prima1_l[2];
			current_prima1_l[2] = 0.85*current_prima2_l[2] + 0.15*current_l[2];
			taskENTER_CRITICAL();
//			current[2] = current_l[2];
			current[2] = current_prima1_l[2];
			taskEXIT_CRITICAL();
		}

//		current_l[3] = readMotor(INA219_ADDRESS_4,timeOut)*2.0;
//		if(current_l[3] != 65.535){	// 65535 Es un estado que significa que el I2c estaba ocupado o no pudo leer la corriente
//			// por lo que dejo el valor anterior
//			current_prima2_l[3] = current_prima1_l[3];
//			current_prima1_l[3] = 0.85*current_prima2_l[3] + 0.15*current_l[3];
//			taskENTER_CRITICAL();
////			current[3] = current_l[3];
//			current[3] = current_prima1_l[3];
//			taskEXIT_CRITICAL();
//		}
		//
//		current_l[4] = readMotor(INA219_ADDRESS_5,timeOut);
//		if(current_l[4] != 65.535){	// 65535 Es un estado que significa que el I2c estaba ocupado o no pudo leer la corriente
//			// por lo que dejo el valor anterior
//			taskENTER_CRITICAL();
//			current[4] = current_l[4];
//			taskEXIT_CRITICAL();
//		}

		//		OJO OJETE HACER LA CONVERSION DE LA TENSION DEL BUS EN EL QUE CORRESPONDA

		current_l[5] = readMotor(INA219_ADDRESS_6,timeOut);
		if(current_l[5] != 65.535){	// 65535 Es un estado que significa que el I2c estaba ocupado o no pudo leer la corriente
			// por lo que dejo el valor anterior
			taskENTER_CRITICAL();
			current[5] = current_l[5]*3.3;
			taskEXIT_CRITICAL();
		}


		osDelay(10);
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
