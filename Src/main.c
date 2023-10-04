/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "stdlib.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "string.h"
#include "rosserial.h"
#include "math.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Define motor parameters */
#define POLES 30
#define ENCODER_TICKS 4096.
#define MAX_CURRENT 8
#define SAMPLE_TIME_MS 0.01 // PID control time(s)

// PID Gain ?
#define KP_L 0.161f//0.161f//0.06938f//0.029//0.0028f
#define KI_L 0.8f//0.805f//0.0192f//0.097f//77.5711f//0.002f
#define KP_R 0.161f//0.161f//0.06938f//0.029//4.9383f
#define KI_R 0.8f//0.805f//0.0172f//0.097f//0.77//77.5711f

// PWM
#define MAX_DUTY 7000 // PWM
#define MIN_DUTY 0    // PWM
#define MAX_integral 30 // PWM
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
volatile int32_t dir1 = 0;
volatile int32_t dir2 = 0;


// TIM3 encoder
volatile int32_t TIM3_CH1_val_prev = 0; //encoder L prev
volatile int32_t TIM3_CH1_val = 0; // encoder L
volatile int32_t diff1 = 0;
volatile int32_t mode = 0;
volatile int32_t mode_new = 0;
extern volatile bool mode_start;

// TIM4 encoder
volatile int32_t TIM4_CH1_val_prev = 0; //encoder R prev
volatile int32_t TIM4_CH1_val = 0; // encoder R
volatile int32_t diff2 = 0;

// PID
volatile int32_t target_speed_L = 0;  // (rpm)
volatile int32_t target_speed_R = 0;  // (rpm)
volatile float current_speed_L = 0.;   // (rpm)
volatile float current_speed_R = 0.;   // (rpm)
volatile float error1 = 0.;
volatile float error2 = 0.;
volatile float integral1 = 0.;
volatile float integral2 = 0.;
volatile float pid_output_L = 0.;
volatile float pid_output_R = 0.;
volatile int32_t pwm_duty_L = MIN_DUTY;
volatile int32_t pwm_duty_R = MIN_DUTY;
volatile int32_t pwm_duty_L1 = MIN_DUTY;
volatile int32_t pwm_duty_R1 = MIN_DUTY;
volatile uint32_t adcVal[1];
volatile uint32_t batPer_cur;
volatile uint32_t batPer;
volatile uint32_t battery_count;
volatile uint32_t battery_sum1;
volatile uint32_t mode_cnt = 0;
volatile uint32_t mode_cnt2 = 0;
volatile float batVolt;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
static float vcontrol_L(float);
static float vcontrol_R(float);
static void Direction_L();
static void Direction_R();
static void SetDutyCW_L(float);
static void SetDutyCW_R(float);
static uint32_t battery_check(uint32_t);
static uint32_t battery_sum(uint32_t);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SetDutyCW_L(float pid_output_L){

	int32_t duty_output_L = (int32_t)pid_output_L * 45; //100rpm -> 4500duty
	pwm_duty_L += duty_output_L;
	pwm_duty_L1 = abs(pwm_duty_L);

	if (pwm_duty_L1 >= MAX_DUTY)
	{
		TIM3_CH1_val_prev = 0;
		TIM3_CH1_val = 0;
		diff1 = 0;
		TIM4_CH1_val_prev = 0;
		TIM4_CH1_val = 0;
		diff2 = 0;
		target_speed_L = 0;  // (rpm)
		target_speed_R = 0;  // (rpm)
		current_speed_L = 0;   // (rpm)
		current_speed_R = 0;   // (rpm)
		error1 = 0;
		error2 = 0;
		integral1 = 0;
		integral2 = 0;
		pid_output_L = 0;
		pid_output_R = 0;
		pwm_duty_L = MIN_DUTY;
		pwm_duty_R = MIN_DUTY;
		pwm_duty_L1 = 0;
		pwm_duty_R1 = 0;
		TIM1->CCR1 = 0;
		TIM8->CCR1 = 0;
	}

	TIM1->CCR1 = pwm_duty_L1;
}
void SetDutyCW_R(float pid_output_R){

	int32_t duty_output_R = (int32_t)pid_output_R * 45; //100rpm -> 4500duty
	pwm_duty_R += duty_output_R;
	pwm_duty_R1 = abs(pwm_duty_R);

	if (pwm_duty_R1 >= MAX_DUTY)
	{
		TIM3_CH1_val_prev = 0;
		TIM3_CH1_val = 0;
		diff1 = 0;
		TIM4_CH1_val_prev = 0;
		TIM4_CH1_val = 0;
		diff2 = 0;
		target_speed_L = 0;  // (rpm)
		target_speed_R = 0;  // (rpm)
		current_speed_L = 0;   // (rpm)
		current_speed_R = 0;   // (rpm)
		error1 = 0;
		error2 = 0;
		integral1 = 0;
		integral2 = 0;
		pid_output_L = 0;
		pid_output_R = 0;
		pwm_duty_L = MIN_DUTY;
		pwm_duty_R = MIN_DUTY;
		pwm_duty_L1 = 0;
		pwm_duty_R1 = 0;
		TIM1->CCR1 = 0;
		TIM8->CCR1 = 0;
	}

	TIM8->CCR1 = pwm_duty_R1;
}

static void Direction_L() // left direction setting
{

	if(target_speed_L >= 0) // target:front,current:front
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		dir1 = 1;// front direction (left)   left->front is SET = 1!!
	}
	else if(target_speed_L < 0 ) // target:back , current:front
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		dir1 = 0;// front direction (left)
	}
}
static void Direction_R() // Right direction setting
{

	if(target_speed_R >= 0) // target:front,current:front
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		dir2 = 0;// front direction (Right)   Right->front is RESET = 0!!
	}
	else if(target_speed_R < 0) // target:back , current:front
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
		dir2 = 1;// front direction (Right)
	}
}

float vcontrol_L(float current_speed)
{
	// PI
	float pid_output1 = 0.;
	target_speed_L = tar_speed_L;
	//error = target_speedL - current_speed;
    if(target_speed_L*current_speed < 0)
    {
    	current_speed = 0;
    }
	error1 = target_speed_L - current_speed;
	//integral1 += error1*SAMPLE_TIME_MS;
	if(pwm_duty_L >= MAX_DUTY)
	{
		integral1 = 0;
	}
	else
	{
		integral1 += error1*SAMPLE_TIME_MS;
	}

	if(integral1 > MAX_integral)
	{
		integral1 = MAX_integral;
	}
	else if(integral1 < -MAX_integral)
	{
		integral1 = -MAX_integral;
	}

	pid_output1 = KP_L*error1 + KI_L*integral1;

	return pid_output1;
}

float vcontrol_R(float current_speed)
{
	// PI
	float pid_output2 = 0.;
	target_speed_R = tar_speed_R;
	//error = target_speedR - current_speed;
	if(target_speed_R*current_speed < 0)
	{
	    current_speed = 0;
	}
	error2 = target_speed_R - current_speed;
	//integral2 += error2*SAMPLE_TIME_MS;
	if(pwm_duty_R >= MAX_DUTY)
	{
		integral2 = 0;
	}
	else
	{
		integral2 += error1*SAMPLE_TIME_MS;
	}
	if(integral2 > MAX_integral)
	{
		integral2 = MAX_integral;
	}
	else if(integral2 < -MAX_integral)
	{
		integral2 = -MAX_integral;
	}

	pid_output2 = KP_R * error2 + KI_R * integral2;

	return pid_output2;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//0.01s
{
	if(htim == &htim9 && mode_new)
	{

		TIM3_CH1_val = TIM3->CNT;
		TIM4_CH1_val = TIM4->CNT;

		diff1 = (TIM3_CH1_val - TIM3_CH1_val_prev);
		diff2 = (TIM4_CH1_val - TIM4_CH1_val_prev);

		if(TIM3_CH1_val > 60000 && TIM3_CH1_val_prev < 5000)
		{
			diff1 -= 65535;
		}
		else if(TIM3_CH1_val < 5000 && TIM3_CH1_val_prev > 60000)
		{
			diff1 += 65535;
		}

		if(TIM4_CH1_val > 60000 && TIM4_CH1_val_prev < 5000)
		{
			diff2 -= 65535;
		}
		else if(TIM4_CH1_val < 5000 && TIM4_CH1_val_prev > 60000)
		{
			diff2 += 65535;
		}


		current_speed_L= (float)diff1*60/4096/4/SAMPLE_TIME_MS; //rps->rpm
		current_speed_R= (float)diff2*60/4096/4/SAMPLE_TIME_MS; //rps->rpm


		Direction_L();
		Direction_R();

		pid_output_L = vcontrol_L(current_speed_L);
		pid_output_R = vcontrol_R(current_speed_R);

		SetDutyCW_L(pid_output_L);
		SetDutyCW_R(pid_output_R);

		TIM3_CH1_val_prev = TIM3_CH1_val;
		TIM4_CH1_val_prev = TIM4_CH1_val;

		/*if(-1 <= target_speed_L <= 1 && -1 <= target_speed_R <= 1)
		{
			mode_cnt2++;
			if(mode_cnt2 >= 500)
			{
				mode_cnt2 = 0;
				mode_start = false;
			}
		}
		else
		{
			mode_cnt2 = 0;
			mode_start = true;
		}*/

	}
	if(mode_start == false)
	{
		TIM3_CH1_val_prev = 0;
		TIM3_CH1_val = 0;
		diff1 = 0;
		TIM4_CH1_val_prev = 0;
		TIM4_CH1_val = 0;
		diff2 = 0;
		target_speed_L = 0;  // (rpm)
		target_speed_R = 0;  // (rpm)
		current_speed_L = 0;   // (rpm)
		current_speed_R = 0;   // (rpm)
		error1 = 0;
		error2 = 0;
		integral1 = 0;
		integral2 = 0;
		pid_output_L = 0;
		pid_output_R = 0;
		pwm_duty_L = MIN_DUTY;
		pwm_duty_R = MIN_DUTY;
		pwm_duty_L1 = 0;
		pwm_duty_R1 = 0;
		TIM1->CCR1 = 0;
		TIM8->CCR1 = 0;
	}
	if(mode_start == true)
	{
		mode_cnt++;
		if(mode_cnt >= 500)
		{
			mode_cnt = 0;
			mode_new = 1;
		}
	}
	else
	{
		mode_cnt = 0;
		mode_new = 0;
	}
	if(target_speed_L < -70 || target_speed_R < -70)
	{
		mode_start = false;
	}
	if (pwm_duty_L1 >= MAX_DUTY)
		mode_start = false;
	if (pwm_duty_R1 >= MAX_DUTY)
		mode_start = false;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)    // Button interrupt
{
	if(GPIO_Pin == GPIO_PIN_1)
	{
		if(mode == 1)
		{//mode_start == true){
			mode = 0;//mode_start = false;
		}
		else if(mode == 0)
		{
			mode = 1;
		}
	}
}

uint32_t battery_check(uint32_t A)
{
	float Volt = 0;
	uint32_t battery = 0;
	Volt = A*0.134f-348.484f;//A*0.1342f-348.4848f;
	battery = (uint32_t)Volt;
	battery_count++;
	return battery;
}

uint32_t battery_sum(uint32_t B)
{
	int32_t battery1 = 0;
	battery_sum1 += B;
	battery1 = battery_sum1/battery_count;

	if(battery_sum1 > 1000000000){
		battery_sum1 = 0;
		battery_count = 0;
	}

	return battery1;
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
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM8_Init();
	MX_TIM9_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim9);
	HAL_ADC_Start_DMA(&hadc1, &adcVal[0], 1);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);

	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);

	setup();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		loop();
		batPer_cur = battery_check(adcVal[0]);
		batPer = battery_sum(batPer_cur);
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
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

	ADC_ChannelConfTypeDef sConfig = {0};

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
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

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
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 8999;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void)
{

	/* USER CODE BEGIN TIM8_Init 0 */

	/* USER CODE END TIM8_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM8_Init 1 */

	/* USER CODE END TIM8_Init 1 */
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 0;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 8999;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */
	HAL_TIM_MspPostInit(&htim8);

}

/**
 * @brief TIM9 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM9_Init(void)
{

	/* USER CODE BEGIN TIM9_Init 0 */

	/* USER CODE END TIM9_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};

	/* USER CODE BEGIN TIM9_Init 1 */

	/* USER CODE END TIM9_Init 1 */
	htim9.Instance = TIM9;
	htim9.Init.Prescaler = 179;
	htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim9.Init.Period = 9999;
	htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM9_Init 2 */

	/* USER CODE END TIM9_Init 2 */

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
	huart2.Init.BaudRate = 57600;
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
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	/* DMA1_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_SET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA1 */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB12 PB13 */
	GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
