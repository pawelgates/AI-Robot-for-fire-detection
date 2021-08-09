/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define usTIM TIM5
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart4;
USART_HandleTypeDef husart3;
DMA_HandleTypeDef hdma_uart4_rx;

/* USER CODE BEGIN PV */
uint8_t icFlag = 0;
uint8_t captureIdx=0;
uint32_t edge1Time=0, edge2Time=0;
char uartBuf[100];
uint8_t Rx_data[10];
uint8_t Rx_data2[4]={0,0,0,0};
uint8_t Rx_data2_cpy[4]={0,0,0,0};
float distance;
const float speedOfSound = 0.0343/2;
uint8_t IR_state[6]={0,0,0,0,0,0};
uint8_t cam_flg=1;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_Init(void);
static void MX_TIM5_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
void init_motors(void);
void usDelay(uint32_t uSec);
void forward_turn(uint32_t right_perc,uint32_t left_perc);
void rot_right(int32_t right_perc,uint32_t left_perc);
void forward(uint32_t speed_prec);
void motor_acc(uint32_t top_speed_prec);
void motor_acc_stop(uint32_t current_speed_perc);
float check_us_dis(void);
uint16_t ir_dir(void);
uint32_t cam_dir(void);
uint32_t cam_precision(void);
float dist_precision(void);
void fire_wall(void);
void fan_actv(void);
void backward(int32_t left_perc,uint32_t right_perc);
uint16_t fire_home(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t ir_dir(void)
{

	if(( !IR_state[0] && IR_state[1] && IR_state[2] && !IR_state[3] && !IR_state[4] ) || ( !IR_state[0] && IR_state[1] && IR_state[2] && IR_state[3] && !IR_state[4] ) || (IR_state[0] && IR_state[1] && IR_state[2] && IR_state[3] && IR_state[4]) || (!IR_state[0] && !IR_state[1] && IR_state[2] && !IR_state[3] && !IR_state[4]) )
	{
		return 2;//center
	}
	else
	{
		if(( !IR_state[0] && IR_state[2] && IR_state[3] && IR_state[4] ) || ( !IR_state[0] && !IR_state[1] && IR_state[4] ) || (!IR_state[0] && !IR_state[1] && IR_state[3]) || (!IR_state[0] && IR_state[3] && IR_state[4]))
		{
			return 0;//left
		}
		else
		{
			if(( IR_state[1] && IR_state[3] && IR_state[4] ) || ( IR_state[0] && !IR_state[3] && !IR_state[4] ) || (IR_state[0] && IR_state[1] && !IR_state[3]) || (IR_state[0] && IR_state[1] && !IR_state[4]))
			{
				return 1;//right
			}
			else
			{

				return 3;
			}
		}
	}
}


void init_motors(void)
{
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	HAL_GPIO_WritePin(L_motor_Len_GPIO_Port,L_motor_Len_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(L_motor_Ren_GPIO_Port,L_motor_Ren_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(L_motor_Lis_GPIO_Port,L_motor_Lis_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(L_motor_Ris_GPIO_Port,L_motor_Ris_Pin,GPIO_PIN_RESET);

	HAL_GPIO_WritePin(R_motor_Len_GPIO_Port,R_motor_Len_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(R_motor_Ren_GPIO_Port,R_motor_Ren_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(R_motor_Lis_GPIO_Port,R_motor_Lis_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R_motor_Ris_GPIO_Port,R_motor_Ris_Pin,GPIO_PIN_RESET);
}

void rot_right(int32_t left_perc,uint32_t right_perc)
{
	uint32_t left=0;
	uint32_t right=0;
	left=left_perc*10;
	right=right_perc*10;

	htim3.Instance->CCR1 = right;
	htim3.Instance->CCR2 = 0;
	htim3.Instance->CCR3 = 0;
	htim3.Instance->CCR4 = left;
}

void backward(int32_t left_perc,uint32_t right_perc)
{
	uint32_t left=0;
	uint32_t right=0;
	left=left_perc*10;
	right=right_perc*10;

	htim3.Instance->CCR1 = 0;
	htim3.Instance->CCR2 = right;
	htim3.Instance->CCR3 = 0;
	htim3.Instance->CCR4 = left;
}

void rot_left(int32_t left_perc,uint32_t right_perc)
{
	uint32_t left=0;
	uint32_t right=0;
	left=left_perc*10;
	right=right_perc*10;

	htim3.Instance->CCR1 = 0;
	htim3.Instance->CCR2 = right;
	htim3.Instance->CCR3 = left;
	htim3.Instance->CCR4 = 0;
}

void forward_turn(uint32_t left_perc,uint32_t right_perc)
{
	uint32_t left=0;
	uint32_t right=0;
	left=left_perc*10;
	right=right_perc*10;

	htim3.Instance->CCR1 = right;
	htim3.Instance->CCR2 = 0;
	htim3.Instance->CCR3 = left;
	htim3.Instance->CCR4 = 0;
}
void forward(uint32_t speed_prec)
{
	uint32_t speed=0;
	speed=speed_prec*10;

	htim3.Instance->CCR1 = speed;
	htim3.Instance->CCR2 = 0;
	htim3.Instance->CCR3 = speed;
	htim3.Instance->CCR4 = 0;

}

void motor_acc(uint32_t top_speed_prec)
{
	forward(top_speed_prec/16);
	HAL_Delay(200);
	forward(top_speed_prec/8);
	HAL_Delay(200);
	forward(top_speed_prec/4);
	HAL_Delay(200);
	forward(top_speed_prec/2);
	HAL_Delay(200);
	forward(top_speed_prec);
}

void motor_acc_stop(uint32_t current_speed_perc)
{
	forward(current_speed_perc);
	HAL_Delay(200);
	forward(current_speed_perc/2);
	HAL_Delay(200);
	forward(current_speed_perc/4);
	HAL_Delay(200);
	forward(current_speed_perc/8);
	HAL_Delay(200);
	forward(current_speed_perc/16);
	HAL_Delay(200);
	forward(0);
	/*
	for(uint32_t i=current_speed;i>=0;i--)
	{
		forward(i);
		HAL_Delay(30);
	}
	//forward(0);*/
}

uint32_t cam_dir(void)
{
	uint32_t i;
	uint32_t check=1,active=1;
	uint32_t x=0;

	for(i=0;i<=4;i++)
		Rx_data2_cpy[i]=Rx_data2[i];
	for(i=0;i<=4;i++)
	{
		if(Rx_data2_cpy[i]<0x29 || Rx_data2_cpy[i]>0x40)
		{
			check=0;
			if(Rx_data2_cpy[i]==120)
				active=0;
		}
	}

	if(check==1)
	{
		x = (Rx_data2_cpy[0]-0x30)*1000+(Rx_data2_cpy[1]-0x30)*100+(Rx_data2_cpy[2]-0x30)*10+(Rx_data2_cpy[3]-0x30);
		return x;
	}
	else
	{
		if(active==0)
			return 55555;
		return check;
	}
}
float check_us_dis(void)
{


	//Set TRIG to LOW for few uSec
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
	usDelay(3);

	//*** START Ultrasonic measure routine ***//
	//1. Output 10 usec TRIG
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
	usDelay(10);
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
	//HAL_UART_Receive(&huart4, Rx_data2, 6, 1000);
	//2. ECHO signal pulse width

	//Start IC timer
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
	//Wait for IC flag
	uint32_t startTick = HAL_GetTick();
	do
	{
		if(icFlag) break;
	}while((HAL_GetTick() - startTick) < 500);  //500ms

	icFlag = 0;
	HAL_TIM_IC_Stop_IT(&htim4, TIM_CHANNEL_1);

	//Calculate distance in cm
	if(edge2Time > edge1Time)
	{
		distance = ((edge2Time - edge1Time) + 0.0f)*speedOfSound;
	}
	else
	{
		distance = 30.0f;
	}
	return distance;
}

float dist_precision(void)
{
	float prec=0;
	uint16_t i;


	for(i=0;i<=4;i++)
	{
		prec=prec+check_us_dis();
		HAL_Delay(5);
	}
	return prec/5;

}

uint32_t cam_precision(void)
{
	uint32_t prec=0;
	uint16_t i;


	for(i=0;i<=4;i++)
	{
		prec=prec+cam_dir();
		HAL_Delay(5);
	}
	return prec/5;
}


void fire_wall(void)
{
	switch(ir_dir())
	{
	case 0://left
		rot_left(20, 20);
		HAL_Delay(800);
		forward_turn(0,0);
		//TBD fire extinguish
		fan_actv();
		while(1);
		break;
	case 1://right
		rot_right(20,20);
		HAL_Delay(800);
		forward_turn(0,0);
		//TBD fire extinguish
		fan_actv();
		while(1);
		break;
	case 2://front
		forward_turn(0,0);
		//TBD fire extinguish
		fan_actv();
		while(1);
		break;
	default:
		backward(50, 50);
		HAL_Delay(2000);
		rot_left(40, 40);
		break;
	}

}
uint16_t fire_home(void)
{
	while(ir_dir()<3 || dist_precision()>20)
	{
		switch(ir_dir())
		{
		case 0://left
			forward_turn(50,25);
			break;
		case 1://right
			forward_turn(25,50);
			break;
		case 2://center
			forward_turn(40,40);
			break;
		default: return 0;
		break;
		}
		if(dist_precision()<20 && ir_dir()<3)
			fire_wall();
	}

	return 0;
}

void fan_actv(void)
{
	HAL_GPIO_WritePin(fan_GPIO_Port, fan_Pin, GPIO_PIN_SET);
	HAL_Delay(1500);
	HAL_GPIO_WritePin(fan_GPIO_Port, fan_Pin, GPIO_PIN_RESET);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_DMA(&huart4, Rx_data2, 4);
	cam_flg=1;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */




	//uint16_t temp=30;
	float dis=0;
	uint16_t close_fire_dir=3;
	uint32_t far_fire_dir=0;




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
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_USART3_Init();
	MX_TIM5_Init();
	MX_UART4_Init();
	/* USER CODE BEGIN 2 */
	init_motors();
	HAL_UART_Receive_DMA(&huart4, Rx_data2, 4);



	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		dis=dist_precision();
		if(dis<20)
		{
			if(ir_dir()<3)
			{
				fire_wall();
			}
			else
			{
				HAL_Delay(2000);
				if(ir_dir()<3)
					fire_wall();
				rot_right(40,40);
				HAL_Delay(1000);//180deg
				motor_acc(50);
			}
		}
		else
		{
			far_fire_dir=cam_dir();
			switch(far_fire_dir)
			{
			case 0://error
				break;
			case 55555://active but no fire
				while(cam_dir()==55555)
				{
					rot_right(20,20);
					if(dist_precision()<20)
					{
						rot_left(40,40);
						HAL_Delay(1500);
						forward_turn(40,40);
						HAL_Delay(2000);
						rot_right(20,20);
					}

				}
				break;
			default:
				forward_turn(0,0);
				if(far_fire_dir<2000 && far_fire_dir>850)//right
				{
					while(dist_precision()>20 && ir_dir()>3 )
					{
						forward_turn(20,15);
						HAL_Delay(5000);
					}
					if(dis<20)
					{
						close_fire_dir=ir_dir();
						if(close_fire_dir<3)
						{
							fire_wall();
						}
						else
						{
							rot_right(20,20);
							HAL_Delay(1000);//180deg
							motor_acc(50);
						}
					}
					if(ir_dir()<2)
					{
						fire_wall();
					}
				}
				else
				{
					if(far_fire_dir<500)//left
					{
						while(dist_precision()>20 && ir_dir()>3 )
						{
							forward_turn(15,20);
							HAL_Delay(500);
						}
						if(dis<20)
						{
							close_fire_dir=ir_dir();
							if(close_fire_dir<3)
							{
								fire_wall();
							}
							else
							{
								rot_right(40,40);
								HAL_Delay(1000);//180deg
								motor_acc(50);
							}
						}
					}
					else//center
					{
						do
						{
							close_fire_dir=ir_dir();
							forward_turn(20,20);
							HAL_Delay(200);
						}
						while(dist_precision()>20 && close_fire_dir>2 );


						if(dis<20 )
						{
							close_fire_dir=ir_dir();
							if(close_fire_dir<3)
							{
								fire_wall();
							}
							else
							{
								rot_right(40,40);
								HAL_Delay(1000);//180deg
								motor_acc(50);
							}
						}
					}
				}
			}
			if(ir_dir()<3)
			{
				forward_turn(0,0);
				fire_home();
			}
			rot_right(20,20);
			HAL_Delay(1000);
		}






		//Print to UART terminal for debugging
		//   sprintf(uartBuf, "dir  = %d \n\n", temp);//sprintf(uartBuf, "Distance (cm)  = %.1f\r\n", distance);
		//HAL_USART_Transmit(&husart3, (uint8_t *)uartBuf, strlen(uartBuf), 6);

	}




	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */

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
	RCC_OscInitStruct.PLL.PLLN = 84;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		Error_Handler();
	}
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

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 84;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1000;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

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

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_IC_InitTypeDef sConfigIC = {0};

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 84-1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 1000000;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 4;
	if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void)
{

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 84-1;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 0;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */

}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void)
{

	/* USER CODE BEGIN UART4_Init 0 */

	/* USER CODE END UART4_Init 0 */

	/* USER CODE BEGIN UART4_Init 1 */

	/* USER CODE END UART4_Init 1 */
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN UART4_Init 2 */

	/* USER CODE END UART4_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	husart3.Instance = USART3;
	husart3.Init.BaudRate = 115200;
	husart3.Init.WordLength = USART_WORDLENGTH_8B;
	husart3.Init.StopBits = USART_STOPBITS_1;
	husart3.Init.Parity = USART_PARITY_NONE;
	husart3.Init.Mode = USART_MODE_TX_RX;
	husart3.Init.CLKPolarity = USART_POLARITY_LOW;
	husart3.Init.CLKPhase = USART_PHASE_1EDGE;
	husart3.Init.CLKLastBit = USART_LASTBIT_DISABLE;
	if (HAL_USART_Init(&husart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

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
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, TRIG_Pin|L_motor_Len_Pin|L_motor_Ren_Pin|L_motor_Lis_Pin
			|L_motor_Ris_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, R_motor_Ren_Pin|R_motor_Ris_Pin|R_motor_Len_Pin|R_motor_Lis_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(fan_GPIO_Port, fan_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : IR1_Pin IR2_Pin IR3_Pin IR4_Pin
                           IR5_Pin */
	GPIO_InitStruct.Pin = IR1_Pin|IR2_Pin|IR3_Pin|IR4_Pin
			|IR5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : TRIG_Pin */
	GPIO_InitStruct.Pin = TRIG_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : L_motor_Len_Pin L_motor_Ren_Pin L_motor_Lis_Pin L_motor_Ris_Pin */
	GPIO_InitStruct.Pin = L_motor_Len_Pin|L_motor_Ren_Pin|L_motor_Lis_Pin|L_motor_Ris_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : R_motor_Ren_Pin R_motor_Ris_Pin R_motor_Len_Pin R_motor_Lis_Pin */
	GPIO_InitStruct.Pin = R_motor_Ren_Pin|R_motor_Ris_Pin|R_motor_Len_Pin|R_motor_Lis_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : fan_Pin */
	GPIO_InitStruct.Pin = fan_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(fan_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */
void usDelay(uint32_t uSec)
{
	if(uSec < 2) uSec = 2;
	usTIM->ARR = uSec - 1; 	/*sets the value in the auto-reload register*/
	usTIM->EGR = 1; 			/*Re-initialises the timer*/
	usTIM->SR &= ~1; 		//Resets the flag
	usTIM->CR1 |= 1; 		//Enables the counter
	while((usTIM->SR&0x0001) != 1);
	usTIM->SR &= ~(0x0001);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

	if(captureIdx == 0) //Fisrt edge
	{
		edge1Time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); //__HAL_TIM_GetCounter(&htim3);//

		captureIdx = 1;
	}
	else if(captureIdx == 1) //Second edge
	{
		edge2Time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		captureIdx = 0;
		icFlag = 1;
	}

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	switch(GPIO_Pin)
	{
	case IR1_Pin  : if(HAL_GPIO_ReadPin(GPIOC,IR1_Pin))
		IR_state[0]=1;
	else
		IR_state[0]=0;
	break;
	case IR2_Pin  :  if(HAL_GPIO_ReadPin(GPIOC,IR2_Pin))
		IR_state[1]=1;
	else
		IR_state[1]=0;
	break;
	case IR3_Pin  :  if(HAL_GPIO_ReadPin(GPIOC,IR3_Pin))
		IR_state[2]=1;
	else
		IR_state[2]=0;
	break;
	case IR4_Pin  :  if(HAL_GPIO_ReadPin(GPIOC,IR4_Pin))
		IR_state[3]=1;
	else
		IR_state[3]=0;
	break;
	case IR5_Pin  : 	if(HAL_GPIO_ReadPin(GPIOC,IR5_Pin))
		IR_state[4]=1;
	else
		IR_state[4]=0;
	break;
	default : __NOP();
	break;
	}

}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
