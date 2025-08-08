/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
int count = 0;
int party = 0;
uint8_t receive = 0;
uint8_t transmit = 97;
uint8_t init = 0;
uint8_t songNum = 0;

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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct Song{
	int bpm; // should be between 50 and 150
	int duration; //in seconds
	int genre;
};
void forward(int ccr){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
	TIM4->CCR1 = ccr;
	TIM3->CCR2 = ccr;
}
void backward(float seconds, int ccr){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
	TIM4->CCR1 = ccr;
	TIM3->CCR2 = ccr;
}
void right(int ccr){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
	TIM4->CCR1 = ccr;
	TIM3->CCR2 = 0;
}
void left(int ccr){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
	TIM4->CCR1 = 0;
	TIM3->CCR2 = ccr;
}
int right_sensor(GPIO_InitTypeDef GPIO_InitStruct){
	  GPIO_InitStruct.Pin = GPIO_PIN_1;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
	  HAL_Delay(5);
	  GPIO_InitStruct.Pin = GPIO_PIN_1;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
	  TIM5->CNT = 0;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	  HAL_Delay(10);
	  return count;
}
int left_sensor(GPIO_InitTypeDef GPIO_InitStruct){
	  GPIO_InitStruct.Pin = GPIO_PIN_10;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
	  HAL_Delay(5);
	  GPIO_InitStruct.Pin = GPIO_PIN_10;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	  TIM2->CNT = 0;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	  HAL_Delay(10);
	  return count;
}
void stop(){
	TIM4->CCR1 = 0;
	TIM3->CCR2 = 0;
	TIM1->CCR3 = 0;
}
void dance(int ccr){
	TIM1->CCR3 = ccr;
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
  MX_USART6_UART_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM9_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

 int ccr = 0;
 int seconds = 0;
 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
 struct Song songs[13];
 songs[0].bpm = 0;
 songs[0].duration = 0;
 songs[0].genre = 0;
 //song 1: one kiss
 songs[1].bpm = 124;
 songs[1].duration = 215;
 songs[1].genre = 3;
 //somg2: both of us
 songs[2].bpm = 124;
 songs[2].duration = 352;
 songs[2].genre = 4;
 //song 3: romantika
 songs[3].bpm = 160;
 songs[3].duration = 250;
 songs[3].genre = 4;
 //song 4: birds of a feather
 songs[4].bpm = 104;
 songs[4].duration = 230;
 songs[4].genre = 3;
 //song 5: new light
 songs[5].bpm = 124;
 songs[5].duration = 217;
 songs[5].genre = 3;
 //song 6: california love
 songs[6].bpm = 92;
 songs[6].duration = 285;
 songs[6].genre = 1;
 //song 7: I had some help
 songs[7].bpm = 128;
 songs[7].duration = 201;
 songs[7].genre = 2;
 //song 8: Future song
 songs[8].bpm = 80;
 songs[8].duration = 148;
 songs[8].genre = 1;
 //song 9: nun major
 songs[9].bpm = 98;
 songs[9].duration = 152;
 songs[9].genre = 1;
 //song 10: bullets
 songs[10].bpm = 125;
 songs[10].duration = 299;
 songs[10].genre = 4;
 //song 11: jolene
 songs[11].bpm = 111;
 songs[11].duration = 161;
 songs[11].genre = 2;
 //song 12: all your'n
 songs[12].bpm = 78;
 songs[12].duration = 218;
 songs[12].genre = 2;


 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
 HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
 HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
 GPIO_InitTypeDef GPIO_InitStruct = {0};
 TIM2->CCER |= (0b1 << 9);
 TIM2->CCER &= ~(0b1 << 11);
 TIM5->CCER |= (0b1 << 5);
 TIM5->CCER &= ~(0b1 << 7);
 int rightS = 0;
 int leftS = 0;
 int genre = 0;
 int find_shape = 0;
 int intersection = 0;

 HAL_UART_Receive_IT(&huart6, &receive, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 while (1)
 {
	  //state: init
	 receive = 0;
	 transmit = 97;
	 init = 1;
	  while(init == 1){

	  }

	  //setup
	  ccr = 15;
	  int length = songs[songNum].duration;
 	  TIM1->ARR = 5000 * 60 / songs[songNum].bpm;
	  dance(1000);
	  genre = songs[songNum].genre;

	  __HAL_TIM_SET_COUNTER(&htim9, 0);
	  HAL_TIM_Base_Start(&htim9);
	  int tim9_time = 0;

	  //state: find_shape part 1
	  find_shape = 1;
	  intersection = 0;
	  while(find_shape == 1){
		  forward(ccr);
		  rightS = right_sensor(GPIO_InitStruct);
		  leftS = left_sensor(GPIO_InitStruct);
		  //test a
		  transmit = 97;
		  HAL_UART_Transmit(&huart6, &transmit, 1, HAL_MAX_DELAY);
		  //test a
		  if(rightS > 530 && leftS > 830) {
			  if((genre == 1 || genre == 2) && intersection == 0){
				  find_shape = 0;
				  break;
			  }
			  else if(intersection == 1){
				  //test c
				  transmit = 99;
				  HAL_UART_Transmit(&huart6, &transmit, 1, HAL_MAX_DELAY);
				  //test c
				  find_shape = 0;
				  break;
			  }
			  if(genre == 3 || genre == 4){
				  while(rightS > 530 && leftS > 830){
					  forward(ccr);
					  rightS = right_sensor(GPIO_InitStruct);
					  leftS = left_sensor(GPIO_InitStruct);
					  //test b
					  transmit = 98;
					  HAL_UART_Transmit(&huart6, &transmit, 1, HAL_MAX_DELAY);
					  //test b
				  }
			  }
			  intersection++;
		  }
		  else if(!(rightS > 530 && leftS > 830)){
			  while(rightS > 530){
				  leftS = left_sensor(GPIO_InitStruct);
				  rightS = right_sensor(GPIO_InitStruct);
				  if(rightS > 530 && leftS > 830){
					  break;
				  }
				  right(ccr + 25);
				  //test d
				  transmit = 100;
				  HAL_UART_Transmit(&huart6, &transmit, 1, HAL_MAX_DELAY);
				  //test d
			  }
			  while(leftS > 830){
				  leftS = left_sensor(GPIO_InitStruct);
				  rightS = right_sensor(GPIO_InitStruct);
				  if(rightS > 530 && leftS > 830){
					  break;
				  }
				  left(ccr + 25);
				  //test e
				  transmit = 101;
				  HAL_UART_Transmit(&huart6, &transmit, 1, HAL_MAX_DELAY);
				  //test e
			  }
		  }
	  }

 	 if(genre == 1 || genre == 3){
		  while(leftS > 830){
			  //test t
			  transmit = 116;
			  HAL_UART_Transmit(&huart6, &transmit, 1, HAL_MAX_DELAY);
			  //test t
			  left(ccr + 25);
			  leftS = left_sensor(GPIO_InitStruct);
		  }
 	 }
 	 else{
		  while(rightS > 530){
			  //test f
			  transmit = 102;
			  HAL_UART_Transmit(&huart6, &transmit, 1, HAL_MAX_DELAY);
			  //test f
			  right(ccr + 25);
			  rightS = right_sensor(GPIO_InitStruct);
		  }
 	 }

 	 //state: find_shape part 2
	  find_shape = 1;
	  while(find_shape == 1){
		  //test g
		  transmit = 103;
		  HAL_UART_Transmit(&huart6, &transmit, 1, HAL_MAX_DELAY);
		  //test g
		  forward(ccr);
		  rightS = right_sensor(GPIO_InitStruct);
		  leftS = left_sensor(GPIO_InitStruct);
		  if(rightS > 530 && leftS > 830) {
			  //test h
			  transmit = 104;
			  HAL_UART_Transmit(&huart6, &transmit, 1, HAL_MAX_DELAY);
			  //test h
			  find_shape = 0;
			  break;
		  }
		  else if(!(rightS > 530 && leftS > 830)){
			  while(rightS > 530){
				  leftS = left_sensor(GPIO_InitStruct);
				  rightS = right_sensor(GPIO_InitStruct);
				  if(rightS > 530 && leftS > 830) {
					  //test j
					  transmit = 106;
					  HAL_UART_Transmit(&huart6, &transmit, 1, HAL_MAX_DELAY);
					  //test j
					  find_shape = 0;
					  break;
				  }
				  right(ccr + 25);
				  //test i
				  transmit = 105;
				  HAL_UART_Transmit(&huart6, &transmit, 1, HAL_MAX_DELAY);
				  //test i
			  }
			  while(leftS > 830){
				  leftS = left_sensor(GPIO_InitStruct);
				  rightS = right_sensor(GPIO_InitStruct);
				  if(rightS > 530 && leftS > 830){
					  //test l
					  transmit = 108;
					  HAL_UART_Transmit(&huart6, &transmit, 1, HAL_MAX_DELAY);
					  //test l
					  find_shape = 0;
					  break;
				  }
				  left(ccr + 25);
				  //test k
				  transmit = 107;
				  HAL_UART_Transmit(&huart6, &transmit, 1, HAL_MAX_DELAY);
				  //test k
			  }
	  	  }
	  }
	  while(rightS > 530){
		  //test m
		  transmit = 109;
		  HAL_UART_Transmit(&huart6, &transmit, 1, HAL_MAX_DELAY);
		  //test m
		  right(ccr + 25);
		  rightS = right_sensor(GPIO_InitStruct);
	  }

	  //state: dance_party for genre 1 and 3
	  party = 1;
	  while(party == 1 && (tim9_time < length * 10)){
		  //test n
		  transmit = 110;
		  HAL_UART_Transmit(&huart6, &transmit, 1, HAL_MAX_DELAY);
		  //test n
		  forward(ccr);
		  rightS = right_sensor(GPIO_InitStruct);
		  leftS = left_sensor(GPIO_InitStruct);
		  while(rightS > 600 && leftS > 900) {
			  //test o
			  transmit = 111;
			  HAL_UART_Transmit(&huart6, &transmit, 1, HAL_MAX_DELAY);
			  //test o
			  forward(ccr);
			  rightS = right_sensor(GPIO_InitStruct);
			  leftS = left_sensor(GPIO_InitStruct);
		  }
		  if(!(rightS > 600 && leftS > 900)){
			  while(rightS > 530){
				  leftS = left_sensor(GPIO_InitStruct);
				  rightS = right_sensor(GPIO_InitStruct);
				  if(leftS > 900){
					  //test q
					  transmit = 113;
					  HAL_UART_Transmit(&huart6, &transmit, 1, HAL_MAX_DELAY);
					  //test q
					  break;
				  }
				  right(ccr + 25);
				  //test p
				  transmit = 112;
				  HAL_UART_Transmit(&huart6, &transmit, 1, HAL_MAX_DELAY);
				  //test p
			  }
			  while(leftS > 830){
				  leftS = left_sensor(GPIO_InitStruct);
				  rightS = right_sensor(GPIO_InitStruct);
				  if(rightS > 600){
					  //test s
					  transmit = 115;
					  HAL_UART_Transmit(&huart6, &transmit, 1, HAL_MAX_DELAY);
					  //test s
					  break;
				  }
				  left(ccr + 25);
			  }
		  }
		  tim9_time = __HAL_TIM_GET_COUNTER(&htim9);
	  }

	  //return part 1
	  find_shape = 1;
	  while(find_shape == 1){
		  forward(ccr);
		  rightS = right_sensor(GPIO_InitStruct);
		  leftS = left_sensor(GPIO_InitStruct);
		  if(rightS > 530 && leftS > 830) {
			  find_shape = 0;
			  break;
		  }
		  else if(!(rightS > 530 && leftS > 830)){
			  while(rightS > 530){
				  leftS = left_sensor(GPIO_InitStruct);
				  rightS = right_sensor(GPIO_InitStruct);
				  if(rightS > 530 && leftS > 830) {
					  find_shape = 0;
					  break;
				  }
				  right(ccr + 25);
			  }
			  while(leftS > 830){
				  leftS = left_sensor(GPIO_InitStruct);
				  rightS = right_sensor(GPIO_InitStruct);
				  if(rightS > 530 && leftS > 830){
					  find_shape = 0;
					  break;
				  }
				  left(ccr + 25);
			  }
	  	  }
	  }

	  while(rightS > 530){
			  right(ccr + 25);
			  rightS = right_sensor(GPIO_InitStruct);
	  }

	  //return part 2
	  find_shape = 1;
	  while(find_shape == 1){
		  forward(ccr);
		  rightS = right_sensor(GPIO_InitStruct);
		  leftS = left_sensor(GPIO_InitStruct);
		  if(rightS > 530 && leftS > 830) {
			  find_shape = 0;
			  break;
		  }
		  else if(!(rightS > 530 && leftS > 830)){
			  while(rightS > 530){
				  leftS = left_sensor(GPIO_InitStruct);
				  rightS = right_sensor(GPIO_InitStruct);
				  if(rightS > 530 && leftS > 830) {
					  find_shape = 0;
					  break;
				  }
				  right(ccr + 25);
			  }
			  while(leftS > 830){
				  leftS = left_sensor(GPIO_InitStruct);
				  rightS = right_sensor(GPIO_InitStruct);
				  if(rightS > 530 && leftS > 830){
					  find_shape = 0;
					  break;
				  }
				  left(ccr + 25);
			  }
		  }
	  }

	  if(genre == 2 || genre == 4){
		  while(leftS > 830){
			  left(ccr + 25);
			  leftS = left_sensor(GPIO_InitStruct);
		  }
	  }
	  else{
		  while(rightS > 530){
			  right(ccr + 25);
			  rightS = right_sensor(GPIO_InitStruct);
		  }
	  }


	  //return part 3
	  find_shape = 1;
	  intersection = 0;
	  while(find_shape == 1){
		  forward(ccr);
		  rightS = right_sensor(GPIO_InitStruct);
		  leftS = left_sensor(GPIO_InitStruct);
		  if(rightS > 530 && leftS > 830) {
			  if((genre == 1 || genre == 2) && intersection == 0){
				  find_shape = 0;
				  break;
			  }
			  else if(intersection == 1){
				  find_shape = 0;
				  break;
			  }
			  if(genre == 3 || genre == 4){
				  while(rightS > 530 && leftS > 830){
					  forward(ccr);
					  rightS = right_sensor(GPIO_InitStruct);
					  leftS = left_sensor(GPIO_InitStruct);
				  }
			  }
			  intersection++;
		  }
		  else if(!(rightS > 530 && leftS > 830)){
			  while(rightS > 530){
				  leftS = left_sensor(GPIO_InitStruct);
				  rightS = right_sensor(GPIO_InitStruct);
				  if(rightS > 530 && leftS > 830){
					  break;
				  }
				  right(ccr + 25);
			  }
			  while(leftS > 830){
				  leftS = left_sensor(GPIO_InitStruct);
				  rightS = right_sensor(GPIO_InitStruct);
				  if(rightS > 530 && leftS > 830){
					  break;
				  }
				  left(ccr + 25);
			  }
		  }
	  }
	  stop();


	  //white right sensor: ~450, white left sensor: 783
	  //black right sensor: ~1180, black left sensor: 2050
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 256;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV128;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim1.Init.Prescaler = 99;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */
  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 9;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */
  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
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
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */
  /* USER CODE END TIM5_Init 2 */

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
  htim9.Init.Prescaler = 49999;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65000;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */
  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */
  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */
  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 USART_RX_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 PA9 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_Receive_IT(&huart6, &receive, 1); //You need to toggle a breakpoint on this line!
  if(init == 1){
	  if(receive > 0 && receive < 13){
		  init = 0;
		  songNum = receive;
		  transmit = 97;
		  HAL_UART_Transmit(&huart6, &transmit, 1, HAL_MAX_DELAY);
	  }
  }
  else if(party == 1){
	  if(receive == 222){
		  party = 0;
		  transmit = 222;
		  HAL_UART_Transmit(&huart6, &transmit, 1, HAL_MAX_DELAY);
	  }
  }
}
// transmit receive songNum init
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

