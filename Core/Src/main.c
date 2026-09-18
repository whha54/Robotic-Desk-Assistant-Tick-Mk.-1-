/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"

#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "math.h" // for complementary filter math

#include "stdlib.h" //for random number gen

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum{
	INIT_STATE,
	EYES,
	POMO,
	BREATHE,
	FIDGET
}States;

typedef enum{
	COUNTDOWN,
	TIMER,
	BREAK
}PomoStates;

typedef enum{
	UP,
	DOWN
}CircleCount;

typedef struct{
	uint8_t x;
	uint8_t y;
}Circles;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BMI323_I2C_ADD (0x68 << 1) //device address of IMU - shift to the left 1 to allow space for read/write bit

#define ACC_DATA_X 0x03 //accel data on x-axis
#define ACC_DATA_Y 0x04 //accel data on y-axis
#define ACC_DATA_Z 0x05 // accdel data on z-axis
#define GYR_DATA_X 0x06 // gyro data on x-axis
#define GYR_DATA_Y 0x07 // gyro data on y-axis
#define GYR_DATA_Z 0x08 // gyro data on z_axis

//needed to configure any-motion advanced feature
#define FEATURE_IO2 0x12
#define FEATURE_IO_STATUS 0x14
#define FEATURE_CTRL 0x40
#define FEATURE_IO1 0x11
#define FEATURE_IO0 0x10 //write to this register to enable advanced feature

#define ACC_CONF 0x20 //enable accel
#define GYR_CONF 0x21 //enable gyro
#define IO_INT_CTRL 0x38 // enable int lines as active high push pull lines
#define INT_MAP2 0x3B // map data ready interrupt to int1
#define INT_CONF 0x39 // enable non-lathced mode (interrupt will turn off automatically)
#define INT_MAP1 0x3A // map int 2 to enable when any-motion is detected
#define INT_STATUS_INT1 0x0D // read this register to see if tap detection or data ready was activated
#define TAP_1 0x1E //tap modes
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
static States curState;
static PomoStates curPomoState = COUNTDOWN;
static CircleCount curfidstate = UP;

volatile int32_t seconds = 0; //seconds elapsed
float groll = 0.0f; // roll angle
float glinearX = 0.0f; // global variable for linearX
float glinearZ = 0.0f; // global variable for linearZ

volatile uint8_t dr_flag = 0; //data-ready imu interrupt.
volatile uint8_t motion_flag = 0;
volatile uint8_t tap_flag = 0; // tap flag
static int32_t pomocounter = 0;
static int32_t tapcounter = 0;

Circles circles[10];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
int16_t BMI323_Read(I2C_HandleTypeDef *hi2c, uint8_t reg_add);
void BMI323_Init(void);
float getRoll(void); // get roll angle of robot
void drawEye(void);
void blinkEye(void);
void centertext(char *str, int y, char *tot);
void pomocount(void);
void pomotimer(void);
void pomobreak(void);
void pomolongbreak(void);
void breathecircle(void);
void drawcircle(void);
void deletecircle();


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2); // start the 1-second timer
  ssd1306_Init(); //initialize display
  BMI323_Init();

  //get roll
  groll = getRoll();


  //clear pixels
  ssd1306_Fill(Black);
  ssd1306_UpdateScreen();

  //initiate screen
  curState = INIT_STATE;

  // just for init_state, use straight boundaries (no software hysteresis) to define rot_state
  if(groll <= 45 && groll >= -45){
    curState = EYES; //initialize as eyes if flat
    }
  else if(groll < -45 && groll >= -90){
    curState = BREATHE; //init as timer if approaching 90 degrees clockwise
    }
  else if(groll > 45 && groll < 90){
    curState = FIDGET; // init as game if approaching 90 degrees ccw
    }
  else{
    curState = POMO; // for all other possible values of roll, init as clock
    }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){

	  if(dr_flag == 1){
		  dr_flag = 0;
		  groll = getRoll();
	  }

	  switch(curState){
	  	  case EYES:

	  		  if(groll > 55 && groll < 135){
	  			  ssd1306_Fill(Black);
	  			  ssd1306_UpdateScreen();
	  			  curfidstate = UP;
	  			  curState = FIDGET;
	  			  break;
	  		  }
	  		  else if(groll < -55 && groll > -135){
	  			  ssd1306_Fill(Black);
	  			  ssd1306_UpdateScreen();
	  			  curState = BREATHE;
	  			  break;
	  		  }

	  		  static uint32_t lastblink = 0;
	  		  uint32_t blinktime = (rand() % 4001) + 5000; //random nuber between 5s and 9s

	  		  //display eyes
	  		  ssd1306_WriteCommand(0xA1);
	  		  ssd1306_WriteCommand(0xC8);

	  		  ssd1306_Fill(Black);
	  		  drawEye();

	  		  if(HAL_GetTick() - lastblink >= blinktime){
	  			  lastblink = HAL_GetTick();
	  			  blinktime = (rand() % 4001) + 1000;
	  			  ssd1306_Fill(Black);
	  			  blinkEye();
	  		  }


	  		  break;

	  	  case BREATHE:

	  		  if(groll > 155 || groll < -155){
	  			  ssd1306_Fill(Black);
	  			  ssd1306_UpdateScreen();
	  			  curState = POMO;
	  			  break;
	  		  }
	  		  else if(groll < 35 && groll > -35){
	  			  ssd1306_Fill(Black);
	  			  ssd1306_UpdateScreen();
	  			  curState = EYES;
	  			  break;
	  		  }

	  		  //display breathe
	  		  ssd1306_WriteCommand(0xA1);
	  		  ssd1306_WriteCommand(0xC8);

	  		  breathecircle();

	  		  break;

	  	  case POMO:

	  		  if(groll < -55 && groll > -135){
	  			  ssd1306_Fill(Black);
	  			  ssd1306_UpdateScreen();
	  			  curState = BREATHE;
	  			  break;
	  		  }
	  		  else if(groll > 55 && groll < 135){
	  			  ssd1306_Fill(Black);
	  			  ssd1306_UpdateScreen();
	  			  curfidstate = UP;
	  			  curState = FIDGET;
	  			  break;
	  		  }

	  		  //rotate 180 degrees
	  		  ssd1306_WriteCommand(0xA0);
	  		  ssd1306_WriteCommand(0xC0);

	  		  //display pomo

	  		  switch(curPomoState){

	  		  case COUNTDOWN:
	  			  ssd1306_Fill(Black);
	  			  pomocount();
	  			  ssd1306_UpdateScreen();
	  			  break;

	  		  case TIMER:
	  			  ssd1306_Fill(Black);
	  			  pomotimer();
	  			  ssd1306_UpdateScreen();
	  			  break;

	  		  case BREAK:
	  		  	  ssd1306_Fill(Black);

	  		  	  if(pomocounter == 4){
	  		  		  pomolongbreak();
	  		  		  ssd1306_UpdateScreen();
	  		  	  }
	  		  	  else{
	  		  		  pomobreak();
	  		  		  ssd1306_UpdateScreen();
	  		  	  }


	  		  }

	  		  break;

	  	  case FIDGET:

	  		  ssd1306_WriteCommand(0xA1);
	  		  ssd1306_WriteCommand(0xC8);

	  		  if(groll > -35 && groll < 35){
	  			  ssd1306_Fill(Black);
	  			  ssd1306_UpdateScreen();
	  			  tapcounter = 0;
	  			  curState = EYES;
	  			  break;
	  		  }
	  		  else if(groll < -155 || groll > 155){
	  			  ssd1306_Fill(Black);
	  			  ssd1306_UpdateScreen();
	  			  tapcounter = 0;
	  			  curState = POMO;
	  			  break;
	  		  }

	  		  if(tap_flag == 1){
	  			  tap_flag = 0;

	  			  switch(curfidstate){

	  			  case UP:
	  				  if(tapcounter < 10){
	  					  drawcircle();
	  				  }

	  				  if(tapcounter >= 10){
	  					  tapcounter = 9;
	  					  curfidstate = DOWN;
	  				  }

	  				  break;

	  			  case DOWN:
	  				  if(tapcounter >= 0){
	  					  deletecircle();
	  				  }
	  				  if(tapcounter < 0){
	  					  curfidstate = UP;
	  					  tapcounter = 0;
	  				  }
	  				  break;
	  			  }
	  		  }

	  		  break;
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
}
  /* USER CODE END 3 */


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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : INT2_Pin INT1_Pin VBUS_SENSE_Pin */
  GPIO_InitStruct.Pin = INT2_Pin|INT1_Pin|VBUS_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CHG_Pin */
  GPIO_InitStruct.Pin = CHG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CHG_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN){
	if(GPIO_PIN == INT1_Pin){

		uint16_t int_status = BMI323_Read(&hi2c2, INT_STATUS_INT1);

		if(int_status & 0x0100){
			tap_flag = 1;
		}

		if(int_status & 0x3000){
			dr_flag = 1;
		}
	}
	else if(GPIO_PIN == INT2_Pin){
		motion_flag = 1;
	}
	else if(GPIO_PIN == VBUS_SENSE_Pin){
		if(HAL_GPIO_ReadPin(GPIOA, VBUS_SENSE_Pin) == GPIO_PIN_SET){
			MX_USB_DEVICE_Init();
		}
		else{
			USB_DEVICE_DeInit();
		}
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){
		seconds += 1;
	}
}


//read function for bmi323. the imu will send out two dummy bytes before the real data, so we need to filter that out
int16_t BMI323_Read(I2C_HandleTypeDef *hi2c, uint8_t reg_add){
	uint8_t buffer[4] = {0}; //make bucket for read data to go into (2 real, 2 dummy)

	HAL_I2C_Mem_Read(hi2c, BMI323_I2C_ADD, reg_add, I2C_MEMADD_SIZE_8BIT, buffer, 4, 100); //read

	int16_t data = (int16_t)(buffer[3] << 8 | buffer[2]); //actual data is the binary in buffer[3] and buffer[2] or'ed together.

	return data;
}

// write all data necessary to activate BMI323
void BMI323_Init(void){

	uint16_t status = 0; //will be used when we poll io1 until it equals 0x0001
	uint8_t data[2] = {0}; // bucket for our write data. our data is 16 bits but i2c only sends out data 8 bits at time, so we need to split it up into 8 bit chunks that it can digest

	//advanced feature initiation - write to io2, then io_status, then feature_ctrl, then polling until io1 = 0x0001, then turning on any-motion

	//io2
	data[0] = 0x2C;
	data[1] = 0x01;
	HAL_I2C_Mem_Write(&hi2c2, BMI323_I2C_ADD, FEATURE_IO2, I2C_MEMADD_SIZE_8BIT, data, 2, 100);
	HAL_Delay(20);

	//io_status
	data[0] = 0x01;
	data[1] = 0x00;
	HAL_I2C_Mem_Write(&hi2c2, BMI323_I2C_ADD, FEATURE_IO_STATUS, I2C_MEMADD_SIZE_8BIT, data, 2, 100);
	HAL_Delay(20);

	//feature_ctrl
	data[0] = 0x01;
	data[1] = 0x00;
	HAL_I2C_Mem_Write(&hi2c2, BMI323_I2C_ADD, FEATURE_CTRL, I2C_MEMADD_SIZE_8BIT, data, 2, 100);
	HAL_Delay(20);

	//io1 polling
	uint32_t start = HAL_GetTick();

	do{
	  status = BMI323_Read(&hi2c2, FEATURE_IO1);
	  HAL_Delay(1);

	  if(HAL_GetTick() - start > 1000){
		  Error_Handler();
	  }

	}while((status & 0x000F) != 0x0001);


	//tap detection - single tap for now; and any-motion detection on all 3 axes
	data[0] = 0x38;
	data[1] = 0x10;
	HAL_I2C_Mem_Write(&hi2c2, BMI323_I2C_ADD, FEATURE_IO0, I2C_MEMADD_SIZE_8BIT, data, 2, 100);
	HAL_Delay(20);

	// configure accelerometer to normal mode with delay to prevent overload
	data[0] = 0x28;
	data[1] = 0x40;
	HAL_I2C_Mem_Write(&hi2c2, BMI323_I2C_ADD, ACC_CONF, I2C_MEMADD_SIZE_8BIT, data, 2, 100);
	HAL_Delay(20);

	// configure gyroscope to normal mode with delay to prevent overload
	data[0] = 0x48;
	data[1] = 0x40;

	HAL_I2C_Mem_Write(&hi2c2, BMI323_I2C_ADD, GYR_CONF, I2C_MEMADD_SIZE_8BIT, data, 2, 100);
	HAL_Delay(20);

	// enable interrupts as active high push-pull lines
	data[0] = 0x05;
	data[1] = 0x05;
	HAL_I2C_Mem_Write(&hi2c2, BMI323_I2C_ADD, IO_INT_CTRL, I2C_MEMADD_SIZE_8BIT, data, 2, 100);
	HAL_Delay(20);

	// write bmi323 to non-latch interrupt mode (interrupt will automatically clear)
	data[0] = 0x00;
	data[1] = 0x00;
	HAL_I2C_Mem_Write(&hi2c2, BMI323_I2C_ADD, INT_CONF, I2C_MEMADD_SIZE_8BIT, data, 2, 100);
	HAL_Delay(20);

	// map data ready interrupt (both accel data and gyr data) and tap detection to int1
	data[0] = 0x01;
	data[1] = 0x05;
	HAL_I2C_Mem_Write(&hi2c2, BMI323_I2C_ADD, INT_MAP2, I2C_MEMADD_SIZE_8BIT, data, 2, 100);
	HAL_Delay(20);

	// map any-motion detection to int2
	data[0] = 0x08;
	data[1] = 0x00;
	HAL_I2C_Mem_Write(&hi2c2, BMI323_I2C_ADD, INT_MAP1, I2C_MEMADD_SIZE_8BIT, data, 2, 100);
	HAL_Delay(20);

	/*
	//set tap mode
	data[0] = 0x36;
	data[1] = 0x00;
	HAL_I2C_Mem_Write(&hi2c2, BMI323_I2C_ADD, TAP_1, I2C_MEMADD_SIZE_8BIT, data, 2, 100);
	HAL_Delay(20);
	*/

}

float getRoll(void){
	static uint32_t oldTime = 0; //for dt
	static float oldRoll = 0; //for later usage in complementary filter

	uint32_t currentTime = HAL_GetTick(); // get current time for dt in ms calculation
	float dt = (currentTime - oldTime) / 1000.0f; // dt in seconds
	oldTime = currentTime; // update last time

	float a = (0.1) / (0.1 + dt); //dynamic a. 0.1 is a target time. we do it this way so that dt is taken into account because it might vary between individual measurements


	//usually we'd be reading acc_y and gyr_y instead of acc_x gyr_x, but not here because of the orientation of the board
	int16_t bin_accel_x = BMI323_Read(&hi2c2, ACC_DATA_X);
	int16_t bin_accel_z = BMI323_Read(&hi2c2, ACC_DATA_Z);
	int16_t bin_gyro_x = BMI323_Read(&hi2c2, GYR_DATA_X);

	float accel_x = bin_accel_x / 4096.0f; // get real accel by dividing by factor
	float accel_z = bin_accel_z / 4096.0f;
	float gyr_x = bin_gyro_x / 16.4f;

	float rawRoll = atan2f(accel_x, accel_z) * (57.2957795f); // raw roll angle, converting radians from atan2 to degrees

	//the computer freaks out at the threshold between 180 and -180, so we gotta fix that wraparound error.
	float gyr_pred = oldRoll + gyr_x * dt; //stemming from conventional complementary filter roll = a(oldRoll * gyr_x * dt) + (1-a)(rawRoll). we just take that first term without the a and call it our prediction value
	float error = rawRoll - gyr_pred; //calculating the error

	if(error >= 180.0f){ //if that error is massive above 180, we subtract 360, which represents the shortest way around a circle
		error -= 360.0f;
	}
	else if(error <= -180.0f){ //vice versa
		error += 360.0f;
	}

	float roll = gyr_pred + ((1 - a) * error); // we use algebra to turn a(oldRoll + gyr_x * dt) + (1-a)(rawRoll) into gyr_pred + (1-a)(error).
	oldRoll = roll; // update roll

	//keep in in [-180, 180]
	if(roll > 180.0f){
		roll = -360.0f + roll;
	}
	else if(roll < -180.0f){
		roll = 360.0f + roll;
	}

	return roll;

}

/* getAccel functions
accelerometer data = gravity + motion + noise
we want to isolate motion
low pass filter on raw accel data to get a rough estimate of gravity
subtract rough estimate of gravity from raw accel data to get motion + noise
filter out noise by thresholding in game function
*/

// eye animations
void drawEye(void){
ssd1306_FillRectangle(37, 19, 56, 38, White); //left eye
ssd1306_FillRectangle(73, 19, 92, 38, White); //right eye
ssd1306_UpdateScreen(); //display eyes
}

//we will use a for loop to decrease and increase the height of both eyes to give the impression of blinking
void blinkEye(){
 //for loop to decrease the height
for(int i = 0; i < 3; i++){

	ssd1306_FillRectangle(37, 19, 56, 38, Black);
	ssd1306_FillRectangle(73, 19, 92, 38, Black);

	ssd1306_FillRectangle(37, 19 + 6 * i, 56, 38, White); //left eye
	ssd1306_FillRectangle(73, 19 + 6 * i, 92, 38, White); //right eye
	ssd1306_UpdateScreen(); //display eyes
	HAL_Delay(1);
}

//for loop to increase the height
for(int i = 0; i < 3; i++){

	ssd1306_FillRectangle(37, 19, 56, 38, Black);
	ssd1306_FillRectangle(73, 19, 92, 38, Black);

	ssd1306_FillRectangle(37, 37 - 6 * i, 56, 38, White); //left eye
	ssd1306_FillRectangle(73, 37 - 6 * i, 92, 38, White); //right eye
	ssd1306_UpdateScreen(); //display eyes
	HAL_Delay(1);
}

}


void centertext(char *str, int y, char *tot){

	int width;

	if(strcmp(tot, "text") == 0){
		width = strlen(str) * 7;
	}
	else if(strcmp(tot, "title") == 0){
		width = strlen(str) * 11;
	}

	int x = (128 - width) / 2;

	ssd1306_SetCursor(x,y);

	if(strcmp(tot, "text") == 0){
		ssd1306_WriteString(str, Font_7x10, White);
	}
	else if(strcmp(tot, "title") == 0){
		ssd1306_WriteString(str, Font_11x18, White);
	}

}


// countdown for pomodoro timer to start to give users some time before the actual timer starts
void pomocount(void){

	int32_t elapsed = 5;
	seconds = 0;

	while(elapsed > -1){

		//keep measuring roll so that we can exit loop if the user wants to
		groll = getRoll();

		if(groll < -55 && groll > -135){
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen();
			pomocounter = 0;
			curPomoState = COUNTDOWN;
			curState = BREATHE;
			return;
		}
		else if(groll > 55 && groll < 135){
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen();
			curfidstate = UP;
			pomocounter = 0;
			curPomoState = COUNTDOWN;
			curState = FIDGET;
			return;
		}

		centertext("Pomo. Timer", 0, "title");
		centertext("Starting in...", 18, "text");

		char sec_str[100];

		snprintf(sec_str, sizeof(sec_str), "%li", elapsed);

		centertext(sec_str, 36, "title");

		ssd1306_UpdateScreen();

		elapsed = 5 - seconds;
	}

	curPomoState = TIMER;

}

//actual pomo timer
void pomotimer(void){
	int32_t elapsed = 1500;
	uint32_t minutes = 0;
	uint32_t sec = 0;

	seconds = 0;

	while(elapsed > -1){
		ssd1306_Fill(Black);

		groll = getRoll();

		if(groll < -55 && groll > -135){
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen();
			pomocounter = 0;
			curPomoState = COUNTDOWN;
			curState = BREATHE;
			return;
		}
		else if(groll > 55 && groll < 135){
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen();
			curfidstate = UP;
			pomocounter = 0;
			curPomoState = COUNTDOWN;
			curState = FIDGET;
			return;
		}

		minutes = elapsed / 60;
		sec = elapsed % 60;

		char time[100];


		if(sec / 10 == 0){
			if(minutes / 10 == 0){
				snprintf(time, sizeof(time), "0%lu:0%lu", minutes, sec);
			}
			else{
				snprintf(time, sizeof(time), "%lu:0%lu", minutes, sec);
			}
		}
		else{
			if(minutes / 10 == 0){
				snprintf(time, sizeof(time), "0%lu:%lu", minutes, sec);
			}
			else{
				snprintf(time, sizeof(time), "%lu:%lu", minutes, sec);
			}
		}

		centertext("Work Time", 0, "text");
		centertext(time, 23, "title");
		ssd1306_UpdateScreen();

		elapsed = 1500 - seconds;

	}

	seconds = 0;

	ssd1306_Fill(White);
	ssd1306_UpdateScreen();

	while(seconds <= 5){

		groll = getRoll();

		if(groll < -55 && groll > -135){
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen();
			pomocounter = 0;
			curPomoState = COUNTDOWN;
			curState = BREATHE;
			return;
		}
		else if(groll > 55 && groll < 135){
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen();
			curfidstate = UP;
			pomocounter = 0;
			curPomoState = COUNTDOWN;
			curState = FIDGET;
			return;
		}

		if(seconds % 2 == 1){
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen();
		}
		else if(seconds % 2 == 0){
			ssd1306_Fill(White);
			ssd1306_UpdateScreen();
		}
	}

	ssd1306_Fill(Black);
	ssd1306_UpdateScreen();

	elapsed = 5;
	seconds = 0;

	while(elapsed > -1){

		//keep measuring roll so that we can exit loop if the user wants to
		groll = getRoll();

		if(groll < -55 && groll > -135){
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen();
			pomocounter = 0;
			curPomoState = COUNTDOWN;
			curState = BREATHE;
			return;
		}
		else if(groll > 55 && groll < 135){
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen();
			curfidstate = UP;
			pomocounter = 0;
			curPomoState = COUNTDOWN;
			curState = FIDGET;
			return;
		}

		centertext("Break Time", 0, "title");
		centertext("Starting in...", 18, "text");

		char sec_str[100];

		snprintf(sec_str, sizeof(sec_str), "%li", elapsed);

		centertext(sec_str, 36, "title");

		ssd1306_UpdateScreen();

		elapsed = 5 - seconds;
		}

	curPomoState = BREAK;

	}

//break timer
void pomobreak(){
int32_t elapsed = 300;
uint32_t minutes = 0;
uint32_t sec = 0;

seconds = 0;

while(elapsed > -1){
	ssd1306_Fill(Black);

	groll = getRoll();

	if(groll < -55 && groll > -135){
		ssd1306_Fill(Black);
		ssd1306_UpdateScreen();
		pomocounter = 0;
		curPomoState = COUNTDOWN;
		curState = BREATHE;
		return;
	}
	else if(groll > 55 && groll < 135){
		ssd1306_Fill(Black);
		ssd1306_UpdateScreen();
		curfidstate = UP;
		pomocounter = 0;
		curPomoState = COUNTDOWN;
		curState = FIDGET;
		return;
	}

	minutes = elapsed / 60;
	sec = elapsed % 60;

	char time[100];


	if(sec / 10 == 0){
		if(minutes / 10 == 0){
			snprintf(time, sizeof(time), "0%lu:0%lu", minutes, sec);
		}
		else{
			snprintf(time, sizeof(time), "%lu:0%lu", minutes, sec);
		}
	}
	else{
		if(minutes / 10 == 0){
			snprintf(time, sizeof(time), "0%lu:%lu", minutes, sec);
		}
		else{
			snprintf(time, sizeof(time), "%lu:%lu", minutes, sec);
		}
	}

		centertext("Break Time", 0, "text");
		centertext(time, 23, "title");
		ssd1306_UpdateScreen();

		elapsed = 300 - seconds;

	}

seconds = 0;

ssd1306_Fill(White);
ssd1306_UpdateScreen();

while(seconds <= 5){

	groll = getRoll();

	if(groll < -55 && groll > -135){
		ssd1306_Fill(Black);
		ssd1306_UpdateScreen();
		pomocounter = 0;
		curPomoState = COUNTDOWN;
		curState = BREATHE;
		return;
	}
	else if(groll > 55 && groll < 135){
		ssd1306_Fill(Black);
		ssd1306_UpdateScreen();
		curfidstate = UP;
		pomocounter = 0;
		curPomoState = COUNTDOWN;
		curState = FIDGET;
		return;
	}

	if(seconds % 2 == 1){
		ssd1306_Fill(Black);
		ssd1306_UpdateScreen();
	}
	else if(seconds % 2 == 0){
		ssd1306_Fill(White);
		ssd1306_UpdateScreen();
	}
}

ssd1306_Fill(Black);
ssd1306_UpdateScreen();

elapsed = 5;
seconds = 0;

while(elapsed > -1){

	//keep measuring roll so that we can exit loop if the user wants to
	groll = getRoll();

	if(groll < -55 && groll > -135){
		ssd1306_Fill(Black);
		ssd1306_UpdateScreen();
		pomocounter = 0;
		curPomoState = COUNTDOWN;
		curState = BREATHE;
		return;
	}
	else if(groll > 55 && groll < 135){
		ssd1306_Fill(Black);
		ssd1306_UpdateScreen();
		curfidstate = UP;
		pomocounter = 0;
		curPomoState = COUNTDOWN;
		curState = FIDGET;
		return;
	}

	centertext("Work Time", 0, "title");
	centertext("Starting in...", 18, "text");

	char sec_str[100];

	snprintf(sec_str, sizeof(sec_str), "%li", elapsed);

	centertext(sec_str, 36, "title");

	ssd1306_UpdateScreen();

	elapsed = 5 - seconds;
	}

pomocounter += 1;
curPomoState = TIMER;

}

void pomolongbreak(){
	int32_t elapsed = 1200;
	uint32_t minutes = 0;
	uint32_t sec = 0;

	seconds = 0;

	while(elapsed > -1){
		ssd1306_Fill(Black);

		groll = getRoll();

		if(groll < -55 && groll > -135){
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen();
			pomocounter = 0;
			curPomoState = COUNTDOWN;
			curState = BREATHE;
			return;
		}
		else if(groll > 55 && groll < 135){
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen();
			curfidstate = UP;
			pomocounter = 0;
			curPomoState = COUNTDOWN;
			curState = FIDGET;
			return;
		}

		minutes = elapsed / 60;
		sec = elapsed % 60;

		char time[100];


		if(sec / 10 == 0){
			if(minutes / 10 == 0){
				snprintf(time, sizeof(time), "0%lu:0%lu", minutes, sec);
			}
			else{
				snprintf(time, sizeof(time), "%lu:0%lu", minutes, sec);
			}
		}
		else{
			if(minutes / 10 == 0){
				snprintf(time, sizeof(time), "0%lu:%lu", minutes, sec);
			}
			else{
				snprintf(time, sizeof(time), "%lu:%lu", minutes, sec);
			}
		}

			centertext("Long Break", 0, "text");
			centertext(time, 23, "title");
			ssd1306_UpdateScreen();

			elapsed = 1200 - seconds;

		}

	seconds = 0;

	ssd1306_Fill(White);
	ssd1306_UpdateScreen();

	while(seconds <= 5){

		groll = getRoll();

		if(groll < -55 && groll > -135){
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen();
			pomocounter = 0;
			curPomoState = COUNTDOWN;
			curState = BREATHE;
			return;
		}
		else if(groll > 55 && groll < 135){
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen();
			curfidstate = UP;
			pomocounter = 0;
			curPomoState = COUNTDOWN;
			curState = FIDGET;
			return;
		}

		if(seconds % 2 == 1){
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen();
		}
		else if(seconds % 2 == 0){
			ssd1306_Fill(White);
			ssd1306_UpdateScreen();
		}
	}

	ssd1306_Fill(Black);
	ssd1306_UpdateScreen();

	elapsed = 5;
	seconds = 0;

	while(elapsed > -1){

		//keep measuring roll so that we can exit loop if the user wants to
		groll = getRoll();

		if(groll < -55 && groll > -135){
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen();
			pomocounter = 0;
			curPomoState = COUNTDOWN;
			curState = BREATHE;
			return;
		}
		else if(groll > 55 && groll < 135){
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen();
			curfidstate = UP;
			pomocounter = 0;
			curPomoState = COUNTDOWN;
			curState = FIDGET;
			return;
		}

		centertext("Work Time", 0, "title");
		centertext("Starting in...", 18, "text");

		char sec_str[100];

		snprintf(sec_str, sizeof(sec_str), "%li", elapsed);

		centertext(sec_str, 36, "title");

		ssd1306_UpdateScreen();

		elapsed = 5 - seconds;
		}

	pomocounter = 0;
	curPomoState = TIMER;
}

void breathecircle(){
	seconds = 0;
	uint8_t radius = 0;
	uint8_t radiusinc = 0;
	uint32_t lasttime = 0;

	while(radius <= 25){

		groll = getRoll();

		if(groll > 155 || groll < -155){
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen();
			curState = POMO;
			return;
		}
		else if(groll < 35 && groll > -35){
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen();
			curState = EYES;
			return;
		}

		if(HAL_GetTick() - lasttime >= 140){
			radiusinc += 1;
			radius = 2 + radiusinc;
			lasttime = HAL_GetTick();
		}

			ssd1306_FillCircle(64, 32, radius, White);
			ssd1306_UpdateScreen();
	}

	seconds = 0;

	while(seconds != 4){

		groll = getRoll();

		if(groll > 155 || groll < -155){
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen();
			curState = POMO;
			return;
		}
		else if(groll < 35 && groll > -35){
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen();
			curState = EYES;
			return;
		}

		ssd1306_FillCircle(64, 32, radius, White);
		ssd1306_UpdateScreen();
	}

	seconds = 0;
	lasttime = 0;
	radiusinc = 0;

	while(radius >= 3){

		groll = getRoll();

		if(groll > 155 || groll < -155){
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen();
			curState = POMO;
			return;
		}
		else if(groll < 35 && groll > -35){
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen();
			curState = EYES;
			return;
		}

		if(HAL_GetTick() - lasttime >= 140){
			radiusinc += 1;
			radius = 25 - radiusinc;
			lasttime = HAL_GetTick();
		}

		ssd1306_Fill(Black);
		ssd1306_FillCircle(64, 32, radius, White);
		ssd1306_UpdateScreen();
	}

	seconds = 0;

	while(seconds != 4){

		groll = getRoll();

		if(groll > 155 || groll < -155){
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen();
			curState = POMO;
			return;
		}
		else if(groll < 35 && groll > -35){
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen();
			curState = EYES;
			return;
		}

		ssd1306_FillCircle(64, 32, radius, White);
		ssd1306_UpdateScreen();
		}
	}

void drawcircle(){

	circles[tapcounter].x = rand() % 121 + 5;
	circles[tapcounter].y = rand() % 55 + 5;

	ssd1306_FillCircle(circles[tapcounter].x, circles[tapcounter].y, 2, White);
	ssd1306_UpdateScreen();
	tapcounter += 1;

	}

void deletecircle(){

	ssd1306_FillCircle(circles[tapcounter].x, circles[tapcounter].y, 2, Black);
	ssd1306_UpdateScreen();
	tapcounter -= 1;

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
