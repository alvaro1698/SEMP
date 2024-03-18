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
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "tim.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "fsm.h"
#include "arm_math.h"
#include "stm32f411e_discovery_accelerometer.h"
#include <float.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TH_ACC_HIGH 10000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile int boton = 0;
//volatile int state = 1;
volatile int SA = 0;
volatile int delta=0;
volatile int tiempo_t2 = 0; //500ms
volatile int tiempo_t1 = 0; //1s
volatile int tiempo_t3 = 0; //5ms
static int muestras = 0;
static float maximo=0;
static float minimo=FLT_MAX;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
volatile int state = 1;
static int boton_sin_pulsar (fsm_t* this);
static int boton_pulsado (fsm_t* this);
static int empieza_muestras (fsm_t* this);
static int muestras_cumplidas (fsm_t* this);
//static int boton_espera_sinpulsar (fsm_t* this);
//static int boton_espera_pulsado (fsm_t* this);
static int Sistema_Activo (fsm_t* this);
static int Sistema_No_Activo (fsm_t* this);
static int LED_azul_empieza (fsm_t* this);
static int LED_azul_parpadeo (fsm_t* this);
static int LED_azul_termina (fsm_t* this);
static int esta_parado (fsm_t* this);
static int esta_andando (fsm_t* this);
static int esta_aviso (fsm_t* this);
//static int sistema_off (fsm_t* this);
static void funcion_LED_azul (fsm_t* this);
static void funcion_LED_azul_empieza (fsm_t* this);
static void funcion_LED_azul_encendido (fsm_t* this);
static void funcion_LED_azul_apagado (fsm_t* this);
static void funcion_sistemaoff (fsm_t* this);
static void funcion_muestreo (fsm_t* this);
static void funcion_terminar (fsm_t* this);
static void funcion_andando (fsm_t* this);
static void funcion_aviso (fsm_t* this);
static void funcion_parado (fsm_t* this);
static void funcion_comparar_calculo (fsm_t* this);
static void funcion_calculo (fsm_t* this);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
enum muestreoP1_state {
  IDLE,
  INICIO_MUESTREO,
  MUESTREO,
};

enum LEDazul_state {
  IDLE_LED,
  REBOTE_INICIO,
  LEDAZUL_ON,
  REBOTE_FINAL,
};

enum P1_state {
  IDLE2,
  PARADO,
  ANDANDO,
  AVISO,
};



static int boton_pulsado (fsm_t* this) //boton a true
{
	if (boton == 1)
	{
		return 1;
	}
	return 0;
}

static int boton_sin_pulsar (fsm_t* this) //boton a false
{
	if (boton == 1)
	{
		return 1;
	}
	return 0;
}





static int LED_azul_empieza (fsm_t* this)
{
	if (tiempo_t2 == 1)//&& boton == 1) //boton a true y 500ms cumplidos -> Sistema activo
	{
		return 1;
	}
	return 0;
}

static int LED_azul_parpadeo (fsm_t* this)
{
	if (tiempo_t1 == 1) // 1s cumplido->LED azul parpadea
	{
		return 1;
	}
	return 0;
}

static int LED_azul_termina (fsm_t* this)
{
	if(tiempo_t2 == 1) //boton == 0 boton sin pulsar (sistema off) y 500 ms cumplidos
	{
		return 1;
	}
	return 0;
}

static int Sistema_Activo (fsm_t* this)
{
	if (SA == 1)
	{
		return 1;
	}
	return 0;
}

static int empieza_muestras (fsm_t* this)
{
	if(tiempo_t3 == 1)
	{
		return 1;
	}
	return 0;
}

static int muestras_cumplidas (fsm_t* this)
{
	if(muestras == 200)
	{
		return 1;
	}
	return 0;
}

static int Sistema_No_Activo (fsm_t* this)
{
	if (SA == 0)
	{
		return 1;
	}
	return 0;
}




static void funcion_LED_azul (fsm_t* this)
{
	tiempo_t2=0;
	tiempo_t1=0;
	HAL_TIM_Base_Start_IT(&htim2); //500 ms
	HAL_TIM_Base_Start_IT(&htim3); //1 s
}

static void funcion_LED_azul_empieza (fsm_t* this)
{
	boton=0;
	tiempo_t2=0;
	SA=1;
}

static void funcion_LED_azul_encendido (fsm_t* this)
{
	tiempo_t1=0;
	HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15);//Led azul
}

static void funcion_LED_azul_apagado (fsm_t* this)
{
	boton=0;
	tiempo_t2=0;
	SA=0;
	HAL_TIM_Base_Stop_IT(&htim3);//timer 3 (1s) zero
	HAL_TIM_Base_Stop_IT(&htim2);//timer 2 (500ms) zero
	HAL_GPIO_WritePin(GPIOD, (uint16_t) LD6_Pin,0); //Led azul off
}



static void funcion_muestreo (fsm_t* this)
{
  tiempo_t3=0;
  HAL_TIM_Base_Start_IT(&htim10); //5ms
}

static void funcion_comparar_calculo (fsm_t* this)
{
	float32_t sensorx;
	float32_t sensory;
	float32_t sensorz;
	float32_t modulo;
	int16_t pDataXYZ[3];

	muestras++;
	tiempo_t3=0;
	BSP_ACCELERO_GetXYZ(pDataXYZ);

	sensorx = 1.0*pDataXYZ[0]; //1.0* para convertir a float
	sensory = 1.0*pDataXYZ[1];
	sensorz = 1.0*pDataXYZ[2];
	float32_t sc = (sensorx*sensorx) + (sensory*sensory) + (sensorz*sensorz);
	arm_sqrt_f32(sc, &modulo);
	if(modulo>=maximo)
	{
		maximo=modulo;
	}
	else if(modulo<=minimo)
	{
		minimo=modulo;
	}
}
static void funcion_calculo (fsm_t* this)
{
	delta=maximo-minimo;
	maximo = 0;
	minimo = FLT_MAX;
	muestras = 0;
}


static int esta_parado (fsm_t* this)
{
  if (muestras>=200 && delta < TH_ACC_HIGH)
  {
	  return 1;
  }
  else
  {
	  return 0;
  }
}



static int esta_andando (fsm_t* this)
{
	if (muestras>=200 && delta > TH_ACC_HIGH)
	{
	  return 1;
	}
	else
	{
	  return 0;
	}
}

static int esta_aviso (fsm_t* this)
{
  if (muestras>=200 && delta < TH_ACC_HIGH)
  {
	 return 1;
  }
  else
  {
	 return 0;
  }
}

static void funcion_sistemaoff (fsm_t* this)
{
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1); //timer 4 pwm a zero
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0); // brillo a 0??
}

static void funcion_terminar (fsm_t* this)
{
	  HAL_TIM_Base_Stop_IT(&htim10);//timer 10 (5ms) a zero
	  muestras=0;
	  maximo=0;
	  minimo=FLT_MAX;
	  SA=0;
	  boton=0;
}

static void funcion_andando (fsm_t* this)
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 5999); //60 a 10 brillo 5999
}
static void funcion_aviso (fsm_t* this)
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 89999); //90 a 100 brillo 8999
}
static void funcion_parado (fsm_t* this)
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2999); //30 a 5 brillo 2999
}

static fsm_trans_t LEDazul[] = {
	{IDLE_LED, 		boton_pulsado, 	    REBOTE_INICIO,	funcion_LED_azul},
	{REBOTE_INICIO,	LED_azul_empieza, 	LEDAZUL_ON,		funcion_LED_azul_empieza},
	{LEDAZUL_ON, 	LED_azul_parpadeo, 	LEDAZUL_ON,		funcion_LED_azul_encendido},
	{LEDAZUL_ON, 	boton_sin_pulsar, 	REBOTE_FINAL, 	funcion_LED_azul},
	{REBOTE_FINAL, 	LED_azul_termina, 	IDLE_LED, 		funcion_LED_azul_apagado},
	{-1, NULL, -1, NULL },
	};

static fsm_trans_t muestreoP1[] = {
	{ IDLE,   			Sistema_Activo, 			MUESTREO,  	funcion_muestreo },
	{ MUESTREO, 		empieza_muestras, 			MUESTREO,  	funcion_comparar_calculo },
	{ MUESTREO, 		muestras_cumplidas, 		MUESTREO,  	funcion_calculo },
	{ MUESTREO, 		Sistema_No_Activo, 			IDLE,  	funcion_terminar},
	{-1, NULL, -1, NULL },
	};

static fsm_trans_t P1[] = {
    { IDLE2,    esta_parado, 		PARADO,  funcion_parado },
	{ PARADO,   esta_andando, 		ANDANDO, funcion_andando },
	{ ANDANDO,  esta_aviso, 		AVISO,   funcion_aviso },
	{ ANDANDO,  esta_andando, 		ANDANDO, funcion_andando },
	{ AVISO,   	esta_parado, 		PARADO,  funcion_parado },
	{ AVISO,    esta_andando, 		ANDANDO, funcion_andando },
	{ PARADO,   esta_parado, 		PARADO,  funcion_parado },
	{ PARADO,   Sistema_No_Activo, 	IDLE2,   funcion_sistemaoff },
	{ ANDANDO,   Sistema_No_Activo, 	IDLE2,   funcion_sistemaoff },
	{ AVISO,   Sistema_No_Activo, 	IDLE2,   funcion_sistemaoff },
	{-1, NULL, -1, NULL },
	};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
 	fsm_t * LEDazul_fsm = fsm_new (LEDazul);
	fsm_t * muestreoP1_fsm = fsm_new (muestreoP1);
  	fsm_t * P1_fsm = fsm_new (P1);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  BSP_ACCELERO_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	fsm_fire (LEDazul_fsm);
	fsm_fire (muestreoP1_fsm);
   	fsm_fire (P1_fsm);
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 200;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 5;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim2)
	{
		tiempo_t2=1; //500ms cumplido
	}
	if(htim == &htim3)
	{
		tiempo_t1=1; //1s cumplido
	}
	if(htim == &htim10)
	{
		tiempo_t3=1; //5ms cumplido
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) // para el boton interrupciones
{
  if(GPIO_Pin == GPIO_PIN_0)
  {
	  boton=1;
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
