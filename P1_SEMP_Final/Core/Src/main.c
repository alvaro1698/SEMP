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
#include "fsm.h"
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

/* USER CODE BEGIN PV */
//bool boton como?
volatile int boton = 0;
volatile int sensor = 0;
volatile int tiempo_t2 = 0;
volatile int tiempo_t1 = 0;
static int muestras = 0;
static int movimientos = 0;
//salidas
/*static int stop = 0;
static int walking = 0;
static int warning = 0;*/
//static fsm_t * muestreoP1_fsm;
//static fsm_t * P1_fsm;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
static int boton_sin_pulsar (fsm_t* this);
static int boton_pulsado (fsm_t* this);
static int empieza_muestras (fsm_t* this);
//static int empieza_movimiento (fsm_t* this);
static int boton_espera (fsm_t* this);
static int esta_parado (fsm_t* this);
static int esta_andando (fsm_t* this);
static int esta_aviso (fsm_t* this);
static void funcion_muestreo (fsm_t* this);
static void funcion_terminar (fsm_t* this);
static void funcion_andando (fsm_t* this);
static void funcion_aviso (fsm_t* this);
static void funcion_parado (fsm_t* this);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
enum muestreoP1_state {
  IDLE,
  MUESTREO,
  TERMINAR,
};

enum P1_state {
  IDLE2,
  PARADO,
  ANDANDO,
  AVISO,
};

static int boton_sin_pulsar (fsm_t* this) //boton a false
{
	/*if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==0) //boton a false
		{
			return 1;
		}
		return 0;*/
	if (boton == 0)
	{
		return 1;
	}
	return 0;
	//return !boton;
}
static int boton_pulsado (fsm_t* this) //boton a true
{
	/*if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==1) //boton a true
		{
			return 1;
		}
		return 0;*/
	if (boton == 1)
	{
		return 1;
	}
	return 0;
}

static int empieza_muestras (fsm_t* this)
{
	if (tiempo_t1 == 1 && boton == 0)
	{
		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15); //Led azul
		muestras++;
		tiempo_t1=0;
		if((HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_11)==1))
		{
			movimientos++;
			tiempo_t1=0;

		}
		return 0;
		if(muestras>=20)
		{
			return 1;
		}

	}
}



	/*if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==0) //boton a false
	{
		muestras++;
		return 1;
	}
	return 0;
	*/
	/*if (boton==0)
	{
		//HAL_TIM_Base_Start_IT(&htim1);// para inicializar el timer? lo hace solo

		if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9)==1) //timer 1 (1 segundo)
			{
				 // contador de las muestras
				//HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15); // led azul
					muestras++;
					return 1;

			}
	}*/

/*static int empieza_movimiento (fsm_t* this)
{
	if (tiempo_t1 == 1 && boton == 0)
	{
		if((HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_11)==1))
		{
			HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15); //Led azul
			movimientos++;
			tiempo_t1=0;
			return 1;
		}
	}
	return 0;*/

	/*if((HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==0) && (HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_11)==1)) //boton a false y sensor a 1
		{
			movimientos++;
			return 1;
		}
		return 0;*/

	/*if (boton==0)
	{

		if(sensor == 1 && HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9)==1) //timer 1 (1 segundo) y sensor activo
		{
			movimientos++;// contador de los movimientos
			HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15); // led azul
			return 1;

		}

	}
	return 0;*/


static int esta_parado (fsm_t* this)
{
  if ((muestras>=20)&&(movimientos<4))
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
	if ((muestras>=20)&&(movimientos>4))
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
  if ((muestras>=20)&&(movimientos<=4))
  {
	 return 1;
  }
  else
  {
	 return 0;
  }
}
static void funcion_muestreo (fsm_t* this)
{
  boton=0;
  muestras=0;
  movimientos=0;
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim2);

}
/*static void funcion_espera (fsm_t* this)
{
	if(boton==0)
	{
		HAL_TIM_Base_Start_IT(&htim2);
		return 1;
	}
	return 0;
	//HAL_TIM_Base_Start_IT(&htim2);
	//HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)=1;
}*/
static int boton_espera (fsm_t* this)
{
	if (boton==0 && tiempo_t2==1) //500ms pasados
	{
		return 1;
		tiempo_t2=0;

		/*if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)==1) //timer 2 (500 ms)
		{
			return 1;

		}*/
	}
	return 0;
}

static void funcion_terminar (fsm_t* this)
{
	  HAL_TIM_Base_Stop_IT(&htim3);//timer 1 zero
	  HAL_TIM_Base_Stop_IT(&htim2); //timer 2 a zero
	  muestras=0;
	  movimientos=0;
	  boton=0;
	  HAL_GPIO_WritePin(GPIOD, (uint16_t) LD6_Pin,0); //Led azul off
	  HAL_GPIO_WritePin(GPIOD, (uint16_t) LD5_Pin,0); //Led rojo off
	  HAL_GPIO_WritePin(GPIOD, (uint16_t) LD4_Pin,0); //Led verde off
	  HAL_GPIO_WritePin(GPIOD, (uint16_t) LD3_Pin,0); //Led naranja off
}

static void funcion_andando (fsm_t* this)
{
	HAL_GPIO_WritePin(GPIOD,(uint16_t) LD3_Pin,1); //Led naranja on
	HAL_GPIO_WritePin(GPIOD,(uint16_t) LD5_Pin,0); //Led rojo off
	HAL_GPIO_WritePin(GPIOD,(uint16_t) LD4_Pin,0); //Led verde off
	muestras=0;
	movimientos=0;
	/*stop = 0;
	walking = 1;
	warning = 0;*/

}
static void funcion_aviso (fsm_t* this)
{
	HAL_GPIO_WritePin(GPIOD,(uint16_t) LD5_Pin,1); //Led rojo on
	HAL_GPIO_WritePin(GPIOD,(uint16_t) LD3_Pin,0); //Led naranja off
	HAL_GPIO_WritePin(GPIOD,(uint16_t) LD4_Pin,0); //Led verde off
	muestras=0;
	movimientos=0;
	/*stop = 0;
	walking = 0;
	warning = 1;*/
}
static void funcion_parado (fsm_t* this)
{
	HAL_GPIO_WritePin(GPIOD,(uint16_t) LD4_Pin,1); //Led verde on
	HAL_GPIO_WritePin(GPIOD,(uint16_t) LD3_Pin,0); //Led naranja off
	HAL_GPIO_WritePin(GPIOD,(uint16_t) LD5_Pin,0); //Led rojo off
	muestras=0;
	movimientos=0;
	/*stop = 1;
	walking = 0;
	warning = 0;*/
}


static fsm_trans_t muestreoP1[] = {
	{ IDLE,   	boton_sin_pulsar, 				MUESTREO,  	funcion_muestreo },
	{ MUESTREO, empieza_muestras, 				MUESTREO,  	funcion_muestreo },
	//{ MUESTREO, empieza_movimiento, 			MUESTREO,  	funcion_muestreo },
	{ MUESTREO, boton_pulsado, 					TERMINAR,  	funcion_terminar},
	{ TERMINAR, boton_espera, 					TERMINAR, 	funcion_terminar},
	{ TERMINAR, boton_pulsado, 					MUESTREO,  	funcion_muestreo },
	{-1, NULL, -1, NULL },
	};

static fsm_trans_t P1[] = {
    { IDLE2,    esta_parado, 	PARADO,  funcion_parado },
	{ PARADO,   esta_andando, 	ANDANDO, funcion_andando },
	{ ANDANDO,  esta_aviso, 	AVISO,   funcion_aviso },
	{ AVISO,   	esta_parado, 	PARADO,  funcion_parado },
	//{ PARADO,   esta_aviso, 	AVISO,   funcion_aviso },
	{ AVISO,   	esta_andando, 	ANDANDO, funcion_andando },
    //{ ANDANDO,  esta_parado, 	PARADO,  funcion_parado },
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  	/*HAL_TIM_Base_Start_IT(&htim1);
  	HAL_TIM_Base_Start_IT(&htim2);
  	muestreoP1_fsm = fsm_new (muestreoP1);
  	P1_fsm = fsm_new (P1);*/

  //timer_val= _HAL_TIM_GET_COUNTER(&htim1); //Get current time
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	fsm_fire (muestreoP1_fsm);
   	fsm_fire (P1_fsm);
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

    /*    if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == GPIO_PIN_SET) //boton a true
        {
    		timer_val=0;
    		 if (_HAL_TIM_GET_COUNTER(&htim1) - timer_val >= 5000)  // 0.5 segundo
    		        	 {
    		        		 //timer_val= _HAL_TIM_GET_COUNTER(&htim1);
    		        		 terminar();
    		        	 }
        }
    else (_HAL_TIM_GET_COUNTER(&htim1) - timer_val >= 10000)  // 1 segundo
        {
        	timer_val= _HAL_TIM_GET_COUNTER(&htim1);
        	HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15); // led azul
        	muestreo();
        }
*/
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
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
	if(htim == &htim3)
	{
		tiempo_t1=1; //1s cumplido

		//HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15); //LED AZUL
		//empieza_muestras(muestreoP1_fsm);
	}
	if(htim == &htim2)
	{
		tiempo_t2=1; //500ms se han cumplido
	}
		/*if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==1) //boton a true pasa el tiempo y a terminar
		{
			terminar(muestreoP1_fsm);


		}
		else // boton a false pasa el tiempo y vuele a muestras
		{
			empieza_muestras(muestreoP1_fsm);
		}*/

}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) // para el boton interrupciones
{
  if(GPIO_Pin == GPIO_PIN_0)
  {
	  boton=1;
	  /*if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==1) //boton a true
	  {
		  funcion_espera(muestreoP1_fsm);
		  //HAL_TIM_Base_Start_IT(&htim2);

	  }
	  else
	  {
		  funcion_espera(muestreoP1_fsm);
		  //HAL_TIM_Base_Start_IT(&htim2);
	  }*/

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
