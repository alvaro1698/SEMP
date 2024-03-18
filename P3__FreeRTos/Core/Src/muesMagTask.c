/*
 * muesAceTask.c
 *
 *  Created on: Dec 18, 2023
 *      Author: alvaromontesano
 */

#include "cmsis_os.h"
#include <stdio.h>
#include <float.h>
#include "stdlib.h"
#include "stm32f4xx_hal.h"
#include "fsm.h"
#include "antSisTask.h"
#include "arm_math.h"
#include "../../STM32F411E-Discovery/stm32f411e_discovery_accelerometer.h"
#include "lsm303_mag.h"

#define MSM         5

static int Sistema_Activo (fsm_t* this);
static int Sistema_No_Activo (fsm_t* this);
static void funcion_muestreo (fsm_t* this);
static void funcion_terminar (fsm_t* this);
static int empieza_muestras (fsm_t* this);
static void funcion_comparar_calculo (fsm_t* this);
static int muestras_cumplidas (fsm_t* this);
static void funcion_calculo (fsm_t* this);

static int muestrasMag = 0;
static float maximo = 0;
static float minimo = FLT_MAX;

extern osMutexId_t i2cMutexHandle;
extern osMessageQueueId_t deltaAceQueueHandle;

static uint32_t tick;
static uint32_t sample_tick;
static uint32_t periodo;

enum muestreoP1_state {
  IDLE,
  INICIO_MUESTREO,
  MUESTREO,
};

static int Sistema_Activo (fsm_t* this)
{
	 return valueSA();
}

static int Sistema_No_Activo (fsm_t* this)
{
	 return !valueSA();
}

static void funcion_muestreo (fsm_t* this)
{
  /*tiempo_t3=0;
  HAL_TIM_Base_Start_IT(&htim10); //5ms*/

	tick = osKernelGetTickCount();
}

static void funcion_terminar (fsm_t* this)
{
	  /*HAL_TIM_Base_Stop_IT(&htim10);//timer 10 (5ms) a zero
	  muestras=0;
	  maximo=0;
	  minimo=FLT_MAX;
	  SA=0;
	  boton=0;*/

	  muestrasMag = 0;
	  maximo = 0;
	  minimo = FLT_MAX;
      sample_tick = sample_tick + periodo;
}

static int empieza_muestras (fsm_t* this)
{
	uint32_t actual = osKernelGetTickCount();
	if ((actual - sample_tick) >= periodo)
	{
		if (muestrasMag < 199) {
			return 1;
		}
	}
	return 0;
}

static void funcion_comparar_calculo (fsm_t* this)
{
	float32_t sensorH;
	float32_t sensorL;
	float32_t modulo;
	int16_t pDataLH[2];

	muestrasMag++;
	osMutexAcquire(i2cMutexHandle, osWaitForever);
	LSM303AGR_MagReadXYZ(pDataLH);
	osMutexRelease(i2cMutexHandle);

	sensorL = (float)pDataLH[0]; //para convertir a float
	sensorH = (float)pDataLH[1];

	float32_t sc = (sensorL*sensorL) + (sensorH*sensorH);
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

static int muestras_cumplidas (fsm_t* this)
{
	if(muestrasMag >= 199)
	{
		uint32_t actual = osKernelGetTickCount();
		if ((actual - sample_tick) >= periodo) {
			return 1;
		}
	}
	return 0;
}

static void funcion_calculo (fsm_t* this)
{
	funcion_comparar_calculo(this);

	uint32_t deltaAce = (uint32_t)(maximo-minimo);
	maximo = 0;
	minimo = FLT_MAX;
	muestrasMag = 0;
	osMessageQueuePut(deltaAceQueueHandle, &deltaAce, 0, osWaitForever);
}

static fsm_trans_t muestreoP1[] = {
	{ IDLE,   			Sistema_Activo, 			MUESTREO,  	funcion_muestreo },
	{ MUESTREO, 		empieza_muestras, 			MUESTREO,  	funcion_comparar_calculo },
	{ MUESTREO, 		muestras_cumplidas, 		MUESTREO,  	funcion_calculo },
	{ MUESTREO, 		Sistema_No_Activo, 			IDLE,  	funcion_terminar},
	{-1, NULL, -1, NULL },
	};

void muesMagFunction(void *argument)
{
	fsm_t * muestreoP1_fsm = fsm_new (muestreoP1);
	tick = osKernelGetTickCount();
	periodo = MSM*osKernelGetTickFreq()/1000;

	 for(;;)
	 {
	   fsm_fire (muestreoP1_fsm);
	   tick += periodo;
	   osDelayUntil(tick); ////Depende de los tick, con lo cual el periodo es mucho mayor que la fercuencia a la que se toman las muestras (5 ms) y no perdemos muestras
	 }
}
