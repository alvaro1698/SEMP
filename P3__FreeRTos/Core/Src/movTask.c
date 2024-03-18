/*
 * movTask.c
 *
 *  Created on: Dec 21, 2023
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
#include "movTask.h"
#include "FreeRTOS.h"
#include "main.h"

#define DIEZ_PORCIENTO      1000
#define CUARENTA_PORCIENTO  4000
#define NOVENTA_PORCIENTO   9000

static int Sistema_No_Activo (fsm_t* this);
static int Sistema_Activo (fsm_t* this);
static int esta_parado (fsm_t* this);
static int esta_andando (fsm_t* this);
static int esta_aviso (fsm_t* this);
static void funcion_andando (fsm_t* this);
static void funcion_aviso (fsm_t* this);
static void funcion_parado (fsm_t* this);
static void funcion_sistemaoff (fsm_t* this);

enum P1_state {
  IDLE2,
  PARADO,
  ANDANDO,
  AVISO,
};

static int Sistema_Activo (fsm_t* this)
{
	 return valueSA();
}

static int Sistema_No_Activo (fsm_t* this)
{
	 return !valueSA();
}

static int esta_parado (fsm_t* this)
{
  fsm_move_t* f_move = (fsm_move_t*)this;
  if (f_move->valid==osOK) {

    if (f_move->delta < f_move->threshold) {
	  return 1;
    }
  }
  return 0;
}



static int esta_andando (fsm_t* this)
{
	fsm_move_t* f_move = (fsm_move_t*)this;
	  if (f_move->valid==osOK) {

	    if (f_move->delta >= f_move->threshold) {
		  return 1;
	    }
	  }
	  return 0;
}

static int esta_aviso (fsm_t* this)
{
  fsm_move_t* f_move = (fsm_move_t*)this;
  if (f_move->valid==osOK) {

    if (f_move->delta < f_move->threshold) {
	  return 1;
    }
  }
  return 0;
}


static void funcion_andando (fsm_t* this)
{
	fsm_move_t* f_move = (fsm_move_t*)this;
	HAL_TIM_PWM_Start(f_move->htim, f_move->channel);
	__HAL_TIM_SET_COMPARE(f_move->htim, f_move->channel, CUARENTA_PORCIENTO);
}
static void funcion_aviso (fsm_t* this)
{
	fsm_move_t* f_move = (fsm_move_t*)this;
	HAL_TIM_PWM_Start(f_move->htim, f_move->channel);
	__HAL_TIM_SET_COMPARE(f_move->htim, f_move->channel, NOVENTA_PORCIENTO);
}
static void funcion_parado (fsm_t* this)
{
	fsm_move_t* f_move = (fsm_move_t*)this;
	HAL_TIM_PWM_Start(f_move->htim, f_move->channel);
	__HAL_TIM_SET_COMPARE(f_move->htim, f_move->channel, DIEZ_PORCIENTO);
}

static void funcion_sistemaoff (fsm_t* this)
{
	fsm_move_t* f_move = (fsm_move_t*)this;
	HAL_TIM_PWM_Stop(f_move->htim, f_move->channel); //timer 4 pwm a zero
	__HAL_TIM_SET_COMPARE(f_move->htim, f_move->channel, 0); // brillo a 0
}

static fsm_trans_t P1[] = {
    { IDLE2,    Sistema_Activo, 		PARADO,  funcion_parado },
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

void movFunction(void *argument)
{
	//fsm_t * P1_fsm = fsm_new (P1);

	fsm_move_t* f_move = (fsm_move_t*)argument;
	fsm_init(&(f_move->f), P1);
	uint32_t tick = 0;

	 for(;;)
	 {
	   //fsm_fire (P1_fsm);
	   f_move->valid = osMessageQueueGet(f_move->mov_queue, &(f_move->delta), NULL, 0);
	   fsm_fire (&(f_move->f));
	   tick += 100;
	   osDelayUntil(tick);
	 }
}
