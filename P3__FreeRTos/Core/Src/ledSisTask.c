/*
 * ledSisTask.c
 *
 *  Created on: Dec 18, 2023
 *      Author: alvaromontesano
 */

#include "cmsis_os.h"
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "fsm.h"
#include "antSisTask.h"

static int Sistema_Activo (fsm_t* this);
static int Sistema_No_Activo (fsm_t* this);
static int Tiempo_Ha_Pasado (fsm_t* this);
static void funcion_LED_azul_encendido (fsm_t* this);
static void funcion_LED_azul_apagado (fsm_t* this);

static uint32_t led_tick;

enum LEDazul_state {
  IDLE_LED,
  LEDAZUL
};

static int Sistema_Activo (fsm_t* this)
{
	 return valueSA();
}

static int Sistema_No_Activo (fsm_t* this)
{
    return !valueSA();
}

static int Tiempo_Ha_Pasado (fsm_t* this)
{
    if ( (osKernelGetTickCount() - led_tick) >= 1000){
    	return 1;
    }
	return 0;
}


static void funcion_LED_azul_encendido (fsm_t* this)
{
	HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15);//Led azul
	led_tick = osKernelGetTickCount();
}

static void funcion_LED_azul_apagado (fsm_t* this)
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,0); //Led azul off
}


static fsm_trans_t LEDazul[] = {
	{IDLE_LED, 		Sistema_Activo, 	    LEDAZUL,	funcion_LED_azul_encendido},
	{LEDAZUL, 	Tiempo_Ha_Pasado, 	LEDAZUL, 		funcion_LED_azul_encendido},
	{LEDAZUL, 	Sistema_No_Activo, 	IDLE_LED, 		funcion_LED_azul_apagado},
	{-1, NULL, -1, NULL },
	};

void ledSisFunction(void *argument)
{
	fsm_t * LEDazul_fsm = fsm_new (LEDazul);

	 for(;;)
	 {
	   fsm_fire (LEDazul_fsm);
	   osDelay(100); //100ms
	 }
}
