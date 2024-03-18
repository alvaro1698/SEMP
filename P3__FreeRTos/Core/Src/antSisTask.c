/*
 * antiSisTask.c
 *
 *  Created on: Dec 18, 2023
 *      Author: alvaromontesano
 */

#include "cmsis_os.h"
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "main.h"

static volatile int SA = 0;

//extern osSemaphoreId_t buttonSemHandle;

int valueSA()
{
	return SA;
}

void antSisFunction(void *argument)
{
	 for(;;)
	 {
		osSemaphoreAcquire(buttonSemHandle, 0);   //boton = 0;
		osSemaphoreAcquire(buttonSemHandle, osWaitForever);  //while (!boton);
		SA = !SA;

	   osDelay(500); //500ms
	 }
}
