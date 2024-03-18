/*
 * movTask.h
 *
 *  Created on: Dec 20, 2023
 *      Author: alvaromontesano
 */

#ifndef INC_MOVTASK_H_
#define INC_MOVTASK_H_

#include "fsm.h"
#include <stdint.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"

#define TH_ACC_HIGH         10000

typedef struct {
	fsm_t f;
	osMessageQueueId_t mov_queue;
	uint32_t threshold;
	TIM_HandleTypeDef* htim;
	uint32_t channel;
	osStatus_t valid;
	uint32_t delta;
} fsm_move_t;

#endif /* INC_MOVTASK_H_ */
