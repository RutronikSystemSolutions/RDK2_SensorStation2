/*
 * proximity_sensing_task.h
 *
 *  Created on: Jan 12, 2023
 *      Author: Gintaras
 */

#ifndef PROXIMITY_SENSING_TASK_H_
#define PROXIMITY_SENSING_TASK_H_

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

extern TaskHandle_t proximity_sensing_task_handle;
void proximity_sensing_task(void *param);

#endif /* PROXIMITY_SENSING_TASK_H_ */
