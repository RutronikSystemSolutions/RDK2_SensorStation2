/*
 * gesture_control_task.h
 *
 *  Created on: Jan 9, 2023
 *      Author: Gintaras
 */

#ifndef GESTURE_CONTROL_TASK_H_
#define GESTURE_CONTROL_TASK_H_

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

extern TaskHandle_t gesture_control_task_handle;

void gesture_control_task(void *param);


#endif /* GESTURE_CONTROL_TASK_H_ */
