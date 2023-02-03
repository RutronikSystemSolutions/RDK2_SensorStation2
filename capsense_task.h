/*
 * capsense_task.h
 *
 *  Created on: Feb 2, 2023
 *      Author: Gintaras
 */

#ifndef CAPSENSE_TASK_H_
#define CAPSENSE_TASK_H_

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

extern TaskHandle_t capsense_task_handle;
extern uint32_t slider_position;
void capsense_task(void *param);

#endif /* CAPSENSE_TASK_H_ */
