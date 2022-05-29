/*
 * tasks.h
 *
 *  Created on: May 29, 2022
 *      Author: scottec
 */

#ifndef INC_TASKS_H_
#define INC_TASKS_H_

extern TaskHandle_t task1_Control_handle;
extern TaskHandle_t task2_MEMS_handle;
extern TaskHandle_t task3_LED_handle;

void task_control(void* parameters);
void task_mems(void* parameters);
void task_led(void* parameters);

void task_ext_callback(uint16_t GPIO_Pin);
void task_timer_callback(TIM_HandleTypeDef *htim);

#endif /* INC_TASKS_H_ */
