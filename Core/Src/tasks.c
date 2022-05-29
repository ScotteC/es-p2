#include <stdio.h>
#include "main.h"
#include "MEMS_sensor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_it.h"


TaskHandle_t task1_Control_handle;
TaskHandle_t task2_MEMS_handle;
TaskHandle_t task3_LED_handle;

typedef enum {
	init,
	idle,
	start,
	eval,
	success,
	fail
} ctrl_state_t;


void task_control(void* parameters) {

	ctrl_state_t state = init;

	uint32_t direction_set,
			 direction_tilt,
			 game_start;

	uint8_t start_blink = 6;

	uint32_t try = 0;

	  uint32_t dummy;
	  HAL_RNG_GenerateRandomNumber(&hrng, &dummy);

	while(1) {
		switch (state) {
		case init:
			// switch all leds off
			xTaskNotify(task3_LED_handle, 0, eSetValueWithOverwrite);
			taskYIELD();

			// maybe rotate direction led with timer, just because...
			state = idle;
			break;

		case idle:
			start_blink = 6;
			state = start;
//			xTaskNotify(task3_LED_handle, 0, eSetValueWithOverwrite);

//			if (xTaskNotifyWait( 0, 0, &game_start, 100)) {
//				HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
//				start_blink = 6;
//				state = start;
//			}
//			else {
//				HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
//				vTaskDelay(pdMS_TO_TICKS(100));
//			}
			break;

		case start:
			if (start_blink-- > 0) {
				xTaskNotify(task3_LED_handle, ((1 << 31)|(1<<7)), eSetValueWithOverwrite);
				vTaskDelay(pdMS_TO_TICKS(500));
			}
			else {
				xTaskNotify(task3_LED_handle, 0, eSetValueWithOverwrite);
			    uint32_t rnd;
				HAL_RNG_GenerateRandomNumber(&hrng, &rnd);
				uint32_t rnd_time = (rnd % (4000 - 1000 + 1)) + 1000;

				direction_set = (1 << (rnd % 4));

				try = 10000; // ToDo: use timer....

				vTaskDelay(pdMS_TO_TICKS(rnd_time));

				xTaskNotify(task3_LED_handle, direction_set, eSetValueWithOverwrite);
				xTaskNotify(task2_MEMS_handle, 1, eSetBits);
				state = eval;
			}
			break;

		case eval:
			xTaskNotifyWait( 0, 0, &direction_tilt, 0);
			if (direction_tilt != 0) {
				state = direction_tilt == direction_set ? success : fail;
				xTaskNotify(task2_MEMS_handle, 2, eSetBits);
			}
			else if (--try <= 0) {
				xTaskNotify(task2_MEMS_handle, 2, eSetBits);
				state = fail;
			}
		break;

		case success:
			xTaskNotify(task3_LED_handle, (direction_set | (1 << 7)), eSetValueWithOverwrite);
			vTaskDelay(pdMS_TO_TICKS(2000));
			state = init;
			break;

		case fail:
			xTaskNotify(task3_LED_handle, (direction_set | (1 << 8)), eSetValueWithOverwrite);
			vTaskDelay(pdMS_TO_TICKS(2000));
			state = init;
			break;

		default:
			break;
		}
	}
}

void task_mems(void* parameters) {

	init_mems();

	int16_t mems_raw[3];
	float mems_scaled[3];
	float scale = (4.0f / 32768);

	uint8_t uart[11] =
		{ 0x16, 0x02, 0x9, 0x0, 0x1, 0, 0, 0, 0, 0, 0};

	uint32_t notify_val = 0;

	while(1) {
		xTaskNotifyWait(0, 0, &notify_val, 0);
		if (notify_val & (1 << 31)) {
			notify_val &= ~(1 << 31);

			read_mems(mems_raw);
			mems_scaled[0] = (float)mems_raw[0] * scale;
			mems_scaled[1] = (float)mems_raw[1] * scale;
			mems_scaled[2] = (float)mems_raw[2] * scale;

			// evaluate tilt
			uint32_t dir = 0;
			if (mems_scaled[0] > 0.7f)
				dir = (1 << 2);
			else if (mems_scaled[0] < -0.7f)
				dir = (1 << 0);
			else if (mems_scaled[1] > 0.7f)
				dir = (1 << 1);
			else if (mems_scaled[1] < -0.7f)
				dir = (1 << 3);

			if (notify_val & (1 << 0))
				xTaskNotify(task1_Control_handle, dir, eSetValueWithOverwrite);
			else if (notify_val & (1 << 1))
				notify_val &= ~((1 << 1)|(1 << 0));

			uart[5]  = (mems_raw[0] >> 8) & 0xFF;
			uart[6]  = (mems_raw[0] >> 0) & 0xFF;
			uart[7]  = (mems_raw[1] >> 8) & 0xFF;
			uart[8]  = (mems_raw[1] >> 0) & 0xFF;
			uart[9]  = (mems_raw[2] >> 8) & 0xFF;
			uart[10] = (mems_raw[2] >> 0) & 0xFF;

			HAL_USART_Transmit(&husart2, uart, 11, 100);

		}
	}
}

void task_led(void* parameters) {

	uint32_t leds;

	while (1) {
		if (xTaskNotifyWait(0, 0, &leds, 0)) {
			// toggle mode
			if (leds & (1 << 31)) {
				if (leds & (1 << 0)) HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
				if (leds & (1 << 1)) HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
				if (leds & (1 << 2)) HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
				if (leds & (1 << 3)) HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);

				if (leds & (1 << 7)) HAL_GPIO_TogglePin(LD7_GPIO_Port, LD7_Pin);
				if (leds & (1 << 8)) HAL_GPIO_TogglePin(LD8_GPIO_Port, LD8_Pin);
			}
			// set mode
			else {
				HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, (leds & (1 << 0)) ? GPIO_PIN_SET : GPIO_PIN_RESET );
				HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, (leds & (1 << 1)) ? GPIO_PIN_SET : GPIO_PIN_RESET );
				HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, (leds & (1 << 2)) ? GPIO_PIN_SET : GPIO_PIN_RESET );
				HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, (leds & (1 << 3)) ? GPIO_PIN_SET : GPIO_PIN_RESET );

				HAL_GPIO_WritePin(LD7_GPIO_Port, LD7_Pin, (leds & (1 << 7)) ? GPIO_PIN_SET : GPIO_PIN_RESET );
				HAL_GPIO_WritePin(LD8_GPIO_Port, LD8_Pin, (leds & (1 << 8)) ? GPIO_PIN_RESET : GPIO_PIN_SET );
			}
		}
	}
}


/*// Code for readout the acceleration sensor:
 *
 // Initialization and variables
 * float ax = 0.f, ay = 0.f, az = 0.f;
 * uint8_t status = 0;
 * float scaling = 2.f/32768;
 * int16_t acceleration[3];
 * init_mems();
 *
 * // readout the acceleration sensor values
 * platform_read(status_, &status, 1);
 * if((status & 0x8) >> 3 == 1) // data ready
 * {
 * read_mems(acceleration);
 * ax = acceleration[0]*scaling;
 * ay = acceleration[1]*scaling;
 * az = acceleration[2]*scaling;
 * }
 *
 * // A threshold of +-0.7 for az and ax should be suitable, so
 *  az >  0.7 signals UP movement (orange LED, LD3 )
 *  az < -0.7 signals DOWN movement (blue LED, LD6)
 *  ax > 0.7 signals RIGHT movement (red LED, LD5)
 *  ax <-0.7 signals LEFT movement (green LED, LD4)
 */


void task_ext_callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == MEMS_INT1_Pin) {
		xTaskNotifyFromISR(task2_MEMS_handle,(0 | (1 << 31)), eSetBits, pdFALSE);
	}
}

void task_timer_callback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6) {
		xTaskNotifyFromISR(task1_Control_handle, (0 | (1 << 31)), eSetBits, pdFALSE);
	}
}

void ITM_print(const char* msg, const uint8_t len){
	for(int i = 0; i < len ; i++){
		ITM_SendChar(msg[i]);
		if (msg[i] == '\0'){
			break;
		}
	}
}
