/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    app_threadx.c
 * @author  MCD Application Team
 * @brief   ThreadX applicative file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
TX_THREAD ThreadOne;
TX_THREAD ThreadTwo;
TX_THREAD ThreadThree;
TX_THREAD ThreadFour;

TX_MUTEX Uart_Mutex;
TX_SEMAPHORE LED_Sem;
enum led_status {
	green_free, red_free, both_free, both_used
};
enum thread_led_status {
	red, green, none, both
};
uint8_t led_status[2] = { 0, 0 };

extern UART_HandleTypeDef huart1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/**
 * @brief  Application ThreadX Initialization.
 * @param memory_ptr: memory pointer
 * @retval int
 */
UINT App_ThreadX_Init(VOID *memory_ptr) {
	UINT ret = TX_SUCCESS;
	TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*) memory_ptr;

	/* USER CODE BEGIN App_ThreadX_MEM_POOL */
//  (void)byte_pool;
	CHAR *ptr_thread1, *ptr_thread2, *ptr_thread3, *ptr_thread4;

	if ((tx_byte_allocate(byte_pool, (VOID**) &ptr_thread1, APP_STACK_SIZE,
	TX_NO_WAIT)) != TX_SUCCESS) {
		ret = TX_POOL_ERROR;
	}

//  UINT        _txe_thread_create(TX_THREAD *thread_ptr, CHAR *name_ptr,
//                  VOID (*entry_function)(ULONG entry_input), ULONG entry_input,
//                  VOID *stack_start, ULONG stack_size,
//                  UINT priority, UINT preempt_threshold,
//                  ULONG time_slice, UINT auto_start, UINT thread_control_block_size);

	if (tx_thread_create(&ThreadOne,"THREAD_ONE",ThreadOne_Entry,0,
			ptr_thread1,APP_STACK_SIZE,
			THREAD_ONE_PRIO,THREAD_ONE_PREEMPTION_THRESHOLD,
			100,TX_AUTO_START) != TX_SUCCESS) {
		ret = TX_THREAD_ERROR;

	}

	if ((tx_byte_allocate(byte_pool, (VOID**) &ptr_thread2, APP_STACK_SIZE,
	TX_NO_WAIT)) != TX_SUCCESS) {
		ret = TX_POOL_ERROR;
	}
	if (tx_thread_create(&ThreadTwo,"THREAD_TWO",ThreadTwo_Entry,0,
			ptr_thread2,APP_STACK_SIZE,
			THREAD_TWO_PRIO,THREAD_TWO_PREEMPTION_THRESHOLD,
			100,TX_AUTO_START) != TX_SUCCESS) {
		ret = TX_THREAD_ERROR;

	}

	if ((tx_byte_allocate(byte_pool, (VOID**) &ptr_thread3, APP_STACK_SIZE,
	TX_NO_WAIT)) != TX_SUCCESS) {
		ret = TX_POOL_ERROR;
	}
	if (tx_thread_create(&ThreadThree,"THREAD_THREE",ThreadThree_Entry,0,
			ptr_thread3,APP_STACK_SIZE,
			THREAD_TWO_PRIO,THREAD_TWO_PREEMPTION_THRESHOLD,
			100,TX_AUTO_START) != TX_SUCCESS) {
		ret = TX_THREAD_ERROR;

	}
	if ((tx_byte_allocate(byte_pool, (VOID**) &ptr_thread4, APP_STACK_SIZE,
	TX_NO_WAIT)) != TX_SUCCESS) {
		ret = TX_POOL_ERROR;
	}
	if (tx_thread_create(&ThreadFour,"THREAD_FOUR",ThreadFour_Entry,0,
			ptr_thread4,APP_STACK_SIZE,
			THREAD_TWO_PRIO,THREAD_TWO_PREEMPTION_THRESHOLD,
			100,TX_AUTO_START) != TX_SUCCESS) {
		ret = TX_THREAD_ERROR;

	}
	tx_semaphore_create(&LED_Sem, "LED_Semaphore", 2);
	tx_mutex_create(&Uart_Mutex, "UART_Mutex", TX_NO_INHERIT);
	/* USER CODE END App_ThreadX_MEM_POOL */

	/* USER CODE BEGIN App_ThreadX_Init */
	/* USER CODE END App_ThreadX_Init */

	return ret;
}

/**
 * @brief  MX_ThreadX_Init
 * @param  None
 * @retval None
 */
void MX_ThreadX_Init(void) {
	/* USER CODE BEGIN  Before_Kernel_Start */

	/* USER CODE END  Before_Kernel_Start */

	tx_kernel_enter();

	/* USER CODE BEGIN  Kernel_Start_Error */

	/* USER CODE END  Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */
void ThreadOne_Entry(ULONG Thread_in) {
	uint8_t t1_counter = 0;
	uint8_t task_led_status = none;
	char hw[] = "ThreadOne:100ms 20 times\n\r";
	uint32_t *GPIO_Port, LED_Pin;
	while (1) {
		if (TX_SUCCESS == tx_semaphore_get(&LED_Sem, TX_WAIT_FOREVER)) {
			if (TX_SUCCESS == tx_mutex_get(&Uart_Mutex, TX_WAIT_FOREVER)) {
				HAL_UART_Transmit(&huart1, hw, strlen(hw), 1000);
				tx_mutex_put(&Uart_Mutex);
			}
			if ((led_status[0] == 0) && (led_status[1] == 0)) {
				GPIO_Port = LED_RED_GPIO_Port;
				LED_Pin = LED_RED_Pin;
				led_status[1] = 1;
				task_led_status = green;
			} else if (led_status[0] == 0) {
				GPIO_Port = LED_RED_GPIO_Port;
				LED_Pin = LED_RED_Pin;
				led_status[0] = 1;
				task_led_status = red;
			} else if (led_status[1] == 0) {
				GPIO_Port = LED_GREEN_GPIO_Port;
				LED_Pin = LED_GREEN_Pin;
				led_status[1] = 1;
				task_led_status = green;
			}
			t1_counter = 0;
			while (t1_counter <= 19) {
				t1_counter++;
				App_Delay(2);
				HAL_GPIO_TogglePin(GPIO_Port, LED_Pin);
			}

//			switch (task_led_status) {
//			case red:
//				led_status[0] = 0;
//				task_led_status = none;
//				break;
//			case green:
//				led_status[1] = 0;
//				task_led_status = none;
//				break;
//			default:
//				break;
//			}
			if (task_led_status == red) {
				led_status[0] = 0;
				task_led_status = none;
				break;
			} else if (task_led_status == green) {
				led_status[1] = 0;
				task_led_status = none;
				break;
			}
			tx_semaphore_put(&LED_Sem);

		}

	}
}

void ThreadTwo_Entry(ULONG Thread_in) {
	uint8_t t2_counter = 0;
	uint32_t *GPIO_Port, LED_Pin;
	uint8_t task_led_status = none;
	char hw1[] = "ThreadTwo:1000ms 9 times\n\r";

	while (1) {
		if (TX_SUCCESS == tx_semaphore_get(&LED_Sem, TX_WAIT_FOREVER)) {
			if (TX_SUCCESS == tx_mutex_get(&Uart_Mutex, TX_WAIT_FOREVER)) {
				HAL_UART_Transmit(&huart1, hw1, strlen(hw1), 1000);

				tx_mutex_put(&Uart_Mutex);
			}

			if ((led_status[0] == 0) && (led_status[1] == 0)) {
				GPIO_Port = LED_RED_GPIO_Port;
				LED_Pin = LED_RED_Pin;
				led_status[1] = 1;
				task_led_status = green;
			} else if (led_status[0] == 0) {
				GPIO_Port = LED_RED_GPIO_Port;
				LED_Pin = LED_RED_Pin;
				led_status[0] = 1;
				task_led_status = red;
			} else if (led_status[1] == 0) {
				GPIO_Port = LED_GREEN_GPIO_Port;
				LED_Pin = LED_GREEN_Pin;
				led_status[1] = 1;
				task_led_status = green;
			}
			t2_counter = 0;
			while (t2_counter <= 19) {
				t2_counter++;
				App_Delay(2);
				HAL_GPIO_TogglePin(GPIO_Port, LED_Pin);
			}

//			switch (task_led_status) {
//			case red:
//				led_status[0] = 0;
//				task_led_status = none;
//				break;
//			case green:
//				led_status[1] = 0;
//				task_led_status = none;
//				break;
//			default:
//				break;
//			}
			if (task_led_status == red) {
				led_status[0] = 0;
				task_led_status = none;
				break;
			} else if (task_led_status == green) {
				led_status[1] = 0;
				task_led_status = none;
				break;
			}
			tx_semaphore_put(&LED_Sem);

		}
//		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, RESET);

	}
}
void ThreadThree_Entry(ULONG Thread_in) {
	uint8_t t2_counter = 0;
	uint32_t *GPIO_Port, LED_Pin;
	uint8_t task_led_status = none;
	char hw2[] = "ThreadThree:2500ms 5 times\n\r";
	while (1) {
		if (TX_SUCCESS == tx_semaphore_get(&LED_Sem, TX_WAIT_FOREVER)) {
			if (TX_SUCCESS == tx_mutex_get(&Uart_Mutex, TX_WAIT_FOREVER)) {
				HAL_UART_Transmit(&huart1, hw2, strlen(hw2), 1000);
				tx_mutex_put(&Uart_Mutex);
			}
			if ((led_status[0] == 0) && (led_status[1] == 0)) {
				GPIO_Port = LED_RED_GPIO_Port;
				LED_Pin = LED_RED_Pin;
				led_status[1] = 1;
				task_led_status = green;
			} else if (led_status[0] == 0) {
				GPIO_Port = LED_RED_GPIO_Port;
				LED_Pin = LED_RED_Pin;
				led_status[0] = 1;
				task_led_status = red;
			} else if (led_status[1] == 0) {
				GPIO_Port = LED_GREEN_GPIO_Port;
				LED_Pin = LED_GREEN_Pin;
				led_status[1] = 1;
				task_led_status = green;
			}
			t2_counter = 0;
			while (t2_counter <= 19) {
				t2_counter++;
				App_Delay(2);
				HAL_GPIO_TogglePin(GPIO_Port, LED_Pin);
			}

//			switch (task_led_status) {
//			case red:
//				led_status[0] = 0;
//				task_led_status = none;
//				break;
//			case green:
//				led_status[1] = 0;
//				task_led_status = none;
//				break;
//			default:
//				break;
//			}
			if (task_led_status == red) {
				led_status[0] = 0;
				task_led_status = none;
				break;
			} else if (task_led_status == green) {
				led_status[1] = 0;
				task_led_status = none;
				break;
			}

			tx_semaphore_put(&LED_Sem);

		}
	}
}
void ThreadFour_Entry(ULONG Thread_in) {
	uint8_t t2_counter = 0;
	uint32_t *GPIO_Port, LED_Pin;
	uint8_t task_led_status = none;
	char hw3[] = "ThreadFour:5000ms 3 times\n\r";
	while (1) {
		if (TX_SUCCESS == tx_semaphore_get(&LED_Sem, TX_WAIT_FOREVER)) {
			if (TX_SUCCESS == tx_mutex_get(&Uart_Mutex, TX_WAIT_FOREVER)) {
				HAL_UART_Transmit(&huart1, hw3, strlen(hw3), 1000);

				tx_mutex_put(&Uart_Mutex);
			}
			if ((led_status[0] == 0) && (led_status[1] == 0)) {
				GPIO_Port = LED_RED_GPIO_Port;
				LED_Pin = LED_RED_Pin;
				led_status[1] = 1;
				task_led_status = green;
			} else if (led_status[0] == 0) {
				GPIO_Port = LED_RED_GPIO_Port;
				LED_Pin = LED_RED_Pin;
				led_status[0] = 1;
				task_led_status = red;
			} else if (led_status[1] == 0) {
				GPIO_Port = LED_GREEN_GPIO_Port;
				LED_Pin = LED_GREEN_Pin;
				led_status[1] = 1;
				task_led_status = green;
			}
			t2_counter = 0;
			while (t2_counter <= 19) {
				t2_counter++;
				App_Delay(2);
				HAL_GPIO_TogglePin(GPIO_Port, LED_Pin);
			}

//			switch (task_led_status) {
//			case red:
//				led_status[0] = 0;
//				task_led_status = none;
//				break;
//			case green:
//				led_status[1] = 0;
//				task_led_status = none;
//				break;
//			default:
//				break;
//			}
			if (task_led_status == red) {
				led_status[0] = 0;
				task_led_status = none;
				break;
			} else if (task_led_status == green) {
				led_status[1] = 0;
				task_led_status = none;
				break;
			}
			tx_semaphore_put(&LED_Sem);

		}
	}
}

void App_Delay(uint32_t Delay) {
	UINT initial_time = tx_time_get();

	while ((tx_time_get() - initial_time) < Delay)
		;
	initial_time = tx_time_get();
}
/* USER CODE END 1 */
