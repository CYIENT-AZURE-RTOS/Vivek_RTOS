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

TX_MUTEX Uart_Mutex;
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
	CHAR *pointer;
	char status[15] = "No error";

	if ((ret = tx_byte_allocate(byte_pool, (VOID**) &pointer, THREAD_ONE_STACK,
			TX_NO_WAIT)) != TX_SUCCESS) {
		sprintf(status, "F code : %d", ret);
		HAL_UART_Transmit(&huart1, status, strlen(status), 1000);
	}

//  UINT        _txe_thread_create(TX_THREAD *thread_ptr, CHAR *name_ptr,
//                  VOID (*entry_function)(ULONG entry_input), ULONG entry_input,
//                  VOID *stack_start, ULONG stack_size,
//                  UINT priority, UINT preempt_threshold,
//                  ULONG time_slice, UINT auto_start, UINT thread_control_block_size);

	if ((ret = tx_thread_create(&ThreadOne, "GREEN_THREAD", ThreadOne_Entry, 0,
			pointer, THREAD_ONE_STACK, THREAD_ONE_PRIO,
			THREAD_ONE_PREEMPTION_THRESHOLD, 1, TX_AUTO_START)) != TX_SUCCESS) {
		sprintf(status, "F code : %d", ret);
		HAL_UART_Transmit(&huart1, status, strlen(status), 1000);

	}

	if ((ret = tx_byte_allocate(byte_pool, (VOID**) &pointer, THREAD_TWO_STACK,
			TX_NO_WAIT)) != TX_SUCCESS) {
		sprintf(status, "F code : %d", ret);
		HAL_UART_Transmit(&huart1, status, strlen(status), 1000);

	}
	if ((ret = tx_thread_create(&ThreadTwo, "RED_THREAD", ThreadTwo_Entry, 0,
			pointer, THREAD_TWO_STACK, THREAD_TWO_PRIO,
			THREAD_TWO_PREEMPTION_THRESHOLD, 1, TX_AUTO_START)) != TX_SUCCESS) {
		sprintf(status, "F code : %d", ret);
		HAL_UART_Transmit(&huart1, status, strlen(status), 1000);

	}

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
	char hw[] = "Who want to see the world burn!!\n\r";
	UINT test_data = 10000;
	while (1) {
		HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);

		HAL_UART_Transmit(&huart1, hw, strlen(hw), 1000);
		tx_thread_sleep(100);

	}

}

void ThreadTwo_Entry(ULONG Thread_in) {
	uint8_t t2_counter = 0;
	char hw[] = "They can't be bought...\n\r";
	UINT test_data = 10000;
	while (1) {
		HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);

		HAL_UART_Transmit(&huart1, hw, strlen(hw), 1000);
		tx_thread_sleep(100);
	}
}
void App_Delay(uint32_t Delay) {
	UINT initial_time = tx_time_get();

	while ((tx_time_get() - initial_time) < Delay)
		;
	initial_time = tx_time_get();
}
/* USER CODE END 1 */
