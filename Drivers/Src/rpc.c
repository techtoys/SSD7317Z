/*
 * rpc.c
 *
 *  Created on: Feb 10, 2021
 *      Author: John
 */

#include "rpc.h"
#include <string.h>

static bool rpc_buffer_empty(void);
static int8_t rpc_get_command(uint8_t *buf, uint16_t *length, uint32_t timeout);

/* Buffer for Remote Procedure Call */
static uart_rx_buf rpc_rx_buf;

/**
 * @brief
 * \b	Description:<br>
 * 			This function returns the number of characters in the communication buffer
 * @return 	`true` if the buffer is empty
 * 			`false` if the buffer is not empty, message removal from the receiving buffer is required
 */
static bool rpc_buffer_empty(void)
{
	bool empty = true;

	if(rpc_rx_buf.ctr > 0)
		empty = false;

	return empty;
}

/**
 * @brief
 * \b	Description:<br>
 * 		This function is called to obtain all characters from the
 * 		communication buffer rpc_rx_buf.buf[] to a destination buffer
 * @param 	*buf is a pointer to the buffer you want to store incoming characters.
 * 			Make sure it is large enough to save the characters.
 * 			A safe array size is RPC_BUF_SIZE.
 * @param	*length points to the variable to hold the length of command receive
 * @param	timeout is the CMSIS_RTOS_TimeOutValue or 0 in case of no time-out(wait forever).
 * 			This argument is valid only when FreeRTOS is used.
 * 			For polling method, timeout does not care.
 * @return	If FreeRTOS is used, this is the status code that indicates the
 * 			execution status of the function with the same return code of osSemaphoreAcquire()
 * 			If no FreeRTOS is used, the return value is always 0
 */
static int8_t rpc_get_command(uint8_t *buf, uint16_t *length, uint32_t timeout)
{
	int8_t err = 0;

	*length = 0;

	if(!rpc_buffer_empty()){
		HAL_UART_DMAPause(&huart2);
		*length = rpc_rx_buf.ctr;

		for(uint16_t i=0; i<rpc_rx_buf.ctr; i++){
			*buf++ = rpc_rx_buf.buf[i];
		}
		rpc_rx_buf.ctr = 0; //reset the counter

		HAL_UART_DMAResume(&huart2);
	}

	return err;
}
/**
 * @brief
 * \b	Description:<br>
 * This function is called in the initialization section of main.c. Assumed USART2 is used.
 * Baud rate set to 115200, 8-n-1 with DMA1 Channel 6 (Peripheral to Memory) enabled and
 * USART2 global interrupt enabled.
 */
void rpc_uart_init(void)
{
	/* USER CODE BEGIN USART2_Init 0 */
	/* DMA controller clock enable before HAL_UART_Init(&huart2) */
	__HAL_RCC_DMA1_CLK_ENABLE();
	/* DMA1_Channel6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */

	  huart2.Instance = USART2;
	  huart2.Init.BaudRate = 115200;
	  huart2.Init.WordLength = UART_WORDLENGTH_8B;
	  huart2.Init.StopBits = UART_STOPBITS_1;
	  huart2.Init.Parity = UART_PARITY_NONE;
	  huart2.Init.Mode = UART_MODE_TX_RX;
	  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	  if (HAL_UART_Init(&huart2) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /* USER CODE BEGIN USART2_Init 2 */
	  rpc_rx_buf.ctr = 0;
	  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	  if(HAL_UART_Receive_DMA(&huart2, rpc_rx_buf.buf, RPC_BUF_SIZE)!=HAL_OK)
	  	{
	  		Error_Handler();
	  	}
	  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief
 * \b	Description:<br>
 *		This callback function is called when there is an active -> idle transition
 *		in USART2 IRQ handler (stm32l4xx_it.c). Characters are inserted into the ring buffer
 *		if it has enough room for incoming data from the DMA channel; otherwise, data will be lost.
 *		This function is declared in main.h and called in stm32l4xx.c::USART2_IRQHandler()
 */
void rpc_idle_callback(void)
{
	HAL_UART_DMAStop(&huart2);
	rpc_rx_buf.ctr = (RPC_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx));
	HAL_UART_Receive_DMA(&huart2, rpc_rx_buf.buf, RPC_BUF_SIZE);
}

/**
 * @brief
 * \b	Description:<br>
 * This is the task to run in the main loop, and this is specific to SSD7317 command and data write.
 * How to use this function:<br>
 * 1) Run a serial terminal program, e.g. YAT (Yet Another Terminal).
 * 2) Match the serial port to the VCP PORT enumerated in your PC. I have assumed you have installed NUCLEO-L432KC.
 * 3) Set baud rate to 115200,8,N,1
 * 4) From Terminal>Settings>Text Settings>Under EOL(End-Of-Line)
 * Check Separate EOL sequences for Tx and Rx and set TxEOL sequence to None
 * In Rx EOL sequence, set it to <CR><LF>
 * 5) Command protocol to send a command : 0x7E 0x63 <cmd>, <cmd> is the command bytes to send
 * e.g. 0x7E 0x63 0xAE to put OLED to sleep.
 * In Send Text window, type \h(7E 63 ae) <Enter> or F3
 *
 * Command protocol to send data: 0x7E 0x64 <data>, <data> is the data bytes to send
 * e.g. 0x7E 0x64 0xff 0x00 0xff draw a line of 24 pixels across the horizontal
 */
void rpc_main_task(void)
{
	  uint16_t length;
	  uint8_t msg[RPC_BUF_SIZE];
	  rpc_get_command(msg, &length, 0);

	  if(length!=0){
	#ifdef USE_FULL_ASSERT
		  printf("Command echo:\r\n");
		  for(uint8_t i=0; i<length; i++)
		  {
			  printf("%c", msg[i]);
		  }
		  printf("\r\n");
	#endif
		  if(length>2){
			if(msg[0]==0x7E && (msg[1]=='c')) //0x7E 0x63
			{
				spi_write_command((const uint8_t *)&msg[2], length-2);
			}
			if(msg[0]==0x7E && (msg[1]=='d')) //0x7E 0x64
			{
				spi_write_data((const uint8_t *)&msg[2], length-2);
			}
		  }
		  memset(msg, 0x00, length);
	  }
}

