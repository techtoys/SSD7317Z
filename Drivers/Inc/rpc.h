/****************************************************************************
* Filename              :   rpc.h
* Description			: 	Header file for Remote Procedure Call as a
* debug module in SSD7317 TDDI display driver.
* Author                :   John Leung
* Origin Date           :   Feb 10, 2021
* Version               :   1.0.0
* Compiler              :   GNU
* Target                :   STM32L432KCUx on NUCLEO-L432KC
* Notes                 :   IDE is STM32CubeIDE
*
* To use this module you need:
* 1) assign USART2 as the communication channel from STM32CubeIDE
* 2) enable DMA1 channel 6 and turn on USART2 global interrupt
* 3) add the following code in USART2_IRQHandler():
* ```
* if(RESET!=__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))
  {
	  __HAL_UART_CLEAR_IDLEFLAG(&huart2);
	  rpc_idle_callback();
  }
* ```
* 4) include this file in main.h
* 5) include rpc.c in the Project
* 6) call rpc_uart_init() in main(void)
* 7) call rpc_main_task() in while(1) loop of main(void)
*****************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY TECHTOYS CO. "AS IS" AND ANY EXPRESSED
* OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL BENINGO ENGINEERING OR ITS CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
*****************************************************************************/
#ifndef INC_RPC_H_
#define INC_RPC_H_

#include <stdint.h>
#include "SSD7317.h"

/* huart2 declared in main.c */
extern UART_HandleTypeDef huart2;
/* hdma_usart2_rx declared in main.c */
extern DMA_HandleTypeDef hdma_usart2_rx;

/* Ring buffer for Remote Procedure Call */
#define RPC_BUF_SIZE 255
typedef struct {
	uint16_t ctr;
	uint8_t  buf[RPC_BUF_SIZE];
}uart_rx_buf;



void rpc_uart_init(void);
void rpc_idle_callback(void);
void rpc_main_task(void);

#endif /* INC_RPC_H_ */
