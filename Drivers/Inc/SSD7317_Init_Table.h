/****************************************************************************
* Filename              :   SSD7317.h
* Description			: 	Initialization table for the touch screen
* 							of Solomon Systech's SSD7317 (TDDI).
* 							1-key and gesture for single tap, long tap,
* 							slide LR & RL
* Author                :   John Leung
* Origin Date           :   26/02/2020
* Version               :   1.0.0
* Compiler              :   GNU
* Target                :   STM32L432KCUx on NUCLEO-L432KC
* Notes                 :   IDE is STM32CubeIDE
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

#ifndef INC_SSD7317_INIT_TABLE_H_
#define INC_SSD7317_INIT_TABLE_H_

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>

/*******************************************************************************
* Types and Typedefs
*******************************************************************************/
typedef struct _SSLFW{
	uint32_t byte_cnt;		//byte count for content
	uint32_t crc;			//CRC checksum
	const uint8_t *content; //pointer to content of initialization table
}SSLFW;

/*******************************************************************************
* Public Variables
*******************************************************************************/
extern SSLFW FW_PM;
extern SSLFW FW_TM;
extern SSLFW FW_DM;

#ifdef __cplusplus
}
#endif
#endif /* INC_SSD7317_INIT_TABLE_H_ */
