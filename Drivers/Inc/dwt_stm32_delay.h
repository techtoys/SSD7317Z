/*
 * dwt_stm32_delay.h
 * https://community.st.com/s/question/0D50X00009XkeRYSAZ/delay-in-us
 *  Created on: Feb 27, 2020
 *      Author: John
 */

#ifndef INC_DWT_STM32_DELAY_H_
#define INC_DWT_STM32_DELAY_H_

#ifdef __cplusplus
extern 'C' {
#endif

#include "stm32l4xx_hal.h"
/**
 * @brief Initializes DWT_Cycle_Count for DWT_Delay_us function
 * @return Error DWT counter
 * 1: DWT counter Error
 * 0: DWT counter works
 */
uint32_t HAL_DWT_Delay_Init(void);

/**
 * @brief This function provides a delay (in microseconds)
 * @param microseconds: delay in microseconds
 */
__STATIC_INLINE void HAL_DWT_Delay_us(volatile uint32_t microseconds)
{
 uint32_t clk_cycle_start = DWT->CYCCNT;
 /* Go to number of cycles for system */
 microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);
 /* Delay till end */
 while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}

#ifdef __cplusplus
}
#endif

#endif /* INC_DWT_STM32_DELAY_H_ */
