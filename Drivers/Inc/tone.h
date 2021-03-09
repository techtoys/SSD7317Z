/****************************************************************************
* Filename              :   tone.h
* Description			: 	Header file for PWM generator to drive a buzzer
* on Solomon Systech TDDI-7317 OLED EVK
* Author                :   John Leung
* Origin Date           :   March 9, 2021
* Version               :   1.0.0
* Compiler              :   GNU
* Target                :   STM32L432KCUx on NUCLEO-L432KC
* Notes                 :   IDE is STM32CubeIDE
*
* To use this module you need to:
* 1) set PA1 as TIM2_CH2
* 2) enable TIM2 > Internal Clock(Internal Clock) > Channel2(PWM Generation CH2) in Touchscreen.ioc
* 3) set Prescaler a value of (320-1) in Touchscreen.ioc
* 4) set Counter Period a value of (100-1) in Touchscreen.ioc
* 5) include this file in main.c
* 6) include tone.c in the Project
* 7) call tone_pwm_init() in main(void)
* 8) call tone_pwm_set(uint16_t freq) to set frequency, and
* call tone_pwm_on()/tone_pwm_off() to turn buzzer on/off
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

#ifndef INC_TONE_H_
#define INC_TONE_H_

#include <stdint.h>


void tone_pwm_init(void);
void tone_pwm_set(uint16_t freq);
void tone_pwm_on(void);
void tone_pwm_off(void);


#endif /* INC_TONE_H_ */
