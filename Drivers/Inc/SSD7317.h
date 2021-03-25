/****************************************************************************
* Filename              :   SSD7317.h
* Description			: 	Device header for an PMOLED with
* 							Touch and Display Driver Integrated (TDDI)
* 							driver IC on the same die (SSD7317)
* Author                :   John Leung
* Origin Date           :   13/02/2020
* Version               :   1.0.0
* Compiler              :   GNU
* Target                :   STM32L432KCUx on NUCLEO-L432KC
* Notes                 :   IDE is STM32CubeIDE
*****************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2020 TECHTOYS CO.
* All rights reserved.</center></h2>
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
#ifndef SSD7317_H_
#define SSD7317_H_

/*******************************************************************************
* Includes
*******************************************************************************/
#include "main.h"
#include "SSD7317_Init_Table.h"
#include "dwt_stm32_delay.h"
#include "sysfont.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/*@note	Slave addresses of SSD7317 is programmable for higher flexibility:
 * 1. SA(0x53) and SA_BIOS(0x57) when touch init command 0x35 set to a value of 0x0A
 * 2. SA(0x5B) and SA_BIOS(0x5F) when touch init command 0x35 set to a value of 0x0B
 *
 * Value of touch init command 0x35 set in the table SSD7317_INIT_TBL[].
 * The directive <USE_TOUCH_SA_SET_0A> controls which option is selected.
 * The choice is arbitrary. If there is no conflict with option A in your system,
 * leaving #define USE_TOUCH_SA_SET_A is OK.
 * */
#define USE_TOUCH_SA_SET_A

#ifndef USE_TOUCH_SA_SET_A
#define USE_TOUCH_SA_SET_B
#endif


#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
* Configuration Constants
*******************************************************************************/
#define OLED_HOR_RES	96 	//panel pixel width
#define OLED_VER_RES	128	//pixel height

#ifdef USE_TOUCH_SA_SET_A
	#define TOUCH_SA		0x53 //7-bit Touch slave address
	#define TOUCH_SA_BIOS	0x57 //7-bit Touch slave address BIOS
#elif defined USE_TOUCH_SA_SET_B
	#define TOUCH_SA		0x5B
	#define TOUCH_SA_BIOS	0x5F
#endif



/******************************************************************************
* Macros
*******************************************************************************/
//https://stackoverflow.com/questions/2602823/in-c-c-whats-the-simplest-way-to-reverse-the-order-of-bits-in-a-byte
//This macro is not used. It is just for reference.
#define BIT_REVERSE(b) ( \
((b>>7)&0x01)|((b>>5)&0x02)| \
((b>>3)&0x04)|((b>>1)&0x08)| \
((b<<7)&0x80)|((b<<5)&0x40)| \
((b<<3)&0x20)|((b<<1)&0x10))

/*
 * @note This macro returns the buffer index (in byte count) corresponding to the pixel coordinates (x,y).
 * This is the device buffer index align with the scanning direction of the OLED module.
 */
#define BUFIDX(x, y)  ((x >> 3) + (y*(OLED_HOR_RES >> 3)))

/*
 * @note This macro returns the bitmask of a pixel
 */
#define PIXIDX(x)     (1 << ((x) & 7))

/*******************************************************************************
* Types and Typedefs
*******************************************************************************/
#define BLACK 0 //relates to init table SSD7317_INIT_TBL[] using 0xa6 for pixel ON/OFF config
#define WHITE 0xFF

typedef uint8_t color_t;

typedef enum{
	SINGLE_TAP_ANYKEY = 1,
	LONG_TAP_ANYKEY = 2,
	DOUBLE_TAP_ANYKEY = 3,
	SWIPE = 4,
	LARGE_OBJ = 0x40,
	ACT_ERROR = 0xFF,
}gesture_act_t;

typedef enum{
	SINGLE_TAP_ANYKEY_DETAIL = 0,
	SINGLE_TAP_KEY1 = 1,
	SINGLE_TAP_KEY2 = 2,
	SINGLE_TAP_KEY3 = 3,
	SINGLE_TAP_KEY4 = 4,
	LONG_TAP_ANYKEY_DETAIL = 0,
	LONG_TAP_KEY1 = SINGLE_TAP_KEY1,
	LONG_TAP_KEY2 = SINGLE_TAP_KEY2,
	LONG_TAP_KEY3 = SINGLE_TAP_KEY3,
	LONG_TAP_KEY4 = SINGLE_TAP_KEY4,
	DOUBLE_TAP_ANYKEY_DETAIL = 0,
	DOUBLE_TAP_KEY1 = SINGLE_TAP_KEY1,
	DOUBLE_TAP_KEY2= SINGLE_TAP_KEY2,
	DOUBLE_TAP_KEY3 = SINGLE_TAP_KEY3,
	DOUBLE_TAP_KEY4 = SINGLE_TAP_KEY4,
	SWIPE_LR = 1,
	SWIPE_DOWN = SWIPE_LR,
	SWIPE_RL = 2,
	SWIPE_UP = SWIPE_RL,
	LARGE_OBJ_DETECT = 1,
	LARGE_OBJ_RELEASE = 0,
	DETAIL_ERROR = 0
}gesture_detail_t;

typedef struct FINGER{
	uint8_t tap_down_key;
	uint8_t tap_up_key;
	gesture_act_t act;
	gesture_detail_t detail;
}finger_t;

typedef struct RECTANGLE{
	int16_t x1;
	int16_t y1;
	int16_t x2;
	int16_t y2;
}rect_t;

typedef enum {
    DISP_0_DEG,
    DISP_90_DEG,
    DISP_180_DEG,
    DISP_270_DEG,
}disp_orientation_t;

/*******************************************************************************
* Functions
*******************************************************************************/
/* Functions for the display part */
void spi_write_command(const uint8_t *command, uint16_t len);
void spi_write_data(const uint8_t *data, uint16_t len);

void ssd7317_init(void);
void ssd7317_display_on(void);
void ssd7317_display_off(void);
void ssd7317_display_clear(color_t color);
void ssd7317_set_pixel(int16_t x, int16_t y, color_t color);
void ssd7317_fill_area(rect_t area, const color_t* color, bool negative);
void ssd7317_fill_color(rect_t area, color_t color);
void ssd7317_set_contrast(uint8_t level);
void ssd7317_enter_lpm(void);

disp_orientation_t ssd7317_orientation_get();
void ssd7317_orientation_set(disp_orientation_t rotation);

/* Function for touch screen */
finger_t ssd7317_get_gesture(void);

/* Functions to draw system font and images generated by lcd-image-converter (uses sysfont.h) */
rect_t ssd7317_put_image(uint16_t left, uint16_t top, const tImage* image, bool negative);
void   ssd7317_put_image_direct(uint16_t left, int16_t top, const tImage* image);

void   ssd7317_cntnt_scroll_image(uint16_t left, int16_t start_col, int16_t end_col, const tImage* image, finger_t dir);
void   ssd7317_cons_scroll_page(rect_t subpage, uint8_t interval, uint8_t accelerate, finger_t dir);
void   ssd7317_cons_scroll_brake(void);

rect_t ssd7317_put_char(uint16_t left, uint16_t top, const tFont* font, uint16_t code, bool negative);
void   ssd7317_get_charsize(const tFont* font, uint16_t ascii_code, uint16_t *w, uint16_t *h);
rect_t ssd7317_put_string(uint16_t left, uint16_t top, const tFont* font, const char *str, bool negative);
void   ssd7317_get_stringsize(const tFont* font, const char *str, uint16_t *w, uint16_t *h);

color_t *fb_scroll_seg(rect_t area, finger_t gesture);
#ifdef __cplusplus
}
#endif
#endif /* SSD7317_H_ */
