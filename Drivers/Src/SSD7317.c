 /****************************************************************************
* Filename              : SSD7317.c
* Description		: Device driver for an PMOLED with
* 			  Touch and Display Driver Integrated (TDDI)
* 			  driver IC on the same die (SSD7317)
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
/**
 * @note	How fast is the SPI transfer fast enough?
 * 		A 16MHz SPI is used for data write. An OLED of 96*128 pixels
 * 		is mapped to a frame buffer of (96/8)*128 = 1536bytes.
 * 		DMA SPI transfer takes ~1.25us for each data write.
 * 		Therefore, a frame takes 1536*1.25us = 1,920us (i.e. 1.92ms).
 * 		FR pulse measured at 2.4ms, with a display period of 9.8ms.
 * 		In conclusion, a full frame refresh synchronized with FR pulse is a possible
 * 		way to solve the tearing issue.
 */

/*******************************************************************************
* Includes
*******************************************************************************/
#include "SSD7317.h"
/*******************************************************************************
* Static Variables
*******************************************************************************/

/*@note touch_event_flag is a boolean flag to describe the hardware pin IRQ status.
 * 	On a valid touch event, IRQ pin goes from high to low.
 * 	An GPIO edge trigger interrupt is used in the firmware to detect the pulse
 * */
static volatile bool touch_event_flag = false;

/*@note fb_flush_pending is a boolean flag to indicate that
 * 	there are pending SPI transfer to update the GUI content
 * */
static volatile bool fb_flush_pending = false;

static const uint8_t SSD7317_INIT_TBL[]=
{		0xfd,	//enter command lock/unlock
		0x12,	//0x12(unlock), 0x16(lock)
		0xae,	//display off (sleep mode)

		0xd5,	//Set Front Clock Divider /Oscillator Frequency
		0xc0,	//Parameter after 0xd5 :
			//A[3:0] sets divide ratio of the display clock, reset value is 0000b, i.e. divide ratio=1.
			//A[7:4] sets the oscillator frequency, reset value is 1011b in range 0000b~1111b
			//value 0xc0 set a frame frequency 100Hz (max value 0xf0 set a frame frequency 160Hz)

		0x20,	//Set Memory Addressing Mode
		0x01,	//COM-page mode

		0x21,	//command to set SEGMENT address
		0x00,	//e.g. 0x7f = 127(decimal)
		OLED_VER_RES-1,

		0x22,	//command to set COM address
		0x00,
		(OLED_HOR_RES>>3)-1,

		0xda,	//set COM pins hw config
		0x12,

		0x81,	//set contrast control
		0x4f,

		0xad,	//set internal IREF selection
		0x10,

		0xa1,	//set segment remap (vertical flip if this value set 0xa0)

		0xd3,	//set display offset for COM
		0x00,	//start from COM0

		0xc8,   //set COM output scan direction
			//top left corner at (COM95,SEG127), top right corner at (COM0,SEG127)
			//bottom left corner at (COM95, SEG0), & bottom right corner at (COM0, SEG0)
			//with byte orientation COM95[LSB] - COM88[MSB].
			//If we need to display a pattern 10110101 at the top left corner,
			//we need to send 0xAD by SPI_Write_Data() but not 0xB5.
			//Most image converter program map MSB to the left-most bit position (COM95) by default.
			//In case there is no bit reverse from the PC software, we need a macro `BIT_REVERSE(b)`
			//to flip the bit positions by swapping MSB to LSB.

		0xa2,	//set display start line
		0x00,	//set start line 00


		0xd9,	//Set Pre-charge Period
		0x22,

		0xdb,	//set VCOM de-select level
		0x30,

		0xa8,	//set multiplex ratio
		0x5F,	//OLED_HOR_RES-1,


		0xa4,	//0xa4:display RAM; 0xa5:display all pixels
		0xa6,	//0xa6:0 is pixel OFF (BLACK), 1 is pixel ON (WHITE); 0xa7:0 is pixel ON, 1 is OFF

		0x31,	//initialization sequence for touch module
		0xd0,
		0x34,
		0x0f,
		0x37,
		0x01,
		0x36,
		0x0f,
		0x35,
#ifdef USE_TOUCH_SA_SET_A //option A with #define USE_TOUCH_SA_SET_A in SSD7317.h
		0x0a,
#else
		0x0b,
#endif
		//0xaf	//display ON command (leave it to ssd7317_display_on() in ssd7317_init())
};

static SPI_HandleTypeDef hspi1;
static I2C_HandleTypeDef hi2c1;
/* hdma_spi1_tx as an argument for HAL_DMA_IRQHandler()in stm32l4xx_it.c
 * It should be global because stm32l4xx_it.c needs it*/
#ifdef USE_SPI_DMA
DMA_HandleTypeDef hdma_spi1_tx;
#endif

/**
 * @note frame_buffer[] is a map of the OLED screen content.
 * One needs to pay attention on read/write operation when a RTOS is implemented
 * because there may be more than one task to gain access to the frame buffer,
 * e.g. the real-time-clock needs to update the clock and at the same time,
 * the GUI front-end to refresh the screen in response to user interaction.
 */
color_t frame_buffer[OLED_VER_RES*(OLED_HOR_RES>>3)];

/*@note Local variable to describe the frame buffer flush area in EXTI interrupt.
 * See HAL_GPIO_EXTI_Callback() for its use.*/
static rect_t fb_flush_area;

static disp_orientation_t scr_orientation;
/*******************************************************************************
* Static Function Prototypes
*******************************************************************************/
static inline int16_t min(int16_t a, int16_t b);
static inline int16_t max(int16_t a, int16_t b);

static void MX_GPIO_Init(void);
#ifdef USE_SPI_DMA
static void MX_DMA_Init(void);
#endif
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);

/* spi_write_command() and spi_write_data() moved to SSD7317.h for rpc.c */
//void spi_write_command(const uint8_t *command, uint16_t len);
//void spi_write_data(const uint8_t *data, uint16_t len);

/*Functions with operations on frame buffer with fb_ prefix*/
static void fb_set_pixel(int16_t x, int16_t y, color_t color);
static void fb_fill_area(rect_t area, const color_t* color, bool negative);
static void fb_clear(rect_t area, color_t color);

static void fb_spi_transfer(rect_t area);

static void fb_flush_pending_set(rect_t area);
static bool fb_flush_pending_get(void);
static void fb_flush_pending_clear(void);
static void fb_flush_suspend(void);

static color_t *fb_scroll_segment(rect_t area, finger_t gesture);

static void i2c_write(uint8_t slave, uint16_t reg, const uint8_t *data, uint16_t len);
static void i2c_read(uint8_t slave, uint16_t reg, uint8_t *buffer, uint16_t len);
static uint16_t touch_crc_checksum(uint16_t byte_cnt, uint8_t trig_cmd);
static void touch_init(void);
static void touch_event_set(void);
static void touch_event_clear(void);
static bool touch_event_get(void);

/*******************************************************************************
* IRQ callback function
*******************************************************************************/
/**
 * @brief
 * \b	Description:<br>
 * 	External interrupt detection callback for two cases:<br>
 * 	(1) Touch event triggered by a falling edge on IRQ pin (PA12),<br>
 * 	(2) Frame synchronization signal on a rising edge on FR pin (PB0).<br>
 *
 * @param  GPIO_Pin Specifies the port pin connected to corresponding EXTI line.
 *
 * \b Note:<br>
 * 	This function overrides its "weak" counterpart declared in stm32l4xx_hal_gpio.c.
 *	for two scenario:<br>
 * 	(1) Touch event - stm32l4xx_it.c::EXTI15_10_IRQHandler() >
 * 			stm32l4xx_hal_gpio.c::HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12) >
 * 			stm32l4xx_hal_gpio.c::HAL_GPIO_EXTI_Callback(GPIO_PIN_12).<br/>
 *	(2) FR high pulse - stm32l4xx_it.c::EXTI0_IRQHandler() >
 *			stm32l4xx_hal_gpio.c::HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0) >
 *			stm32l4xx_hal_gpio.c::HAL_GPIO_EXTI_Callback(GPIO_PIN_0).<br/>
 *
 * \b Pre-requisite:<br>
 * 	(1) External Interrupt Mode with falling edge trigger detection enabled
 * 	for TCH_GPIO_IRQ_Pin (PA12:IRQ interface for SSD7317).<br>
 * 	(2) NVIC for EXTI line[15:10] interrupts enabled.<br>
 * 	Both (1) and (2) are configured in STM32CubeMX application on SSD7317Driver.ioc.<br>
 * 	(3) External Interrupt Mode with rising edge trigger detection enabled
 * 	for OLED_GPIO_FR_Pin (PB0).<br>
 * 	(4) NVIC for EXTI line[0] interrupts enabled.<br>
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin==OLED_FR_Pin)
	{
		// FR signal synchronized
		// Copy frame buffer to GDDRAM on-the-spot of this interrupt callback if there is pending data to flush
		if(fb_flush_pending_get()){
			fb_spi_transfer(fb_flush_area);
		}
	}
	else if (GPIO_Pin==TCH_IRQ_Pin)
	{
		//Set flag for ssd7317_get_gesture() on a valid touch event.
		//Polling ssd7317_get_gesture() is required in non-RTOS environment.
		touch_event_set();
	}
}

/*******************************************************************************
* SPI callback function
*******************************************************************************/
/**
 * @brief
 * \b 	Description:<br>
 * 	Override the weak function HAL_SPI_TxCpltCallback() in stm32l4xx_hal_spi.c
 * 	for SPI transfer by DMA. The Flush_Pending_Flag is reset after SRAM copy to GDDRAM
 * 	is complete by DMA transfer.
 */
#ifdef USE_SPI_DMA
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	fb_flush_pending_clear();
}
#endif

/**
 * @brief
 * \b		Description:<br>
 * 		Initialize the PMOLED module for display and touch.<br>
 * \b Pre-requisite:<br>
 * 		Run system reset and system clock before this function<br>
 *
 * \b Example:
 * @code
 * 		int main(void)
 * 		{
 * 			HAL_Init(); //system reset
 * 			SystemClock_Config(); //Configure the system clock
 * 			ssd7317_init();	//OLED display On after this function
 * 		}
 * @endcode
 */
void ssd7317_init(void){
	/*(1)Configure GPIO pins for display and touch interfaces*/
	MX_GPIO_Init();
	/*(2)Enable DMA for SPI*/
#ifdef USE_SPI_DMA
	MX_DMA_Init();
#endif
	/*(3)Initialize SPI interface for the display part*/
	MX_SPI1_Init();
	/*(4)Initialize I2C interface for touch*/
	MX_I2C1_Init();

	/*(6)Send initialization commands through SPI according to data sheet except 0xaf*/
	spi_write_command((const uint8_t*)SSD7317_INIT_TBL, sizeof(SSD7317_INIT_TBL));

	/*(7) Hw reset for touch screen*/
	HAL_GPIO_WritePin(TCH_TRES_GPIO_Port, TCH_TRES_Pin, GPIO_PIN_RESET);
	/*TRES set low for 1ms*/
	HAL_Delay(1);
	/*TRES pin set high for normal operation*/
	HAL_GPIO_WritePin(TCH_TRES_GPIO_Port, TCH_TRES_Pin, GPIO_PIN_SET);

	/*(8)Clear frame buffer and copy full screen to GDDRAM by non-DMA.
	 * No sync with FR should be implemented because SSD7317 not switched on yet -> no FR signal
	 * */
	rect_t screen = {0,0,OLED_HOR_RES-1,OLED_VER_RES-1};
	fb_clear(screen, BLACK);
	spi_write_data((const uint8_t *)&frame_buffer[0], (sizeof(frame_buffer)/sizeof(color_t)));

	/*(9)Display ON for OLED (12V VCC on and send over command 0xaf)*/
	ssd7317_display_on();

	/*(10)Initialization for the touch controller*/
	touch_init();
}

/**
 *@brief
 *\b	Description:<br>
 *	Switch OLED On with command 0xAF.
 */
void ssd7317_display_on(void){
	const uint8_t cmd[1]={0xaf};

	/*Software delay 1ms for VCC ramp up*/
	//HAL_Delay(1);

	/*Send display ON command*/
	spi_write_command((const uint8_t*)cmd, 1);
	/*Software delay 100ms to stabilize*/
	HAL_Delay(100);
}

/**
 *@brief
 *\b	Description:<br>
 *	Switch OLED off with command 0xAE. GDDRAM content is preserved.<br/>
 *	Better to use ssd7317_enter_lpm().
 */
void ssd7317_display_off(void){
	const uint8_t cmd[1]={0xae};

	/*Send display OFF command*/
	spi_write_command((const uint8_t*)cmd, 1);
}

/**
 * @brief
 * \b		Description:<br>
 * 		Clear PMOLED display with a color. First, the framebuffer is cleared for a color (BLACK/WHITE)
 * 		and then copied to GDDRAM on synchronization with FR signal to eliminate tearing effect.
 *
 * \b Example:
 * @code
 * 	int main(void)
 * 	{
 * 		HAL_Init(); //system reset
 * 		SystemClock_Config(); //Configure the system clock
 * 		ssd7317_init();	//OLED display On after this function
 * 		ssd7317_display_clear(BLACK);
 * 	}
 * @endcode
 */
void ssd7317_display_clear(color_t color){

	rect_t screen = {0,0,OLED_HOR_RES-1, OLED_VER_RES-1};

	fb_clear(screen, color);

	fb_flush_suspend();
	fb_flush_pending_set(screen);
}

/**
 * @brief
 * \b		Description:<br>
 * 		Set a pixel on PMOLED display with a color.<br>
 * @param	x is the x-coordinate
 * @param	y is the y-coordinate
 * @param	color is BLACK or WHITE
 *
 * \b Example:
 * @code
 * 	int main(void)
 * 	{
 * 	HAL_Init(); //system reset
 * 	SystemClock_Config(); //Configure the system clock
 * 	ssd7317_init();	//OLED display On with BLACK after this function
 * 	ssd7317_set_pixel(64/2, 128/2, WHITE);
 * 	}
 * @endcode
 */
void ssd7317_set_pixel(int16_t x, int16_t y, color_t color)
{
	if (x>(OLED_HOR_RES-1) || y>(OLED_VER_RES-1))
		return;

	fb_set_pixel(x,y,color);

	fb_flush_suspend();	    //wait until previous SPI flushes finished

	rect_t point = {x,y,x,y};
	fb_flush_pending_set(point);//set flag to indicate frame buffer flush pending and wait for a FR pulse
}


/**
 * @brief
 * \b	Description:<br>
 * 	Function to fill a region in the frame buffer with a pattern and update the GDDRAM on FR signal.<br>
 * @param area is the area to fill with (x1,y1) the top left and (x2,y2) the lower right corner (inclusive)
 * @param color is a pointer to the pattern in non-volatile media i.e. MCU's Flash
 * @param negative is a boolean parameter to display the image negative if it is 'true'
 *
 * \b Pre-requisite:x2>=x1 & y2>=y1<br>
 */
void ssd7317_fill_area(rect_t area, const color_t* color, bool negative)
{
	fb_fill_area(area,color,negative);

	fb_flush_suspend();	//wait until previous SPI flushes finished
	fb_flush_pending_set(area); //set flag to indicate frame buffer flush pending and wait for a FR pulse
}

/**
 * @brief
 * \b	Description:<br>
 * 	Function to fill a region in the frame buffer with a single color and update the GDDRAM on FR signal.<br>
 * @param area is the area to fill with (x1,y1) the top left and (x2,y2) the lower right corner (inclusive)
 * @param color is BLACK or WHITE
 */
void ssd7317_fill_color(rect_t area, color_t color)
{
	for(int16_t y = area.y1; y < area.y2+1; y++){
		for(int16_t x = area.x1; x < area.x2+1; x++)
			fb_set_pixel(x,y,color);
	}

	fb_flush_suspend();
	fb_flush_pending_set(area);
}

/**
 * @brief
 * \b	Description:<br>
 * 	Function to set display contrast with default level 0x4f<br>
 * @param level is the contrast level from 01 ~ FFh, a higher value sets higher contrast
 */
void ssd7317_set_contrast(uint8_t level)
{
	uint8_t cmd[2]={0x81, level};

	if(level==0) {cmd[1] = 0x01;}

	spi_write_command((const uint8_t*)cmd, 2);
}


/**
 * @brief
 * \b		Description:<br>
 * 		Function to return current screen orientation
 * @return 	local variable `scr_orientation`
 */
disp_orientation_t ssd7317_orientation_get()
{
	return scr_orientation;
}

/**
 * @brief
 * \b		Description:<br>
 * 		Function to set current screen orientation
 * @param 	Enumerated values DISP_0_DEG, DISP_90_DEG, DISP_180_DEG, DISP_270_DEG
 */
void ssd7317_orientation_set(disp_orientation_t rotation)
{
	scr_orientation = rotation;
}

/******************************************************************************
* Private functions
*******************************************************************************/
/**
 * @brief Function to return minimum value
 */
static inline int16_t min(int16_t a, int16_t b){
	if(a>b)
		return b;
	return a;
}
/**
 * @brief Function to return maximum value
 */
static inline int16_t max(int16_t a, int16_t b){
	if(a<b)
		return b;
	return a;
}

/**
  * @brief 	GPIO Initialization Function.
  * 		Configure all GPIO pins for display and touch interfaces.
  * 		Direct copy from STM32CubeIDE Code Generation utility.
  * @param 	None
  * @return None
  */
static void MX_GPIO_Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOA, TCH_TRES_Pin, GPIO_PIN_SET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin : OLED_FR_Pin */
	  GPIO_InitStruct.Pin = OLED_FR_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  HAL_GPIO_Init(OLED_FR_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : OLED_DC_Pin */
	  GPIO_InitStruct.Pin = OLED_DC_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(OLED_DC_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : TCH_TRES_Pin */
	  GPIO_InitStruct.Pin = TCH_TRES_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(TCH_TRES_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : TCH_IRQ_Pin */
	  GPIO_InitStruct.Pin = TCH_IRQ_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  HAL_GPIO_Init(TCH_IRQ_GPIO_Port, &GPIO_InitStruct);

	  /* EXTI interrupt init*/
	  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief Enable DMA controller clock
  */
#ifdef USE_SPI_DMA
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}
#endif


/**
  * @brief 	SPI1 Initialization Function.
  * 		Initialize SPI interface for the display part.
  * 		Direct copy from STM32CubeIDE Code Generation utility.
  * @param 	None
  * @return None
  */
static void MX_SPI1_Init(void)
{
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_1LINE;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
  * @brief 	I2C1 Initialization Function.
  *		Initialize I2C interface for touch.
  * 		Direct copy from STM32CubeIDE Code Generation utility.
  * @param 	None
  * @return 	None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
	  Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
	  Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
	  Error_Handler();
  }
}

/**
  * @brief 	SPI command write (non-DMA).
  * @param 	*command points to the command array to send.
  * @param	len is the data length in byte.
  * @return 	None
  */
void spi_write_command(const uint8_t *command, uint16_t len){

	/*DC pin set low for command send*/
	HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET);

	/*SPI send with non-DMA method = blocking transfer*/
	HAL_StatusTypeDef err = HAL_SPI_Transmit(&hspi1, (uint8_t *)command, len, 10);

	switch(err){
		case HAL_TIMEOUT:
		case HAL_ERROR:
			Error_Handler();
		break;
		default:
		break;
	}
}

/**
  * @brief 	SPI data write (non-DMA).
  * @param 	*data points to the data array to send.
  * @param	len is the data length in byte.
  * @return 	None
  */
void spi_write_data(const uint8_t *data, uint16_t len){

	/*DC pin set high for data send in next SPI transfer*/
	HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET);

	/*SPI send (non-DMA) = blocking function*/
	HAL_StatusTypeDef err = HAL_SPI_Transmit(&hspi1, (uint8_t *)data, len, 10);

	switch(err){
		case HAL_TIMEOUT:
		case HAL_ERROR:
			Error_Handler();
		break;
		default:
		break;
	}
}

/**
 * @brief Function to fill a pixel in the frame buffer. No actual OLED-write operation is involved.
 * @param x is the x-coordinate
 * @param y is the y-coordinate
 * @param color is WHITE/BLACK
 */
static void fb_set_pixel(int16_t x, int16_t y, color_t color)
{
	if(color==WHITE)
		frame_buffer[BUFIDX(x,y)] |= PIXIDX(x);
	else
		frame_buffer[BUFIDX(x,y)] &= ~PIXIDX(x);
}

/**
 * @brief
 * \b	Description:<br>
 * 	Function to fill an area in the frame buffer with a pattern from the Flash.
 *	No SPI transfer is called yet. Only frame buffer operation invoked.
 * @param area is the area to fill with (x1,y1) the top left and (x2,y2) the lower right corner (inclusive)
 * @param color is a pointer to the pattern in non-volatile media i.e. MCU's Flash
 */
static void fb_fill_area(rect_t area, const color_t* color, bool negative)
{
	uint8_t  byte_w = ((area.x2-area.x1+1) +7)>>3;
	uint8_t  pixel, bit_position;
	color_t _color;

	for(uint16_t y=area.y1; y<min(area.y2+1,OLED_VER_RES); y++){
		for(uint16_t x=area.x1; x<min(area.x2+1,OLED_HOR_RES); x++){
			pixel = color[((y-area.y1)*byte_w)+BUFIDX((x-area.x1),0)];
			bit_position = (x-area.x1)%8;
			pixel = pixel>>bit_position; //right shift to the lowest bit for comparison
			if(negative){
				(pixel&0x01)?(_color=BLACK):(_color=WHITE);
			}else{
				(pixel&0x01)?(_color=WHITE):(_color=BLACK);
			}
			fb_set_pixel(x,y,_color);
		}
	}
/*
	for(uint16_t y=0; y<(area.y2-area.y1+1); y++){
		for(uint16_t x=0; x<(area.x2-area.x1+1); x++){
			pixel = color[(y*byte_w) + BUFIDX(x,0)];
			//pixel = BIT_REVERSE(pixel); //if there is no software setup to flip the bit, uncomment this line
			bit_position = x%8;
			pixel = pixel>>bit_position; //right shift to the lowest bit for comparison

			//Image inverse can be implemented by swapping WHITE and BLACK
			if(negative){
				(pixel&0x01)?(_color=BLACK):(_color=WHITE);
			}else{
				(pixel&0x01)?(_color=WHITE):(_color=BLACK);
			}
				fb_set_pixel(area.x1+x, area.y1+y, _color);
		}
	}
*/
}

/**
  * @brief
  * \b	Description:<br>
  * 	Fill the frame buffer with a single color<br>
  * @param	area to fill in frame buffer
  * @param 	color_t color is WHITE or BLACK
  * @return None
  */
static void fb_clear(rect_t area, color_t color)
{
	color_t _color=0xff;

	if(color==BLACK){
		_color=0x00;
	}

	for(uint16_t y=area.y1; y<=area.y2; y++)
	{
		for(uint16_t x=area.x1; x<=area.x2; x+=8)
		{
			frame_buffer[BUFIDX(x,y)] = _color;
		}
	}
}

/**
  * @brief
  * \b	Description:<br>
  * 	Fill GDDRAM of OLED with frame buffer content by SPI transfers<br>
  * @param area is the content in frame buffer to copy to GDDRAM
  */
static void fb_spi_transfer(rect_t area)
{
	/*avoid running outside array index, may use assert here*/

	if(	area.y1>(OLED_VER_RES-1)|| area.y2>(OLED_VER_RES-1))
	{
#ifdef  USE_FULL_ASSERT
		assert_failed((uint8_t *)__FILE__,__LINE__);
#endif
		return;
	}

	/*Set segment address with command {0x21, start SEG, end SEG}*/
	uint8_t cmd[3] = {0x21, (uint8_t)area.y1, (uint8_t)area.y2};
	spi_write_command((const uint8_t *)&cmd, 3);

	/*Set page address with command {0x22, start PAGE=0, end PAGE=11(for 96*128 OLED resolution)}*/
	cmd[0] = 0x22;
	cmd[1] = BUFIDX(0,0);
	cmd[2] = BUFIDX((OLED_HOR_RES-1),0);
	spi_write_command((const uint8_t *)&cmd, 3);

	uint16_t length = (area.y2-area.y1+1)*(cmd[2]-cmd[1]+1);
	/*DC pin set high for data send in next SPI transfer*/
	HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET);

#ifdef USE_SPI_DMA
	/*DMA send = non blocking function*/
	HAL_StatusTypeDef err = HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)&frame_buffer[BUFIDX(0,area.y1)], length);
#else
	/*non-DMA SPI transfer, it is a blocking function*/
	HAL_StatusTypeDef err = HAL_SPI_Transmit(&hspi1, (uint8_t *)&frame_buffer[BUFIDX(0,area.y1)], length, 10);
	fb_flush_pending_clear();
#endif

	switch(err){
		case HAL_TIMEOUT:
		case HAL_ERROR:
			Error_Handler();
			break;
		default:
			break;
	}
}

/**
 *@brief
 *\b	Description:<br>
 *	Set flag to tell that GUI content to be flushed.
 *	Used in HAL_GPIO_EXTI_Callback().
 *@param area is the content in frame buffer to copy to GDDRAM
 */
static void fb_flush_pending_set(rect_t area)
{
	fb_flush_area = area;
	fb_flush_pending = true;
}

/**
 * @brief Wait until previous SPI transfer (if any) has finished
 */
static void fb_flush_suspend(void)
{
	uint8_t timeout = 20;
	while(fb_flush_pending_get())
	{
		if(timeout-- == 0)
		{
	#ifdef USE_FULL_ASSERT
				//Error with DMA Flag not protected well. Check timing.
				assert_failed((uint8_t *)__FILE__, __LINE__);
	#endif
				break;
		}
		HAL_Delay(1);
	}
}

/**
 * @brief
 * \b	Description:
 * 	Clear flush pending flag
 */
static void fb_flush_pending_clear(void)
{
	fb_flush_pending = false;
}

/**
 * @return Frame buffer flush pending flag
 */
static bool fb_flush_pending_get(void)
{
	return fb_flush_pending;
}

/**
 * @brief
 * \b	Description:<br>
 * 	I2C Write with commands and addresses in 16-bit width.<br>
 * 	Byte order: lower byte send first e.g. reg[7:0] follow by reg[15:8].<br>
 * @param slave is the 7-bit slave address
 * @param reg is the register address sending with LSB first i.e. 0xf0 sent first with reg=0x0af0
 * @param *data points to the data buffer to send
 * @param len is the byte count to send
 */
static void i2c_write(uint8_t slave, uint16_t reg, const uint8_t *data, uint16_t len)
{
	HAL_StatusTypeDef err;

	if(len){
		//swap high and low bytes so that lower byte is sent first
		uint16_t reg_byte_swap = ((reg<<8)&0xff00) | ((reg>>8)&0x00ff);
		err = HAL_I2C_Mem_Write(&hi2c1, slave<<1, reg_byte_swap, 2, (uint8_t *)data, len, 5000);
	}
	else
	{
		//[high:low] bytes of reg are swapped with HAL_I2C_Master_Transmit()
		//i.e. Byte 0x01 is sent first followed by 0x00 when reg=0x0001.
		//Example in i2c_write(TOUCH_SA, 0x0001, 0, 0)
		err = HAL_I2C_Master_Transmit(&hi2c1, slave<<1, (uint8_t *)&reg, 2, 500);
	}

	switch(err){
		case HAL_ERROR:
			Error_Handler();
			break;
			default:
			break;
		}
}

/**
 * @brief
 * \b	Description:<br>
 * 	I2C read from touch controller<br>
 * @param slave is the 7-bit slave address
 * @param reg is the register to read from
 * @param buffer points to an array to keep incoming characters
 * @param len is the byte count to read
 */
static void i2c_read(uint8_t slave, uint16_t reg, uint8_t *buffer, uint16_t len)
{
	HAL_StatusTypeDef err = HAL_I2C_Master_Transmit(&hi2c1, slave<<1, (uint8_t *)&reg, 2, 500);
	switch(err){
			case HAL_ERROR:
				Error_Handler();
				break;
				default:
				break;
			}

	HAL_DWT_Delay_us(200); //delay for 200us is mandatory for I2C Read

	err = HAL_I2C_Master_Receive(&hi2c1, (slave<<1)|0x01, buffer, len, 1000);

	switch(err){
		case HAL_ERROR:
			Error_Handler();
			break;
			default:
			break;
		}
}

/**
 * @brief
 * \b	Description:<br>
 * 	Do CRC checksum according to section 3.3.4 on datasheet<br>
 * @param byte_cnt is the size of PM_content, TM_content, & DM_content arrays defined in SSD7317_Init_table.c.
 * @param trig_cmd is the command to trigger a CRC read: PM trigger(0x03), DM trigger (0x05), TM trigger (0x09)
 * @return CRC value calculated by SSD7317
 */
static uint16_t touch_crc_checksum(uint16_t byte_cnt, uint8_t trig_cmd)
{
	uint16_t i2c_ret=0, expiry_ms=5; //preset 5ms for CRC calculation time

	//(1)Clear SA
	i2c_write(TOUCH_SA, 0x0000, 0, 0);

	//(2)Send CRC trigger command
	uint8_t data[3] = {(uint8_t)byte_cnt&0xff, (uint8_t)(byte_cnt>>8), trig_cmd};
	i2c_write(TOUCH_SA_BIOS, 0x8100, (const uint8_t *)&data, 3);

	//(3)Read S&L and wait until CRC is ready; otherwise, a timeout failure triggered
	while(i2c_ret!=0xcaf0){
		i2c_read(TOUCH_SA, 0x0af0, (uint8_t *)&i2c_ret, 2);
		HAL_Delay(1);
		if((expiry_ms--)==0){
#ifdef USE_FULL_ASSERT
			assert_failed((uint8_t *)__FILE__, __LINE__);
#endif
			return 0;
		}
	}

	//(4)Read CRC from 0x8100
	i2c_read(TOUCH_SA_BIOS, 0x8100, (uint8_t *)&i2c_ret, 2);

	return i2c_ret;
}

/**
 *@brief
 *\b	Description:<br>
 *	Initialize touch interface to implement "Reset and Boot" procedures as stated on Section 3<br>
 */
static void touch_init(void)
{
	uint16_t i2c_ret;

	//(0) Initialize micro-seconds software delay in STM32. A delay of 200us is mandatory in Touch_I2C_Read().
	if(HAL_DWT_Delay_Init()){
#ifdef USE_FULL_ASSERT
		assert_failed((uint8_t *)__FILE__, __LINE__);
#endif
	}

	//(1) IC Initialization with 0x0af0 (I2C send with lower byte first)
	i2c_read(TOUCH_SA, 0x0af0, (uint8_t *)&i2c_ret, 2);
	if(i2c_ret!=0xcaf0){
#ifdef USE_FULL_ASSERT
		assert_failed((uint8_t *)__FILE__, __LINE__);
#endif
		return;
	}

	//(2) Send byte sequence for PM-select for 1228 bytes in 24 blocks of 512 bytes each
	//PM Select
	i2c_write(TOUCH_SA, 0x0001, 0, 0);
	for(uint8_t n=0; n<24; n++){
		i2c_write(TOUCH_SA_BIOS, 2*n, (const uint8_t *)&FW_PM.content[n*512], 512);
	}

	//(3) Send byte sequence for TM-select 1844 bytes,
	i2c_write(TOUCH_SA, 0x0003, 0, 0);
	i2c_write(TOUCH_SA_BIOS, 0, (const uint8_t *)&FW_TM.content[0], 512);
	i2c_write(TOUCH_SA_BIOS, 0x0002, (const uint8_t *)&FW_TM.content[512], 512);
	i2c_write(TOUCH_SA_BIOS, 0x0004, (const uint8_t *)&FW_TM.content[1024], 512);
	i2c_write(TOUCH_SA_BIOS, 0x0006, (const uint8_t *)&FW_TM.content[1536], 308); //send remainder bytes

	//(4) Send byte sequence for DM-select for 2048 bytes
	i2c_write(TOUCH_SA, 0x0002, 0, 0);
	i2c_write(TOUCH_SA_BIOS, 0, (const uint8_t *)&FW_DM.content[0], 512);
	i2c_write(TOUCH_SA_BIOS, 0x0002, (const uint8_t *)&FW_DM.content[512], 512);
	i2c_write(TOUCH_SA_BIOS, 0x0004, (const uint8_t *)&FW_DM.content[1024], 512);
	i2c_write(TOUCH_SA_BIOS, 0x0006, (const uint8_t *)&FW_DM.content[1536], 512);

	//Do checksum for PM Trig
	if(touch_crc_checksum(FW_PM.byte_cnt, 0x03)!=FW_PM.crc){
#ifdef USE_FULL_ASSERT
			assert_failed((uint8_t *)__FILE__, __LINE__);
#endif
	}
	//Do checksum for TM Trig
	if(touch_crc_checksum(FW_TM.byte_cnt, 0x09)!=FW_TM.crc){
#ifdef USE_FULL_ASSERT
			assert_failed((uint8_t *)__FILE__, __LINE__);
#endif
	}
	//Do checksum for DM Trig
	if(touch_crc_checksum(FW_DM.byte_cnt, 0x05)!=FW_DM.crc){
#ifdef USE_FULL_ASSERT
			assert_failed((uint8_t *)__FILE__, __LINE__);
#endif
	}

	//(5) MCU Un-stall(means passing control to MCU)
	i2c_write(TOUCH_SA, 0x0000, 0, 0);
	uint8_t data[2]={0,0};
	i2c_write(TOUCH_SA_BIOS, 0x8300, (const uint8_t *)&data, 2);
	data[0]=0x03;
	i2c_write(TOUCH_SA_BIOS, 0x8000, (const uint8_t *)&data, 2);
	data[0]=0x00;
	i2c_write(TOUCH_SA_BIOS, 0x8000, (const uint8_t *)&data, 2);

	//(6) After MCU Un-stall, wait 5ms
	HAL_Delay(5);

	//(7) Send display command 0xf4, 0x90 for initialization complete
	data[0]=0xf4;
	data[1]=0x90;
	spi_write_command((const uint8_t*)&data, 2);

	//(8) Clear interrupt
	touch_event_clear();
}

/**
 * @brief
 * \b		Description:<br>
 * 		Reset OLED's IRQ pin and clear touch event flag for local variable 'Touch_Evt_Flag'<br>
 * \b Note:<br>
 * 		There is a typo error on section 3.3.6. <br>
 * 		The correct sequence should be 0x43 0x00 0x00 0x00.<br>
 */
static void touch_event_clear(void)
{
	uint16_t cmd=0; //sending two bytes of 0x00

	i2c_write(TOUCH_SA, 0x0043, (const uint8_t *)&cmd, 2);

	__disable_irq();
	touch_event_flag = false;
	__enable_irq();

}

/**
 * @brief
 * \b		Description:<br>
 * 		Set IRQ flag in IRQ handler for a high-to-low IRQ pin transition triggered by a touch event
 */
void touch_event_set(void)
{
	touch_event_flag = true;
}

/**
 * @brief
 * \b		Description:<br>
 * 		Get touch event flag<br>
 * @return 	true for a touch event
 * 		false for no touch event
 */
inline static bool touch_event_get(void)
{
	return touch_event_flag;
}

/**
 * @brief
 * \b Description:<br>
 * Function to get touch gesture.<br>
 * @return finger_t structure with gesture and key number
 */
finger_t ssd7317_get_gesture(void){
	finger_t finger = {0, 0, ACT_ERROR, DETAIL_ERROR};

	if(touch_event_get()){
		uint16_t status;
		/*gesture_upload[0]=Gesture ACT, gesture_upload[1]=Gesture Detail, gesture_upload[2]=KeyNum[6:4]KeyNum[2:0]*/
		uint8_t gesture_upload[6];

		i2c_read(TOUCH_SA, 0x0af0, (uint8_t *)&status, 2);
		/*Lower byte of status is the byte count to read from 0x0af1 register*/
		if((status&0xff)>0){
			i2c_read(TOUCH_SA, 0x0af1, (uint8_t *)&gesture_upload, status&0xff);
		}
#ifdef USE_FULL_ASSERT
		/**
		 * Example:
		 * SWIPE_UP:	f6, 4, 2, ca, 0, 0
		 * SWIPE_DOWN:	f6, 4, 1, 9c, 0, 0
		 * SWIPE_LR: 	f6, 4, 1, b3, 0, 1
		 * SWIPE_RL: 	f6, 4, 2, bb, 0, 1
		 */
		printf("Gesture data read from 0x0AF1 in hex %x, %x, %x, %x, %x, %x \r\n",
				gesture_upload[0], gesture_upload[1], gesture_upload[2], gesture_upload[3], gesture_upload[4], gesture_upload[5]);
#endif
		touch_event_clear();

		switch(gesture_upload[1]){
		case 0x01:
			finger.act = SINGLE_TAP_ANYKEY;
			if(gesture_upload[2]==1)
				finger.detail = SINGLE_TAP_KEY1;
			else if (gesture_upload[2]==2)
				finger.detail = SINGLE_TAP_KEY2;
			else if (gesture_upload[2]==3)
				finger.detail = SINGLE_TAP_KEY3;
			else if (gesture_upload[2]==4)
				finger.detail = SINGLE_TAP_KEY4;
			else
				finger.detail = SINGLE_TAP_ANYKEY_DETAIL;
			break;
		case 0x02:
			finger.act = LONG_TAP_ANYKEY;
			if(gesture_upload[2]==1)
				finger.detail = LONG_TAP_KEY1;
			else if (gesture_upload[2]==2)
				finger.detail = LONG_TAP_KEY2;
			else if (gesture_upload[2]==3)
				finger.detail = LONG_TAP_KEY3;
			else if (gesture_upload[2]==4)
				finger.detail = LONG_TAP_KEY4;
			else
				finger.detail = LONG_TAP_ANYKEY_DETAIL;
			break;
		case 0x03:
			finger.act = DOUBLE_TAP_ANYKEY;
			if(gesture_upload[2]==1)
				finger.detail = DOUBLE_TAP_KEY1;
			else if (gesture_upload[2]==2)
				finger.detail = DOUBLE_TAP_KEY2;
			else if (gesture_upload[2]==3)
				finger.detail = DOUBLE_TAP_KEY3;
			else if (gesture_upload[2]==4)
				finger.detail = DOUBLE_TAP_KEY4;
			else
				finger.detail = DOUBLE_TAP_ANYKEY_DETAIL;
			break;
		case 0x04:
			finger.act = SWIPE;
			if(gesture_upload[2]==1){
				finger.detail = SWIPE_LR;
			}else if(gesture_upload[2]==2){
				finger.detail = SWIPE_RL;
			}
			finger.tap_down_key = (gesture_upload[3]>>4)&0x07;
			finger.tap_up_key = gesture_upload[3]&0x07;
			break;
		case 0x40:
			finger.act = LARGE_OBJ;
			if(gesture_upload[2]==1)
				finger.detail = LARGE_OBJ_DETECT;
			else
				finger.detail = LARGE_OBJ_RELEASE;
			break;
		case 0xFF:
			finger.act = ACT_ERROR;
			finger.detail = DETAIL_ERROR;
			break;
		}
	}
	return finger;
}

/**
 * @brief
 *  \b Description:<br>
 *  		Function to enter Low Power Mode (LPM) with VCC discharged and display goes to sleep.
 *  		Send ssd7317_display_on() to wake up after this function.
 */
void ssd7317_enter_lpm(void)
{
	uint8_t data[2] = {0x01, 0x00};
	i2c_write(TOUCH_SA, 0x0037, (const uint8_t *)&data, 2);
	HAL_Delay(150);	//VCC discharge time

	data[0]=0xae;	//send display off (0xae) SPI command

	/*Send display OFF command*/
	spi_write_command((const uint8_t*)data, 1);
}

/**
 * @brief
 *  \b Description:<br>
 *  		Function to draw an image of tImage type on OLED
 * @param 	left is the x coordinate of the image's top left
 * @param 	top is the y coordinate of the image's top left
 * @param 	*image is a pointer to tImage structure
 * @param 	negative is a boolean flag to invert the image if this parameter is 1 or 'true'
 * @return 	box bounding the image drawn
 *
 * \b Example:
 * @code
 *			#include "icon_swimming.h" //this file contains the data
 *
 * 			int main(void)
 * 			{
 * 				HAL_Init(); //system reset
 * 				SystemClock_Config(); //Configure the system clock
 * 				ssd7317_init();	//OLED display On after this function
 * 				sys_put_image(0, 12, &icon_swimming, 0);
 *
 * 				while(1){
 * 					;//your task here...
 * 				}
 * 			}
 * @endcode
 */
rect_t ssd7317_put_image(uint16_t left, uint16_t top, const tImage* image, bool negative)
{
	rect_t area={left,top,(left+image->width-1),(top+image->height-1)};

	ssd7317_fill_area(area, image->data, negative);

	return area;
}

/**
 * @brief
 *  \b Description:<br>
 *  		Function to draw an image of tImage type on OLED without frame buffer operation.<br/>
 *  		Pixels from FLASH is copied directly to GDDRAM of the OLED.
 * @param 	left is the x coordinate of the image's top left in pixel.
 * @param 	top is the y coordinate of the image's top left in pixel.
 * @note	This value can be negative particularly designed for swipe up/down event with an icon scrolling off-screen.
 * @param 	*image is a pointer to tImage structure
 * @return 	None
 * @note	Need to apply a trick if we want to scroll an icon without messing up the screen -
 * pad at least one blank line (BLACK) each at the top and bottom of the icon to reset the dirty area
 * when the icon moves across.
 *
 *
 * \b Example:
 * @code
 *		#include "icon_swimming.h" //this file contains the data
 *
 * 		int main(void)
 * 		{
 * 		HAL_Init(); //system reset
 * 		SystemClock_Config(); //Configure the system clock
 * 		ssd7317_init();	//OLED display On after this function
 *
 * 		while(1){
 *
 * 		int y; //a negative value is allowed to show an icon scrolling off-screen
 *
 * 		for(y=-64; y<193; y++){
 * 		ssd7317_put_image_direct((96-64)/2,y,&icon_swimming);
 * 		HAL_Delay(2);
 * 		}
 *
 * 		for(y=192; y>-65; y--){
 * 		ssd7317_put_image_direct((96-64)/2,y,&icon_swimming);
 * 		HAL_Delay(2);
 * 		}
 * 		}
 * 		}
 * @endcode
 */
void ssd7317_put_image_direct(uint16_t left, int16_t top, const tImage* image)
{
	if (left+image->width > OLED_HOR_RES){
#ifdef  USE_FULL_ASSERT
		//image out of bound
		assert_failed((uint8_t *)__FILE__, __LINE__);
#endif
		return;
	}

	uint16_t top_margin = 0;
	uint16_t bottom_margin = 0;

	if(top < 0){
		//bound the top to the image height above OLED's y=0
		top_margin = min(-top, image->height);
		top = 0;
	}

	if((top+image->height) > OLED_VER_RES){
		//bound the bottom to the image height below OLED's y=OLED_VER_RES
		bottom_margin = min(top+image->height-OLED_VER_RES, image->height);
	}

	uint8_t cmd[6];

	cmd[0] = 0x21; //set column address
	cmd[1] = top;
	cmd[2] = min(top+image->height-1, OLED_VER_RES-1);
	cmd[3] = 0x22; //set page address
	cmd[4] = left>>3;
	cmd[5] = min((left>>3)+((image->width+7)>>3)-1, (OLED_HOR_RES>>3)-1);

	spi_write_command((const uint8_t*)cmd, 6); //SPI write command for the rectangle that bounds the image

	HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET);//DC pin set high for data

	const uint8_t *px = image->data;
	uint16_t byte_len = (image->height - top_margin - bottom_margin)*(image->width>>3);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&px[((image->width)>>3)*top_margin], byte_len, 10);
}

/**
 * @brief
 * \b		Description:<br/>
 * 		Function to scroll an image from FLASH with content scroll command 2Ch/2Dh.<br/>
 * 		This function is valid for COM-page H mode only.
 * @param	left is the top left position of the image to scroll in pixel, only valid in a multiple of 8.<br/>
 * @param	top is the start column(segment), it is also the top segment address in native orientation of the OLED.
 * @param 	*image is a pointer to tImage structure.
 * @param	dir is the swipe direction, either SWIPE_UP(SWIPE_RL) or SWIPE_DOWN(SWIPE_LR).
 * @note	end_col should be larger than start_col; else, the function will swap it for you.
 * Scroll direction is controlled by dir.detail==SWIPE_DOWN / SWIPE_UP, not by end_col and start_col pair.
 */
void   ssd7317_cntnt_scroll_image(uint16_t left, uint16_t top, const tImage* image, finger_t dir)
{
	if((dir.detail!=SWIPE_DOWN) && (dir.detail!=SWIPE_UP))
	{
#ifdef  USE_FULL_ASSERT
		assert_failed((uint8_t *)__FILE__, __LINE__);
#endif
		return;
	}

	uint8_t page_start = 0;

	if(left > (OLED_HOR_RES-1)){
#ifdef  USE_FULL_ASSERT
		assert_failed((uint8_t *)__FILE__, __LINE__);
#endif
		return;
	}else{
		page_start = (uint8_t)(left>>3); //left in pixel, we need to find the page address for SSD7317
	}

	uint8_t cmd[8]; //command sequence to send to OLED to scroll an image

	uint16_t _end_col =  min(top+image->height-1, OLED_VER_RES-1);

	const uint8_t *px = image->data;
	uint16_t byte_w = min((OLED_HOR_RES>>3)-page_start, (image->width)>>3);

	uint8_t pad[byte_w]; //pad is useful when scrolling height is larger than the image height
	memset((uint8_t *)&pad, BLACK, byte_w);

	for(uint16_t col=0; col<image->height; col++)
	{
		/* Area setup for the line to write */
		cmd[0]=0x21;
		if(dir.detail==SWIPE_DOWN){
			cmd[1]=top; //only a line is written in scrolling therefore cmd[1]=cmd[2]
			cmd[2]=top; //on SWIPE_DOWN, the top line is written
		}else{
			cmd[1]=_end_col; //on SWIPE_UP, the bottom line is written
			cmd[2]=_end_col;
		}

		cmd[3]=0x22;
		cmd[4]=page_start;
		cmd[5]=page_start+(uint8_t)byte_w-1;
		spi_write_command((const uint8_t*)cmd, 6); //define the line to write data from the bottom

		HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET);//DC pin set high for data

		/* Write pixels from FLASH to GDDRAM */
		if(dir.detail==SWIPE_DOWN){
			HAL_SPI_Transmit(&hspi1, (uint8_t *)&px[(image->height-1-col)*(image->width>>3)], byte_w, 10);
		}else{
			HAL_SPI_Transmit(&hspi1, (uint8_t *)&px[col*(image->width>>3)], byte_w, 10);
		}

		/* Scroll the area */
		(dir.detail==SWIPE_DOWN)?(cmd[0]=0x2c):(cmd[0]=0x2d); //content scrolling command
		cmd[1]=0; 			//dummy byte
		cmd[2]=page_start;	//left page margin
		cmd[3]=0x01; 		//no wrap around of RAM content
		cmd[4]=page_start+(uint8_t)byte_w-1;
		cmd[5]=0;			//another dummy byte
		cmd[6]=top;
		cmd[7]=_end_col;
		spi_write_command((const uint8_t*)cmd, 8); //scroll it

		HAL_Delay(15); //Data sheet tells us a delay of 20ms is required. Actual experiments show 15ms is also good.
	}

	ssd7317_put_image(page_start<<3, top, image, 0); //Update the frame buffer after scrolling
}

/**
 * @brief
 * \b Description:<br/>
 * This function is the counterpart of ssd7317_cntnt_image() which scrolling effect is done by graphic commands.
 * @param	left is the top left position of the image to scroll in pixel, only valid in a multiple of 8.<br/>
 * @param	top is the start column(segment), it is also the top segment address in native orientation of the OLED.
 * @param	tick is the delay between successive line update of the image in millisecond precision.
 * @param 	*image is a pointer to tImage structure.
 * @param	dir is the swipe direction, either SWIPE_UP(SWIPE_RL) or SWIPE_DOWN(SWIPE_LR).
 * @note	In this function, scrolling is done in the frame buffer. It turns out that a delay of 15ms is not required and
 * a much better performance is achieved. Parameter end_col should be larger than start_col; else, the function will swap it for you.
 * Scroll direction is controlled by dir.detail==SWIPE_DOWN / SWIPE_UP, not by end_col and start_col pair.
 * \b Example:
 * @code
 * 		int main(void)
 * 		{
 * 			HAL_Init(); //system reset
 * 			SystemClock_Config(); //Configure the system clock
 * 			ssd7317_init();	//OLED display On after this function
 * 			//...
 * 			static uint8_t icon_index=0;
 * 			finger_t gesture;
 * 			while(1)
 * 			{
 * 			switch(gesture.act){
 * 			case SWIPE:
 * 			(finger.detail==SWIPE_DOWN)?(icon_index++):(icon_index--);
 * 			//assume there are icons of 64x64 pixels declared as tIcon icons[]
 * 			ssd7317_cntnt_fbscroll_image(16,64,1,icons[icon_index].image,finger);
 * 			break;
 * 			}
 * 			}
 * 		}
 * @endcode
 */
void   ssd7317_cntnt_fbscroll_image(uint16_t left, uint16_t top, uint8_t tick, const tImage* image, finger_t dir)
{
	if((dir.detail!=SWIPE_DOWN) && (dir.detail!=SWIPE_UP))
	{
#ifdef  USE_FULL_ASSERT
		assert_failed((uint8_t *)__FILE__, __LINE__);
#endif
		return;
	}

	if(left > (OLED_HOR_RES-1)){
#ifdef  USE_FULL_ASSERT
		assert_failed((uint8_t *)__FILE__, __LINE__);
#endif
		return;
	}

	const uint8_t *px;// = image->data;

	rect_t area = {left, top, left+image->height-1, top+image->height-1};

	for(uint16_t col=0; col<image->height; col++){
		color_t *fb = fb_scroll_segment(area, dir);
		(dir.detail==SWIPE_DOWN)?(px = &image->data[(image->height-1-col)*(image->width>>3)]):(px = &image->data[col*(image->width>>3)]);
		uint16_t byte_w = min(((OLED_HOR_RES>>3)-(left>>3)), (image->width)>>3);
		while(byte_w--)
			*fb++ = *px++;
		if(tick>0)
			HAL_Delay(tick);
		//fb_flush_suspend();
		fb_flush_pending_set(area);
	}
}

/**
 * @brief
 * \b Description:<br>
 * Function to continuously scroll the screen content<br/>
 * This function is valid for COM-page H mode only and the hardware-specific commands 26h/27h/29h/2Ah.
 * @param subpage defines the start page and end page address to scroll
 * @param interval sets time interval between each scroll step in terms of frame frequency from 0-7<br/>
 * 0(6 frames), 1(32 frames), 2(64 frames), 3(128frames), 4(3 frames), 5(4 frames), 6(5 frames), 7(2 frames)
 * @param accelerate is the scrolling offset from 1 row to 95 rows
 * @param dir is the SWIPE direction (SWIPE_UP or SWIPE_DOWN)
 * @note This function comes from part of the Advanced Graphic Acceleration Command set released by Solomon Systech.<br/>
 * There are eight consecutive bytes to setup the scrolling parameters including start page, end page, start segment, end segment,
 * the command itself and some dummy bytes. There is no frame buffer operation with this function.
 */
void   ssd7317_cons_scroll_page(rect_t subpage, uint8_t interval, uint8_t accelerate, finger_t dir)
{
	if((dir.detail!=SWIPE_DOWN) && (dir.detail!=SWIPE_UP))
	{
#ifdef  USE_FULL_ASSERT
		assert_failed((uint8_t *)__FILE__, __LINE__);
#endif
		return;
	}

	if((subpage.x1>subpage.x2) || (subpage.y1>subpage.y2))
	{
#ifdef  USE_FULL_ASSERT
		assert_failed((uint8_t *)__FILE__, __LINE__);
#endif
		return;
	}

	uint8_t cmd[9]={0};

	if(dir.detail==SWIPE_UP){
		(dir.tap_down_key==dir.tap_up_key)?(cmd[0] = 0x2a):(cmd[0] = 0x27);
	}else{
	 //dir.gesture==SWIPE_DOWN
		(dir.tap_down_key==dir.tap_up_key)?(cmd[0] = 0x29):(cmd[0] = 0x26);
	}

	//cmd[1] = 0x00 assuming no diagonal scrolling for command 29/2a

	//start page pixel
	uint16_t page_px = min(OLED_HOR_RES-1,subpage.x1);

	cmd[2] = BUFIDX(page_px,0); //start page address in page number

	cmd[3] = (interval&0x07);	//interval in frame frequency

	//end page pixel
	page_px = min(OLED_HOR_RES-1,subpage.x2);
	cmd[4] = BUFIDX(page_px,0);	//end page address in page number

	if((cmd[0]==0x29) || (cmd[0]==0x2a)){
		cmd[5] = ((accelerate|0x01)&0x5F); //bound acceleration within 1~95 rows offset
	}

	cmd[6] = subpage.y1;
	cmd[7] = subpage.y2&(OLED_VER_RES-1);
	cmd[8] = 0x2f; //activate scrolling
	spi_write_command((const uint8_t *)&cmd, 9);
}

/**
 * @brief
 * \b Description:<br>
 * This function applies brake to continuous page scroll command `ssd7317_cont_scroll_page()` with 0x2E
 */
void   ssd7317_cons_scroll_brake(void)
{
	const uint8_t cmd[1]={0x2e};

	spi_write_command((const uint8_t *)&cmd, 1);
}

/**
 * @brief
 * \b Description:<br>
 */
rect_t ssd7317_put_char(uint16_t left, uint16_t top, const tFont* font, uint16_t code, bool negative)
{
	const tChar* pChar = font->chars;

	rect_t area = {0,0,0,0};

	bool in_range = false; //assume char code not in range first
	int len = font->length;//scan the whole array length of tFont in do-while loop below
	int i = 0;

	do{
		if(code == pChar[i].code){
			in_range = true;
			break;
		}
		i++;
	}while(len--);

	if(!in_range){
#ifdef USE_FULL_ASSERT
			assert_failed((uint8_t *)__FILE__, __LINE__); //character out of range
#endif
			return area;
	}

	if(left+pChar[i].image->width > OLED_HOR_RES){
#ifdef USE_FULL_ASSERT
			assert_failed((uint8_t *)__FILE__, __LINE__); //character out of range
#endif
			return area;
	}

	const uint8_t* pData = pChar[i].image->data;

	area.x1 = left;
	area.y1 = min(top, OLED_VER_RES-1);
	area.x2 = (left+pChar[i].image->width-1);
	area.y2 = min((top+pChar[i].image->height-1), OLED_VER_RES-1);

	ssd7317_fill_area(area, pData, negative);

	return area;
}

/**
 * @brief
 * \b Description:<br>
 */
void   ssd7317_get_charsize(const tFont* font, uint16_t ascii_code, uint16_t *w, uint16_t *h)
{
	*w=0; *h=0;
	const tChar* pChar = font->chars;

	if(pChar!=0){
		uint16_t _code = ascii_code-pChar[0].code; //need to offset the first array member
		*w = pChar[_code].image->width;
		*h = pChar[_code].image->height;
	}else{
#ifdef USE_FULL_ASSERT
			assert_failed((uint8_t *)__FILE__, __LINE__);
#endif
	}
}

/**
 * @brief
 * \b Description:<br>
 */
rect_t ssd7317_put_string(uint16_t left, uint16_t top, const tFont* font, const char *str, bool negative)
{
	rect_t err={0,0,0,0};

	/* make sure *font and *str are not NULL pointer */
	if(font==0 || str==0){
#ifdef USE_FULL_ASSERT
			assert_failed((uint8_t *)__FILE__, __LINE__);
#endif
			return err;
	}

	if((left>OLED_HOR_RES-1) || (top>OLED_VER_RES-1)){
#ifdef USE_FULL_ASSERT
			assert_failed((uint8_t *)__FILE__, __LINE__);
#endif
			return err;
	}

	const tChar* pChar = font->chars;
	const char *pStr = str;
	uint16_t array_idx[strlen(str)];
	uint16_t idx = 0;

	/* first, make sure all characters in str can be found in tFont*/
	while(*pStr!='\0'){

		int len = font->length;
		int i = 0;
		do{
			if(*pStr==pChar[i].code){
				array_idx[idx++]=i; //store the array index of tFont in array_idx
				break; //found the matching character
			}
			i++;
		}while(len--);

		if(len < 0){
		/* there is a character not found in tFont */
#ifdef USE_FULL_ASSERT
				assert_failed((uint8_t *)__FILE__, __LINE__);
#endif
			return err;
		}
		pStr++;
	}

	/* Now, data assert all pass. We can get each char from the string and put it in frame buffer */
	rect_t area= {left,top,left,min(top + (font->chars->image->height-1),OLED_VER_RES-1)};

	uint16_t _x = left;

	for(idx = 0; idx < strlen(str); idx++){
		const uint8_t* pData = pChar[array_idx[idx]].image->data;
		_x += (pChar[array_idx[idx]].image->width);
		if(_x > OLED_HOR_RES) break;
		area.x2 = _x-1;
		fb_fill_area(area,pData,negative);
		area.x1 = _x; //increment for new area.x1
	}
	area.x1 = left;

	fb_flush_suspend();	//wait until previous SPI flushes finished
	fb_flush_pending_set(area); //set flag to indicate frame buffer flush pending and wait for a FR pulse

	return area;
}

/**
 * @brief
 * \b Description:<br>
 */
void   ssd7317_get_stringsize(const tFont* font, const char *str, uint16_t *w, uint16_t *h)
{
	if((font==0) || (str==0)){
#ifdef USE_FULL_ASSERT
			assert_failed((uint8_t *)__FILE__, __LINE__);
			return;
#endif
	}

	uint16_t _w, _h, _x = 0;

	while(*str != '\0'){
		ssd7317_get_charsize(font, (const uint16_t)*str++, &_w, &_h);
		_x += _w;
	}

	*w = _x;
	*h = _h;
}

/**
 * @brief Scroll memory by one segment
 * @param area defines to area to scroll
 * @param gesture supports only SWIPE_UP or SWIPE_DOWN
 * @return pointer to the segment that scrolled out.
 * It can be filled with new data from FLASH for a new image scrolled in.
 * \b Example:
 * @code
 * 		int main(void)
 * 		{
 * 			HAL_Init(); //system reset
 * 			SystemClock_Config(); //Configure the system clock
 * 			ssd7317_init();	//OLED display On after this function
 * 			//...
 * 			rect_t image={16,64,16+63,127}; //image of 64*64 pixels w/ top left (16,64)
 * 			finger_t gesture;
 * 			//On an event = gesture.SWIPE_UP
 * 			uint8_t len=64;
 * 			while(len--){
 * 			 color_t *segment = fb_scroll_segment(image, &frame_buffer[0], gesture);
 * 			 //memset() fills the scrolled area in black -
 * 			 //it can be replaced by a new image copy to the frame buffer line-by-line
 * 			 memset(segment, BLACK, 8);
 * 			 HAL_Delay(1);
 * 			 fb_flush_pending_set(image);
 * 			}
 * 		}
 * @endcode
 */
static color_t *fb_scroll_segment(rect_t area, finger_t gesture)
{
	uint16_t x1=min(area.x1,OLED_HOR_RES-1);
	uint16_t x2=min(area.x2,OLED_HOR_RES-1);
	uint16_t y1=min(area.y1,OLED_VER_RES-1);
	uint16_t y2=min(area.y2,OLED_VER_RES-1);

	x1=max(x1,0);
	x2=max(x2,0);
	y1=max(y1,0);
	y2=max(y2,0);

	if(y1>y2){
		uint16_t _tmp=y2;
		y2=y1;
		y1=_tmp;
	}

	uint8_t byte_w;
	uint8_t height = y2 - y1 + 1;

	color_t *pReturn, *pDest, *pSrc;

	if(gesture.detail==SWIPE_UP){
		/* Data append from below */
		pReturn = &frame_buffer[BUFIDX(x1,y2)];
		for(uint16_t y=0; y<(height-1); y++)
		{
			byte_w=BUFIDX(x2,0)-BUFIDX(x1,0)+1;
			pDest = &frame_buffer[BUFIDX(x1, (y1+y))];
			pSrc  = &frame_buffer[BUFIDX(x1, (y1+y+1))]; //mind the bracket (y1+y+1) inside BUFIDX. It is mandatory
			while(byte_w--)
				*pDest++ = *pSrc++;
		}
	}
	else if(gesture.detail==SWIPE_DOWN)
	{ 	/* Data append over the top */
		pReturn = &frame_buffer[BUFIDX(x1,y1)];
		for(uint16_t y=0; y<(height-1); y++)
		{
			byte_w=BUFIDX(x2,0)-BUFIDX(x1,0)+1;
			pDest = &frame_buffer[BUFIDX(x1,(y2-y))];
			pSrc  = &frame_buffer[BUFIDX(x1,(y2-y-1))];
			while(byte_w--)
				*pDest++ = *pSrc++;
		}
	}
	else
	{
	//...
	}
	//HAL_Delay(1);
	//fb_flush_pending_set(area);

	/*
	 * fb_flush_pending_set(area) here for testing only.
	 * The flush_pending flag should be set after new image content copied to frame buffer.
	 *
	 */
	return pReturn;
}

