# ILI9341 DMA driver library for STM32 HAL
For 320x240 SPI LCD boards based on ILI9341 driver chip.

## How to use this library
* Generate SPI peripheral and 3 GPIO outputs
	* STM32Cube code generation: separate .c/.h files for each peripheral
	* SPI peripheral: CPOL=LOW, CPHA=1EDGE, 8 bits, MSB first, Software NSS, up to 50 MHz
	* Tx DMA for SPI: Half Word (16 bits) data width
	* GPIO: GPIO_SPEED_FREQ_VERY_HIGH for CS and DC pins
	* GPIO initial values: RST=Low, CS=High - LCD is in reset state before ILI_Init()
* Configure parameters in ILI9341_Driver.h:
	* Define your ILI_SPI_HANDLE
	* Define your CS, DC and RST outputs
	* Check if ILI_SCREEN_WIDTH and ILI_SCREEN_HEIGHT match your LCD size and orientation.
* In your main program initialize LCD with ILI_Init() before use

## Code example
	/* Includes */
	#include "ILI9341_DMA_Driver.h"
	/* Global vars */
	volatile uint16_t display_buf[ILI_SCREEN_WIDTH*ILI_SCREEN_HEIGHT] __ALIGNED(32);	// aligned for DCache
	/* Interrupt callback */
	void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
		if (hspi->Instance == ILI_SPI_HANDLE.Instance) {
			ILI_DMA_Callback();
		}
	}
	/* Main program init section */
	ILI_Init();
	/* Draw something */
	uint32_t i;
	for (i = 0; i < (ILI_SCREEN_WIDTH*ILI_SCREEN_HEIGHT); i++) {
	  if (i < ILI_SCREEN_HEIGHT/4*ILI_SCREEN_WIDTH) {
		  display_buf[i] = RED;
	  } else if (i < ILI_SCREEN_HEIGHT/2*ILI_SCREEN_WIDTH) {
		  display_buf[i] = GREEN;
	  } else if (i < ILI_SCREEN_HEIGHT/4*3*ILI_SCREEN_WIDTH) {
		  display_buf[i] = BLUE;
	  } else {
		  display_buf[i] = YELLOW;
	  }
	}
	/* Load frame buffer */
	ILI_DMA_Load((uint16_t *)display_buf, (ILI_SCREEN_WIDTH*ILI_SCREEN_HEIGHT));
