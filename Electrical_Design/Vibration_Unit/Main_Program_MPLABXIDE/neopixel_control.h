/* 
 * File:   neopixel_control.h
 * Author: bingjian
 *
 * Created on January 7, 2025, 3:38 PM
 */

#ifndef NEOPIXEL_CONTROL_H
#define	NEOPIXEL_CONTROL_H

#include <xc.h> // or other necessary headers
// We define the 3-bit patterns for '0' and '1'.
#define WS2812_0 0b100  // ~ 0.42 us high, 0.83 us low
#define WS2812_1 0b110  // ~ 0.83 us high, 0.42 us low
#define NUM_BYTES_PER_COLOR 4    // 3 encoded bytes per one color byte
#define _XTAL_FREQ 32000000  // 32 MHz for __delay_us()

void SPI_Init(void);
void encodeByte(uint8_t inputByte, uint8_t outArray[4]);
void sendColor_SPI(uint8_t g, uint8_t r, uint8_t b);

#endif	/* NEOPIXEL_CONTROL_H */

