#include "neopixel_control.h"

void SPI_Init(void)
{
    // Example: MSSP1 in SPI Master mode
    // SCK = Fosc / (4 * (SSP1ADD + 1)) in Mode 0
    // If we want 2.4 MHz, we solve for SSP1ADD:
    //   2.4e6 = 32e6 / (4*(SSP1ADD + 1))
    //   32e6 / 2.4e6 = 13.3333 = 4*(SSP1ADD + 1)
    //   (SSP1ADD + 1) = 13.3333 / 4 = 3.3333
    //   SSP1ADD ? 2.33
    // We must pick an integer, so let's pick SSP1ADD=2 => 2.666 MHz (close enough!)
    // or SSP1ADD=3 => 2.0 MHz.  Either can work.

    SSP1ADD  = 2;        // This yields ~2.67 MHz (a bit above 2.4, typically still OK)
    SSP1CON1 = 0b00101010;     // SSPEN=1 (enable), CKP=0 (Idle low), SSPM=b1010 (Master Fosc/(4*(SSPxADD+1)))
    SSP1STAT = 0b00000000;     // CKE=1 (transmit on active->idle)
    
    // TRIS setup: SDO is output
    // On PIC16F18313, check datasheet for pin assignment
    TRISA4 = 0;  // configure RA4 as output (SDO)
    RA4PPS = 0b11001; // RA4 is SDO1 (SPI1 data out)
    // Remap pins if needed (APFCON, etc.). Check your PIC16F18313 pinout.
}

// inputByte = 8 bits to send to WS2812
// outArray must have space for 3 bytes (24 bits).
// We'll store them in big-endian or little-endian order as convenient.
void encodeByte(uint8_t inputByte, uint8_t outArray[4])
{
    uint8_t i;
    uint32_t encoded = 0; // We'll build up 24 bits

    // Build from MSB to LSB of inputByte
    for(i = 0; i < 8; i++)
    {
        if (i % 2 == 0){
            encoded <<= 2;
        }
        encoded <<= 3; // Make room for 3 new bits
        if (inputByte & 0x80)
        {
            // bit = '1'
            encoded |= WS2812_1;
        }
        else
        {
            // bit = '0'
            encoded |= WS2812_0;
        }
        inputByte <<= 1; // Next bit
    }

    // Now we have 24 bits in 'encoded'. Split into 3 bytes
    // The top 8 bits are (encoded >> 16), then >> 8, then low 8 bits
    outArray[0] = (uint8_t)( (encoded >> 24) & 0xFF );
    outArray[1] = (uint8_t)( (encoded >> 16) & 0xFF );
    outArray[2] = (uint8_t)( (encoded >>  8) & 0xFF );
    outArray[3] = (uint8_t)(  encoded        & 0xFF );
}

void sendColor_SPI(uint8_t g, uint8_t r, uint8_t b)
{
    uint8_t gEnc[4], rEnc[4], bEnc[4];
    encodeByte(g, gEnc);
    encodeByte(r, rEnc);
    encodeByte(b, bEnc);

    // Total 9 bytes to send for 24 bits color
    // G -> 3 bytes, R -> 3 bytes, B -> 3 bytes
    // Wait for SPI idle, then write them out.
    for(uint8_t i = 0; i < 4; i++)
    {
        SSP1BUF = gEnc[i];
        while(!SSP1STATbits.BF);  // Wait
    }
    for(uint8_t i = 0; i < 4; i++)
    {
        SSP1BUF = rEnc[i];
        while(!SSP1STATbits.BF);  // Wait
    }
    for(uint8_t i = 0; i < 4; i++)
    {
        SSP1BUF = bEnc[i];
        while(!SSP1STATbits.BF);  // Wait
    }

    // After finishing all bytes, we do a latch/reset by holding line low ~50us.
    // With SPI, usually the line idles at low if CKP=0, so just wait in software:
    __delay_us(60);  // ~60us is enough for the WS2812 to latch
}