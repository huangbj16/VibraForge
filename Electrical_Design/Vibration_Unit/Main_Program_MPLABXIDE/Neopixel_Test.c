#include <xc.h>
#include <stdint.h>

// CONFIG1
#pragma config FEXTOSC = OFF    // FEXTOSC External Oscillator mode Selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32  // Power-up default value for COSC bits (HFINTOSC (1MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; I/O or oscillator function on OSC2)
#pragma config CSWEN = OFF      // Clock Switch Enable bit (The NOSC and NDIV bits cannot be changed by user software)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR/VPP pin function is MCLR; Weak pull-up enabled )
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config WDTE = ON       // Watchdog Timer Enable bits (WDT disabled; SWDTEN is ignored)
#pragma config LPBOREN = OFF    // Low-power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset disabled)
#pragma config BORV = LOW       // Brown-out Reset Voltage selection bit (Brown-out voltage (Vbor) set to 2.45V)
#pragma config PPS1WAY = OFF    // PPSLOCK bit One-Way Set Enable bit (The PPSLOCK bit can be set and cleared repeatedly (subject to the unlock sequence))
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a Reset)
#pragma config DEBUG = OFF      // Debugger enable bit (Background debugger disabled)

// CONFIG3
#pragma config WRT = OFF        // User NVM self-write protection bits (Write protection off)
#pragma config LVP = OFF         // Low Voltage Programming Enable bit (Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored.)

// CONFIG4
#pragma config CP = OFF         // User NVM Program Memory Code Protection bit (User NVM code protection disabled)
#pragma config CPD = OFF        // Data NVM Memory Code Protection bit (Data NVM code protection disabled)


#define _XTAL_FREQ 32000000UL  // 32 MHz for __delay_us()
// FOSC = 32 MHz
// We want ~2.4 MHz SPI => let's pick a setting close to that

void SPI_Init(void)
{
    // Example: MSSP1 in SPI Master mode
    // SCK = Fosc / (4 * (SSP1ADD + 1)) in Mode 0
    // If we want 2.4 MHz, we solve for SSP1ADD:
    //   2.4e6 = 32e6 / (4*(SSP1ADD + 1))
    //   32e6 / 2.4e6 = 13.3333 = 4*(SSP1ADD + 1)
    //   (SSP1ADD + 1) = 13.3333 / 4 = 3.3333
    //   SSP1ADD ≈ 2.33
    // We must pick an integer, so let's pick SSP1ADD=2 => 2.666 MHz (close enough!)
    // or SSP1ADD=3 => 2.0 MHz.  Either can work.

    SSP1ADD  = 2;        // This yields ~2.67 MHz (a bit above 2.4, typically still OK)
    SSP1CON1 = 0x2A;     // SSPEN=1 (enable), CKP=0 (Idle low), SSPM=b1010 (Master Fosc/(4*(SSPxADD+1)))
    SSP1STAT = 0x40;     // CKE=1 (transmit on active->idle)
    
    // TRIS setup: SDO is output
    // On PIC16F18313, check datasheet for pin assignment
    TRISA4 = 0;  // configure RA4 as output (SDO)
    RA4PPS = 0b11001; // RA4 is SDO1 (SPI1 data out)
    // Remap pins if needed (APFCON, etc.). Check your PIC16F18313 pinout.
}

// We define the 3-bit patterns for '0' and '1'.
#define WS2812_0 0b100  // ~ 0.42 us high, 0.83 us low
#define WS2812_1 0b110  // ~ 0.83 us high, 0.42 us low

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


#define NUM_BYTES_PER_COLOR 3    // 3 encoded bytes per one color byte

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
        (void)SSP1BUF;           // Clear BF
    }
    for(uint8_t i = 0; i < 4; i++)
    {
        SSP1BUF = rEnc[i];
        while(!SSP1STATbits.BF);
        (void)SSP1BUF;
    }
    for(uint8_t i = 0; i < 4; i++)
    {
        SSP1BUF = bEnc[i];
        while(!SSP1STATbits.BF);
        (void)SSP1BUF;
    }

    // After finishing all bytes, we do a latch/reset by holding line low ~50us.
    // With SPI, usually the line idles at low if CKP=0, so just wait in software:
    __delay_us(60);  // ~60us is enough for the WS2812 to latch
}

int main(void)
{
    // 1) Configure system clock to 32MHz
    //    On PIC16F18313, you might do:
    OSCFRQbits.HFFRQ = 0x06;  // 32 MHz
    // ... plus other oscillator configs

    // 2) Initialize SPI
    SPI_Init();

    // 3) Main loop: demonstrate color changes
    while(1)
    {
        // Example: Set LED to Green
        sendColor_SPI(0xFF, 0x00, 0x00); // G=0xFF, R=0, B=0
        __delay_ms(500);
        
        // Red
        sendColor_SPI(0x00, 0xFF, 0x00);
        __delay_ms(500);

        // Blue
        sendColor_SPI(0x00, 0x00, 0xFF);
        __delay_ms(500);

        // Off
        sendColor_SPI(0x00, 0x00, 0x00);
        __delay_ms(500);
    }
    
    return 0;
}
