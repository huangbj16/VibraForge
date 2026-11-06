/*
 * File:   main.c
 * Author: MasonC
 *
 * Created on November 8, 2022, 10:19 PM
 * Revised on November 6, 2026 by Bingjian Huang and Darius Giannoli
 *
 * Vibration unit firmware for the PIC16F18313 + DRV8837 board. It drives the actuator with a 64-step
 * sine (SPWM on a 40 kHz carrier) at 16 intensity levels x 8 frequencies (123-384 Hz), and brakes it
 * when idle.
 *
 * UART protocol (same as the square-wave firmware): 115200 baud, 9 bits, the 9th bit is even parity.
 *   address byte [0][addr6][start]: addr 0 is this board; otherwise it is forwarded with addr - 1.
 *                                   start = 0 stops this board.
 *   data byte    [1][duty4][freq3]: right after a start address byte for this board, sets the
 *                                   intensity (0 = idle) and frequency; otherwise forwarded unchanged.
 */

#include <stdio.h>
#include <stdlib.h>
#include "neopixel_control.h"

// PIC16F18313 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = OFF    // FEXTOSC External Oscillator mode Selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32  // Power-up default value for COSC bits (HFINTOSC with 2x PLL, 32 MHz)
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; I/O or oscillator function on OSC2)
#pragma config CSWEN = OFF      // Clock Switch Enable bit (The NOSC and NDIV bits cannot be changed by user software)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR/VPP pin function is MCLR; Weak pull-up enabled )
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config WDTE = OFF       // Watchdog Timer Enable bits (WDT disabled; SWDTEN is ignored)
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

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#include <xc.h>
#include <stdint.h>
#include <stdlib.h>
#define _XTAL_FREQ  32000000     // Set clock frequency

uint8_t buffer = 0; // uart recv buffer
uint8_t parity = 0; // uart recv parity
volatile uint8_t uart_recv_flag = 0;
uint8_t state = 0; // is the board being addressed by the controller

volatile uint8_t duty_index = 0; // 0-15 intensity, 0 = idle (brake)
uint8_t freq_index = 3; // 0-7 maps to frequency {123, 145, 170, 200, 235, 275, 322, 384} Hz;

// neopixel color per intensity (G, R, B as sent by sendColor_SPI)
uint8_t color_index = 0;
const uint8_t KEY16[16][3] = {
    {0,0,0}, {0,32,32}, {0,64,64}, {0,128,128},         // off, dark teal, teal, cyan-blue
    {0,255,255}, {0,255,128}, {0,255,0}, {128,255,0},   // cyan, light greenish cyan, green, yellow-green
    {255,255,0}, {255,128,0}, {255,0,0}, {255,0,128},   // yellow, orange, red, pink
    {255,0,255}, {128,0,255}, {64,0,255}, {255,255,255} // bright magenta, magenta, purple, white
};

// ==================== DRIVER PINS ====================
// RA0/RA1 drive the DRV8837 inputs. Both low coasts (outputs float and pick up mains hum), both high
// brakes (both outputs driven low), which also stops the actuator faster.
#define CCP1_PPS_CODE 0x0C
#define COAST_MACRO() { RA1PPS=0; LATAbits.LATA1=0; RA0PPS=0; LATAbits.LATA0=0; }
#define BRAKE_MACRO() { LATAbits.LATA1=1; LATAbits.LATA0=1; RA1PPS=0; RA0PPS=0; }

// ==================== SINE ====================
// 64 steps per cycle, each timed by Timer1 (1 us ticks). The ISR rewrites Timer1 ~3.6 us after it
// overflows (measured), so f = 1 MHz / (64 * (65536 - T1_RELOAD + 3.6)).
#define SINE_LEN 64u
const uint16_t T1_RELOAD[8] = {
    65413, 65432, 65448, 65461, 65473, 65483, 65491, 65499  // 123, 145, 170, 200, 235, 275, 322, 384 Hz
};

// CCP1 compare value (of the 800-count, 40 kHz PWM period) for |sine| at quarter-wave steps 0-16,
// one row per intensity 1-15. The PWM pin drives the actuator while low and brakes it while high.
// Intensities 1-14 are generated as  value = 799 - min(799, (s * Q[k] + 64) >> 7),  s = (799 * pct + 50) / 100,
//   Q   = {0,12,25,37,49,60,71,81,90,99,106,113,118,122,125,127,127}  (quarter of a 64-step sine, peak 127)
//   pct = {0,1,2,3,5,6,9,12,16,21,27,35,46,59,77}  (peak amplitude % per intensity, the old duty cycles)
// Intensity 15 is a full-scale sine,  value = 800 - (800 * Q[k] + 63) / 127,  so its peak (value 0)
// keeps the pin low for the whole PWM period: 100 % drive.
const uint16_t SINE_PWM[15][17] = {
    {799,798,797,797,796,795,795,794,793,793,792,792,792,791,791,791,791},  //  1:  1 %
    {799,797,796,794,793,791,790,789,788,787,786,785,784,784,783,783,783},  //  2:  2 %
    {799,797,794,792,790,788,786,784,782,780,779,778,777,776,776,775,775},  //  3:  3 %
    {799,795,791,787,784,780,777,774,771,768,766,764,762,761,760,759,759},  //  4:  5 %
    {799,794,790,785,781,776,772,769,765,762,759,757,755,753,752,751,751},  //  5:  6 %
    {799,792,785,778,771,765,759,753,748,743,739,735,733,730,729,728,728},  //  6:  9 %
    {799,790,780,771,762,754,746,738,731,725,719,714,710,707,705,704,704},  //  7: 12 %
    {799,787,774,762,750,739,728,718,709,700,693,686,681,677,674,672,672},  //  8: 16 %
    {799,783,766,750,735,720,706,693,681,669,660,651,644,639,635,632,632},  //  9: 21 %
    {799,779,757,737,716,698,679,662,647,632,620,608,600,593,588,585,585},  // 10: 27 %
    {799,773,744,718,692,668,644,622,602,582,567,552,541,532,526,521,521},  // 11: 35 %
    {799,764,727,693,658,626,595,566,540,514,494,474,460,448,440,434,434},  // 12: 46 %
    {799,755,707,663,619,578,538,501,468,435,409,383,365,350,339,332,332},  // 13: 59 %
    {799,741,679,621,564,511,458,410,367,323,290,256,232,213,198,189,189},  // 14: 77 %
    {800,724,643,567,491,422,353,290,233,176,132, 88, 57, 31, 13,  0,  0},  // 15: 100 %
};

const uint16_t *sine_row = SINE_PWM[0]; // SINE_PWM row for the current intensity
volatile uint8_t  sine_on   = 0;
volatile uint16_t t1_reload = 0;
volatile uint8_t  phase_idx = 0;
volatile int8_t   last_sign = 0;

static inline void route_halfcycle(int8_t sign){
    // the PWM goes to one input, the other is held high: drive one way while the PWM is low, brake while high
    if (sign > 0){
        RA0PPS = CCP1_PPS_CODE; RA1PPS = 0;
        LATAbits.LATA1 = 1; LATAbits.LATA0 = 0;
    } else {
        RA1PPS = CCP1_PPS_CODE; RA0PPS = 0;
        LATAbits.LATA0 = 1; LATAbits.LATA1 = 0;
    }
}

void sine_idle(void){ // stop, or start at intensity 0
    T1CONbits.TMR1ON = 0; PIE1bits.TMR1IE = 0; PIR1bits.TMR1IF = 0;
    sine_on = 0;
    BRAKE_MACRO();
    CCPR1H = 3; CCPR1L = 0x1F; // 799: the PWM pin stays high (brake) for the first steps of the next start
}

void sine_apply(void){ // start, or change intensity/frequency without restarting the sine
    if (duty_index == 0){ sine_idle(); return; }
    t1_reload = T1_RELOAD[freq_index];
    sine_row = SINE_PWM[duty_index - 1];
    if (sine_on) return; // already running: the next step picks up the new values
    phase_idx = 0; last_sign = 0;
    T1CON = 0; T1CONbits.T1CKPS = 0b11; // Fosc/4 / 8 = 1 us ticks
    TMR1H = (uint8_t)(t1_reload >> 8); TMR1L = (uint8_t)(t1_reload & 0xFF);
    PIR1bits.TMR1IF = 0; PIE1bits.TMR1IE = 1;
    sine_on = 1;
    T1CONbits.TMR1ON = 1;
}

void init_pwm(void) {
    LATA = 0; TRISA = 0b00111100; // RA0, RA1 outputs to the DRV8837
    ANSELA = 0; // all digital
    WPUA = 0; // weak pullup
    BRAKE_MACRO();
    CCP1CON = 0b10001111; // PWM, right-aligned: CCPR1H:CCPR1L is the 10-bit compare value
    CCPR1H = 3; CCPR1L = 0x1F; // 799
    T2CON = 0; PR2 = 199; TMR2 = 0; // 1:1 prescaler: 40 kHz PWM with 800 counts per period
    T2CONbits.TMR2ON = 1;
}

void usart_init() {
    TRISA5 = 1; //port 5 as RXinput
    TRISA2 = 1; //port 2 as TXout
    ANSELA = 0;
    RXPPS = 0b00101; //port5 RX
    RA2PPS = 0b10100; //port2 TX

    RC1STA = 0b10010000; // 8 bit continuous reception CREN = 1 SPEN = 1
    TX1STA = 0b00100100; // Asynchronous reception High Baud Rate selection BRGH = 1  TXEN = 1; //transmit enable //async mode
    BAUD1CON = 0b00001000; // 16 bit SPBRG   BRG16 = 1
    SP1BRGH = 0;
    SP1BRGL = 68; // 115.2k baud rate
    TX9 = 1; //enable 9bit transmission
    RX9 = 1; //enable 9-bit recv
    RCIE = 1; //receive interrupt enable

    __delay_us(100); //wait for set up
}

uint8_t make_addr_byte(uint8_t start, uint8_t addr){
    //make the first, address byte for transmission
    uint8_t addr_byte = 0;
    addr_byte |= (start & 0b1);
    addr_byte |= (addr << 1);

    return addr_byte;
}

uint8_t getParity(uint8_t n)
{ //return the parity of n. if odd, return 1, even, return 0
    uint8_t parity = 0;
    while (n)
    {
        parity = !parity;
        n = n & (n - 1);
    }
    return (parity & 0b1);
}

void UART_Write(uint8_t data) {
    while(!TRMT){};

    TX9D = (getParity(data) & 0b1);
    TX1REG = data;
}

void UART_processing(){
    if(getParity(buffer) != parity) return; //drop packet
    if((buffer >> 7) != 0b1){ //addr byte recved
        uint8_t addr = (buffer >> 1); //get addr
        uint8_t start = (buffer & 0b1); //get start/stop
        if(addr != 0){ //not the current address
            state = 0;// if state = 1 but the byte after is not data byte, then the unit should reset, to let commands go through.
            --addr; //decrease addr
            UART_Write(make_addr_byte(start, addr)); //send
            return;
        }
        else{ //the current addr
            state = start;
            if(start == 0){
                duty_index = 0;
                sine_idle();
            }
            return;
        }
    }
    else { //data byte received
        if(state == 0){ //previous data byte not address to this board
            while(!TRMT){};
            TX9D = parity & 0b1;
            TX1REG = buffer;//transmit directly
            return;
        }
        else{ //data byte addressed to this board
            freq_index = (buffer & 0b111);
            duty_index = (buffer & 0b1111000) >> 3;
            sine_apply();
            state = 0;//state flipped to 0 again
            return;
        }
    }
}

void __interrupt() ISR(void) {
    if (RCIF) {
        if (RC1STAbits.OERR){ RC1STAbits.CREN = 0; RC1STAbits.CREN = 1; } // an overrun stops reception until CREN is cycled
        if (RC1STAbits.FERR){ volatile uint8_t junk = RC1REG; (void)junk; return; } // drop bytes with a framing error
        parity = RX9D; //read the 9th parity bit before RC1REG pops the byte
        buffer = RC1REG; // Read The Received Data Buffer
        UART_processing();
        uart_recv_flag = 1;
    }
    else if (PIR1bits.TMR1IF){
        PIR1bits.TMR1IF = 0;
        if (!sine_on) return;
        // T1_RELOAD is calibrated to the cycles it takes to get here (27): keep the RCIF and TMR1IF
        // tests, the sine_on check and this reload as they are, with sine_on and t1_reload in bank 0
        TMR1H = (uint8_t)(t1_reload >> 8); TMR1L = (uint8_t)(t1_reload & 0xFF);

        uint8_t i = phase_idx + 1; if (i >= SINE_LEN) i = 0; phase_idx = i;
        uint8_t q = i & 31u;
        if (q){ // steps 0 and 32 are the zero crossings: keep the current half-cycle
            int8_t sgn = (i < 32u) ? 1 : -1;
            if (sgn != last_sign){ COAST_MACRO(); route_halfcycle(sgn); last_sign = sgn; }
            if (q > 16u) q = 32u - q; // |sine| mirrors around steps 16 and 48
        }
        uint16_t v = sine_row[q];
        CCPR1L = (uint8_t)v; CCPR1H = (uint8_t)(v >> 8);
    }
}

int main(int argc, char** argv) {
    init_pwm();
    usart_init();
    SPI_Init();
    PEIE = 1; GIE = 1;

    // Infinite loop
    while(1) {
        if (uart_recv_flag) {
            uart_recv_flag = 0;
            // also check if the color needs to be updated
            if (color_index != duty_index){
                color_index = duty_index;
                sendColor_SPI(KEY16[color_index][0], KEY16[color_index][1], KEY16[color_index][2]);
            }
        }
    }
}
