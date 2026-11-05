/* Vibration_Unit_FINAL.c ? PIC16F18313
 * VERSION OPTIMISEE TAILLE (LIGHT) AVEC LUTS CALIBREES
 * * SINE Curve:   Min 4%  -> Mid 42% -> Max 100% (Progressive douce)
 * SQUARE Curve: Min 6%  -> Mid 20% -> 75%@65 -> Max 100% (Douce puis Agressive)
 */

#include <xc.h>
#include <stdint.h>
#include "neopixel_control.h" 

// ==================== Config Bits ====================
#pragma config FEXTOSC=OFF, RSTOSC=HFINT32, CLKOUTEN=OFF, CSWEN=OFF, FCMEN=OFF
#pragma config MCLRE=OFF, PWRTE=ON, WDTE=OFF, LPBOREN=OFF, BOREN=OFF, BORV=LOW, PPS1WAY=OFF, STVREN=ON, DEBUG=OFF
#pragma config WRT=OFF, LVP=OFF, CP=OFF, CPD=OFF

// ==================== Globals ====================
static volatile uint8_t buffer = 0;
static volatile uint8_t state  = 0;     
static volatile uint8_t wave_mode  = 1; 
static volatile uint8_t duty5_raw  = 0; 
static volatile uint8_t duty_pct   = 0; 
static volatile uint8_t freq_index = 3; 
static volatile uint8_t uart_led_flag = 0;

// ==================== LUTS (CALIBRATION UTILISATEUR) ====================

// SINE: 4 -> 42 -> 100 (Gamma ~1.3)
const uint8_t LUT_SINE[32] = {
    0,   4,   5,   7,   9,  11,  13,  15,
   17,  20,  23,  26,  29,  32,  35,  39,
   42,  46,  50,  54,  58,  62,  66,  71,
   75,  80,  85,  90,  95,  97,  99, 100
};

// SQUARE: 6 -> 20 -> 65 -> 100 (Multi-pente)
// 0-16: Progression lente (6->20). 16-24: Montée rapide (20->65). 24-31: Fin (65->100)
const uint8_t LUT_SQUARE[32] = {
    0,   6,   7,   8,   9,  10,  11,  12,
   13,  14,  15,  16,  17,  18,  19,  19,
   20,  26,  32,  38,  44,  50,  56,  62,
   68,  73,  78,  83,  88,  92,  96, 100
};

// ==================== LED LUT (Compacte) ====================
static uint8_t color[3] = {0,0,0};
static const uint8_t KEY16[16][3] = {
  {0,0,0}, {0,32,32}, {0,64,64}, {0,128,128},
  {0,255,255}, {0,255,128}, {0,255,0}, {128,255,0},
  {255,255,0}, {255,128,0}, {255,0,0}, {255,0,128},
  {255,0,255}, {128,0,255}, {64,0,255}, {255,255,255}
};

static void update_led_color(uint8_t idx32){
    if(idx32 > 31) idx32 = 31;
    uint8_t seg = idx32 >> 1;
    color[0] = KEY16[seg][0];
    color[1] = KEY16[seg][1];
    color[2] = KEY16[seg][2];
}

// ==================== PWM & TOOLS ====================
static inline void set_pwm10_isr(uint16_t dc10){
    CCPR1H = (uint8_t)(dc10 >> 2);
    CCPR1L = (uint8_t)((dc10 & 0x3u) << 6);
}

#define COAST_MACRO() { RA1PPS=0; LATAbits.LATA1=0; RA0PPS=0; LATAbits.LATA0=0; }

// ==================== TABLES SINE ====================
#define SINE_LEN 64u
#define CCP1_PPS_CODE 0x0C
static const int8_t SINE64_8[SINE_LEN] = {
     0, 12, 25, 37, 49, 60, 71, 81, 90, 99,106,113,118,122,125,127,
   127,127,125,122,118,113,106, 99, 90, 81, 71, 60, 49, 37, 25, 12,
     0,-12,-25,-37,-49,-60,-71,-81,-90,-99,-106,-113,-118,-122,-125,-127,
  -127,-127,-125,-122,-118,-113,-106,-99,-90,-81,-71,-60,-49,-37,-25,-12
};

// f = 1 MHz / (64 * (65536 - T1_RELOAD + ~3.6 us ISR reload latency, measured)) -> 123, 145, 170, 200, 235, 275, 322, 384 Hz
static const uint16_t T1_RELOAD[8] = {
    65413, 65432, 65448, 65461, 65473, 65483, 65491, 65499 
};

static volatile uint8_t  phase_idx = 0;
static volatile int8_t   last_sign = 0;
static volatile uint16_t t1_reload = 0;
static volatile uint16_t scale     = 0;

static inline void route_halfcycle(int8_t sign){
    if (sign > 0){
        RA0PPS = CCP1_PPS_CODE; RA1PPS = 0;
        LATAbits.LATA1 = 1; LATAbits.LATA0 = 0;
    } else {
        RA1PPS = CCP1_PPS_CODE; RA0PPS = 0;
        LATAbits.LATA0 = 1; LATAbits.LATA1 = 0;
    }
}

static void lra_set_amp(uint8_t pct){
    if (pct > 100) pct = 100;
    uint32_t top = 799; 
    scale = (uint16_t)((top * pct + 50) / 100); 
    if (!scale){ COAST_MACRO(); set_pwm10_isr(0); }
}

// ==================== SQUARE ====================
static const uint8_t PR_val[8] = {85,72,60,52,44,37,32,27};
static volatile uint8_t index200 = 0;
static volatile uint8_t cwg_flag  = 0;
static volatile uint8_t duty_flag = 0;
static volatile uint8_t square_tick = 0;

static void square_processing(void){
    uint8_t on = 0;
    if (index200 < duty_pct) on = 1;
    else if (index200 >= 100 && index200 < (100 + duty_pct)) on = 1;

    if (on){
        uint8_t half = (index200 >= 100);
        if (cwg_flag == 0 || (last_sign != (half ? -1 : +1))){
            CWG1CON0bits.EN = 0;
            if (!half){ CWG1CON1bits.POLA=0; CWG1CON1bits.POLB=0; last_sign=1; }
            else       { CWG1CON1bits.POLA=1; CWG1CON1bits.POLB=1; last_sign=-1; }
            CWG1CON0bits.EN = 1; cwg_flag = 1;
        }
        if (duty_flag == 0){
            set_pwm10_isr(799); // Top direct pour max power square
            duty_flag = 1;
        }
    }else{
        if (cwg_flag == 1){
            CWG1CON0bits.EN = 0; CWG1CON1bits.POLA=0; CWG1CON1bits.POLB=1;
            CWG1CON0bits.EN = 1; cwg_flag = 0;
        }
        if (duty_flag == 1){
            set_pwm10_isr(0); duty_flag = 0;
        }
    }
}

// ==================== UART ====================
static void UART_Write(uint8_t d){ while(!TRMT){} TX1REG = d; }

static void UART_processing(void){
    if ((buffer & 0x80) == 0){ // ADDR
        uint8_t addr  = (buffer >> 1) & 0x3F;
        uint8_t start = (buffer & 1u);
        if (addr != 0){
            --addr; 
            UART_Write((addr << 1) | start); 
            state = 0; 
        }else{
            if (!start){
                T1CONbits.TMR1ON = 0; PIE1bits.TMR1IE = 0;
                COAST_MACRO(); set_pwm10_isr(0);
                T2CONbits.TMR2ON = 0; TMR2IE = 0; CWG1CON0bits.EN = 0; 
                duty_pct = 0; cwg_flag = 0; duty_flag = 0; square_tick = 0;
                duty5_raw = 0;        // Reset duty for LED
                uart_led_flag = 1;    // Trigger LED update (will turn off)
                state = 0;
            }else{ state = 1; }
        }
    }else{ // DATA
        if (state == 0){ UART_Write(buffer); }
        else if (state == 1){
            duty5_raw = buffer & 0x1F; state = 2; 
        }else{
            // === DECODAGE STANDARD (AVEC LUTs) ===
            uint8_t d2 = buffer & 0x7F;
            wave_mode  = (d2 >> 3) & 0x01;
            freq_index = d2 & 0x07;

            // Sécurité Index
            if (duty5_raw > 31) duty5_raw = 31;

            // Application des LUTs calibrées
            if (wave_mode) {
                duty_pct = LUT_SINE[duty5_raw];
            } else {
                duty_pct = LUT_SQUARE[duty5_raw];
            }

            uart_led_flag = 1;

            if (wave_mode){ // SINE
                T2CONbits.TMR2ON = 0; T2CONbits.T2CKPS = 0; 
                PR2 = 199; set_pwm10_isr(0);
                TMR2 = 0; TMR2IF = 0; T2CONbits.TMR2ON = 1;
                CCP1CON = 0b10011111; CWG1CON0bits.EN = 0; 
                COAST_MACRO();
                T1CON = 0; T1CONbits.T1CKPS = 0b11; 
                PIR1bits.TMR1IF = 0; PIE1bits.TMR1IE = 1;
                phase_idx = 0; last_sign = 0;
                lra_set_amp(duty_pct);       
                
                t1_reload = T1_RELOAD[freq_index];
                T1CONbits.TMR1ON = 0; TMR1H=(t1_reload>>8); TMR1L=(t1_reload&0xFF);
                PIR1bits.TMR1IF = 0; T1CONbits.TMR1ON = 1;
            }else{ // SQUARE
                T1CONbits.TMR1ON = 0; PIE1bits.TMR1IE = 0;
                COAST_MACRO();
                CCP1CON = 0b10011111;
                RA1PPS = 0b01000; RA0PPS = 0b01001; 
                CWG1CON0bits.EN = 1; 
                T2CONbits.TMR2ON = 0; T2CONbits.T2CKPS = 1; 
                PR2 = PR_val[freq_index];    
                TMR2 = 0; TMR2IF = 0; T2CONbits.TMR2ON = 1; TMR2IE = 1;
                index200 = 0; cwg_flag = 0; duty_flag = 0; square_tick = 0; last_sign = 0;
            }
            state = 0; 
        }
    }
}

// ==================== ISR ====================
void __interrupt() ISR(void){
    if (RCIF){
        if (RC1STAbits.OERR){ RC1STAbits.CREN=0; RC1STAbits.CREN=1; }
        if (RC1STAbits.FERR){ volatile uint8_t junk=RC1REG; (void)junk; RCIF=0; return; }
        RCIF=0; buffer=RC1REG; UART_processing();
    }
    else if (PIR1bits.TMR1IF){ 
        PIR1bits.TMR1IF = 0;
        if (!wave_mode) return;
        TMR1H = (uint8_t)(t1_reload >> 8); TMR1L = (uint8_t)(t1_reload & 0xFF);
        
        uint8_t i = phase_idx + 1; if (i >= SINE_LEN) i = 0; phase_idx = i;
        int8_t  s8 = SINE64_8[i];
        int8_t  sgn = (s8 > 0) ? 1 : ((s8 < 0) ? -1 : last_sign);
        
        if (!scale){ COAST_MACRO(); set_pwm10_isr(0); last_sign = sgn; return; }
        if (sgn != last_sign){ COAST_MACRO(); route_halfcycle(sgn); last_sign = sgn; }
        
        uint16_t mag = (uint16_t)(s8 >= 0 ? s8 : -s8);
        uint32_t tmp = (uint32_t)scale * (uint32_t)mag;
        uint16_t duty_fwd = (uint16_t)((tmp + 64u) >> 7);
        uint16_t top = 799; 
        if (duty_fwd >= top) duty_fwd = top;
        set_pwm10_isr(top - duty_fwd);
    }
    else if (TMR2IF){
        TMR2IF = 0;
        if (!wave_mode){
            index200++; if (index200 == 200) index200 = 0;
            square_tick = 1;
        }
    }
}

// ==================== Main ====================
int main(void){
    LATA = 0; TRISA = 0b00111100; ANSELA = 0; CWG1CON0 = 0; 
    
    // Init UART
    RXPPS = 0x05; RA2PPS = 0x14; 
    RC1STA = 0x90; TX1STA = 0x24; BAUD1CON = 0x08; 
    SP1BRGH=0; SP1BRGL=68; RCIE=1;

    // Init PWM/CWG 
    CCP1CON = 0x9F; 
    CCPR1H = 0; CCPR1L = 0; 
    T2CON = 0; TMR2 = 0; PR2 = 199; T2CONbits.TMR2ON = 1;
    CWG1CLKCON = 1; CWG1DAT = 3; CWG1CON1 = 0; CWG1AS0 = 0x78; CWG1DBR=0; CWG1DBF=0;
    CWG1CON0 = 0x44; CWG1CON0bits.EN = 0; 
    COAST_MACRO();
    
    SPI_Init();

    PEIE = 1; GIE = 1;

    for(;;){
        if (uart_led_flag){
            uart_led_flag = 0;
            update_led_color(duty5_raw);
            uint8_t gie = INTCONbits.GIE; INTCONbits.GIE = 0;
            sendColor_SPI(color[0], color[1], color[2]);
            INTCONbits.GIE = gie;
        }
        if (square_tick){ square_processing(); square_tick = 0; }
    }
    return 0;
}