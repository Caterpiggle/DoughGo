//=====================================================
//
// Dough-Go Rev 2
// 2024
//
// PIC16F18344
//
//              -----------------
//      VDD  --|  1           20 |--  VSS
//        x  --|  2/RA5   RA0/19 |--  ICSPDAT
//      SCK  --|  3/RA4   RA1/18 |--  ICSPCLK
//      VPP  --|  4/RA3   RA2/17 |--  x
//    CS_SR  --|  5/RC5   RC0/16 |--  UART TX
//     MOSI  --|  6/RC4   RC1/15 |--  x
//     IN_A  --|  7/RC3   RC2/14 |--  X
//    SEL_0  --|  8/RC6   RB4/13 |--  x
//      PWM  --|  9/RC7   RB5/12 |--  CS_TC
//     IN_B  --| 10/RB7   RB6/11 |--  MISO
//              -----------------
//
//=====================================================

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef DOUGH_GO_8BIT_H
#define	DOUGH_GO_8BIT_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>


// CONFIG1
#pragma config FEXTOSC  = OFF     // FEXTOSC External Oscillator mode Selection bits (Oscillator not enabled)
#pragma config RSTOSC   = HFINT1  // Power-up default value for COSC bits (HFINTOSC (1MHz)
#pragma config CLKOUTEN = OFF     // Clock Out Enable bit (CLKOUT function is disabled; I/O or oscillator function on OSC2)
#pragma config CSWEN    = ON      // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN    = ON      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config MCLRE    = ON      // Master Clear Enable bit (MCLR/VPP pin function is MCLR; Weak pull-up enabled)
#pragma config PWRTE    = OFF     // Power-up Timer Enable bit (PWRT disabled)
#pragma config WDTE     = OFF     // Watchdog Timer Enable bits (WDT disabled; SWDTEN is ignored)
#pragma config LPBOREN  = OFF     // Low-power BOR enable bit (ULPBOR disabled)
#pragma config BOREN    = OFF     // Brown-out Reset Enable bits (Brown-out Reset disabled)
#pragma config BORV     = LOW     // Brown-out Reset Voltage selection bit (Brown-out voltage (Vbor) set to 2.45V)
#pragma config PPS1WAY  = ON      // PPSLOCK bit One-Way Set Enable bit (The PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN   = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a Reset)
#pragma config DEBUG    = OFF     // Debugger enable bit (Background debugger disabled)

// CONFIG3
#pragma config WRT = OFF // User NVM self-write protection bits (Write protection off)
#pragma config LVP = OFF // Low Voltage Programming Enable bit (High Voltage on MCLR/VPP must be used for programming.)

// CONFIG4
#pragma config CP  = OFF // User NVM Program Memory Code Protection bit (User NVM code protection disabled)
#pragma config CPD = OFF // Data NVM Memory Code Protection bit (Data NVM code protection disabled)

// Pin Definitions
#define CS_TC LATBbits.LATB5 // Thermocouple chip select output
#define CS_SR LATCbits.LATC5 // Display shift register chip select output
#define IN_A  LATCbits.LATC3 // TEC driver IN_A logic input
#define IN_B  LATBbits.LATB7 // TEC driver IN_B logic input
#define SEL_0 LATCbits.LATC6 // TEC driver SEL_0 logic input

#define TSET    24 // Set temp 24C = 75F
#define K_p   0.05 // PID controller constants
#define K_i      0
#define dt    0.13 // dt val (s) based on TC read rate (7.65Hz)

#define N_buff 20 // Size of circular temperature data buffer

uint16_t temp_buffer[N_buff];

uint8_t buff_index = 0;
uint8_t buff_full = 0;

uint8_t SP_crossed = 0; // Setpoint crossing detection for integrator windup

float cur_PWM; // PWM value storage
float PWM_max; // PWM value limit

float cur_temp;

uint8_t TEMPF_ones = 0x20; // Temp disp ones digit, reset 0-on
uint8_t TEMPF_tens = 0x10; // Temp disp tens digit, reset 0-on

uint8_t temp_val = 0;
uint8_t temp_div = 0;

uint8_t mode;

float cum_err     = 0; // Cumulative (integral) error
float cum_err_max = 2; // Max cumulative error to limit windup
float prev_err    = 0; // Previous error calculation storage

struct {
    unsigned int TC_read  : 1; // Interrupt flag for TC read
    unsigned int disp_ref : 1; // Interrupt flag for display refresh
    unsigned int disp_dig : 1; // Display digit tracker
} flags;

// Read from SPI slave
uint8_t SPI_read(void);

// Shift out to SPI line
void SPI_out(uint8_t dat);

// Print value to UART
void char_print(uint8_t dat);

// Enter data in buffer
void buff_put(uint16_t dat);

// Temperature display refresh
void ref_disp(void);

// Update display
void upd_disp(float dat);

// Read and store thermocouple data
uint16_t read_TC(void);

// Get average temperature
float get_avg_temp(void);

// Set PWM
void set_PWM(uint16_t PWM);

// Update PWM
void upd_PWM(void);

void init(void);


#endif	/* DOUGH_GO_8BIT_H */

