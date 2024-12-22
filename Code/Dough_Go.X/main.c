//=====================================================
//
// Dough-Go Rev 1
// 2024
//
// PIC16F18344
//
//              -----------------
//      VDD  --|  1           20 |--  VSS
//           --|  2/RA5   RA0/19 |--  ICSPDAT
//           --|  3/RA4   RA1/18 |--  ICSPCLK
//           --|  4/RA3   RA2/17 |--  
//           --|  5/RC5   RC0/16 |--  
//           --|  6/RC4   RC1/15 |--  
//           --|  7/RC3   RC2/14 |--  
//           --|  8/RC6   RB4/13 |-- 
//           --|  9/RC7   RB5/12 |-- 
//           --| 10/RB7   RB6/11 |-- 
//              -----------------
//
//=====================================================

#include <xc.h>
#include <stdint.h>
#include <math.h>

// CONFIG1
#pragma config FEXTOSC = OFF    // FEXTOSC External Oscillator mode Selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with 2x PLL (32MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; I/O or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR/VPP pin function is MCLR; Weak pull-up enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config WDTE = OFF       // Watchdog Timer Enable bits (WDT disabled; SWDTEN is ignored)
#pragma config LPBOREN = OFF    // Low-power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset disabled)
#pragma config BORV = LOW       // Brown-out Reset Voltage selection bit (Brown-out voltage (Vbor) set to 2.45V)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (The PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a Reset)
#pragma config DEBUG = OFF      // Debugger enable bit (Background debugger disabled)

// CONFIG3
#pragma config WRT = OFF        // User NVM self-write protection bits (Write protection off)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (High Voltage on MCLR/VPP must be used for programming.)

// CONFIG4
#pragma config CP = OFF         // User NVM Program Memory Code Protection bit (User NVM code protection disabled)
#pragma config CPD = OFF        // Data NVM Memory Code Protection bit (Data NVM code protection disabled)

// Pin Definitions
#define	TX  	PORTAbits.RA0   // UART TX output
#define	RX  	PORTAbits.RA1   // UART RX input
#define	ENC_A	PORTAbits.RA4   // Encoder A input
#define	ENC_B	PORTAbits.RA5   // Encoder B input

#define PWM     PORTBbits.RB4   // Peltier driver PWM output
#define DIR_SEL PORTBbits.RB5   // Peltier driver direction selection output
#define CS_TC   PORTBbits.RB6   // Thermocouple chip select output
#define RCLK    PORTBbits.RB7   // Shift register latch output

#define DEC_A   PORTCbits.RC0   // Display decoder bit 0 output
#define DEC_B   PORTCbits.RC1   // Display decoder bit 1 output
#define DEC_C   PORTCbits.RC2   // Display decoder bit 2 output
#define DEC_D   PORTCbits.RC3   // Display decoder bit 3 output
#define MOSI    PORTCbits.RC4   // SPI MOSI output
#define MISO    PORTCbits.RC5   // SPI MISO input
#define SCK     PORTCbits.RC6   // SPI clock output
#define DRDY    PORTCbits.RC7   // TC Data ready input

#define T_MAX   85
#define T_MIN   70

#define _XTAL_FREQ (4000000)

// PI controller constants
uint8_t K_P = 1;
uint8_t K_I = 1;

// Temperature variables
uint8_t T_cur = 70;
uint8_t T_set = 85;

uint8_t T_cur_tens = 0;
uint8_t T_cur_ones = 0;
uint8_t T_set_tens = 0;
uint8_t T_set_ones = 0;

// PI controller variables
uint8_t err_prev    = 0;
uint8_t dt          = 0;

uint8_t DC_max = 0; // Max duty cycle

uint8_t TC_count = 0;

uint8_t disp_dig[] = {0x01,0x02,0x04,0x08};
uint8_t disp_num   = 0;

// Array for tracking values in current/set temperature displays
// {T_cur_tens,T_cur_ones,T_set_tens,T_set_ones}
uint8_t disp_T[] = {0,0,0,0};

// SPI shift helper functions
void SPI_out(uint8_t dat) {
    // Initiate data shift out
    SSP1BUF = dat;
    // Wait for transmission completion
    while(!SSP1STATbits.BF);
}

uint8_t SPI_read(uint8_t addr) {
    // Shift out address
    SPI_out(addr);
    // Shift in address contents
    SPI_out(0x00);
    
    return SSP1BUF;
}

// UART helper function
void char_print(uint8_t dat) {
    // Wait for previous transmission to complete
    while(!PIR1bits.TXIF);
    
    // Move data to TX1REG
    TX1REG = dat;
}

// Update set temperature from encoder interrupts
void upd_temp(void) {
    // Split temperatures into digits
    T_cur_tens = T_cur/10;
    T_set_tens = T_set/10;
    
    T_cur_ones = T_cur%10;
    T_set_ones = T_set%10;
    
    disp_T[0] = T_cur_tens;
    disp_T[1] = T_cur_ones;
    disp_T[2] = T_set_tens;
    disp_T[3] = T_set_ones;
}

void SPI_read_TC(void) {
    uint8_t TC_HB;
    uint8_t TC_MB;
    uint8_t TC_LB;
    
    CS_TC = 0;
    TC_HB = SPI_read(0x0C); // LTCBH=
    CS_TC = 1;
    
    CS_TC = 0;
    TC_MB = SPI_read(0x0D); // LTCBH
    CS_TC = 1;
    
    
    CS_TC = 0;
    TC_LB = SPI_read(0x0E); // LTCBH
    CS_TC = 1;
    
    //int temp = ((TC_HB << 16) + (TC_MB << 8) + TC_LB)*1600/6553600;
    T_cur = TC_MB;
}

void get_cur_temp(void) {
    CS_TC = 0;
    SPI_out(0x00);  // CR0
    SPI_out(0x40);  // Set 1SHOT bit
    CS_TC = 1;      // Trigger conversion
    
    while (DRDY);
    
    SPI_read_TC();
    upd_temp();
}

// Update temperature displays
void upd_disp(void) {
    // Get value for current display digit
    // Set output to value
    
    SPI_out(0x00);  // Clear display
    RCLK = 1;
    RCLK = 0;
    
    disp_num++;     // Increment counters
    TC_count++;
        
    if (disp_num > 3) {
        disp_num = 0;
    }
        
    if (TC_count > 250) {   // TC temp read at 1Hz
        SPI_read_TC();
        TC_count = 0;
        upd_temp();
    }
        
    PORTC &= 0xF0;
    PORTC |= disp_T[disp_num];
    
    SPI_out(disp_dig[disp_num]);
    RCLK = 1;
    RCLK = 0;
}

// Update PWM from PI control loop
void upd_PWM(void) {
    // Read current temp
    
    // PI control calculation
    uint8_t err = T_set - T_cur;
    uint8_t y = K_P*err + dt*K_I*(err + err_prev)/2;
            
    err_prev = err;
}

// Peripheral interrupt handler
__interrupt() void ISR(void) {
    
    INTCONbits.GIE  = 0;
    INTCONbits.PEIE = 0;
    
    // Timer0 Overflow ISR
    // Update display
    if (PIE0bits.TMR0IE && PIR0bits.TMR0IF) {
        upd_disp();
        PIR0bits.TMR0IF = 0;
    }
    
    if (PIE0bits.IOCIE && PIR0bits.IOCIF) {
        // Determine which pin/edge changed
        if (IOCAFbits.IOCAF4) {
            if (PORTAbits.RA5 == 1) {
                T_set += 1;
                if (T_set > T_MAX) {
                    T_set = T_MAX;
                }
            } else {
                T_set -= 1;
                if (T_set < T_MIN) {
                    T_set = T_MIN;
                }
            }
        }
        
        IOCAF &= (IOCAF ^ 0xFF);
        
        upd_temp();
    }
    
    INTCONbits.GIE  = 1;
    INTCONbits.PEIE = 1;
    
}

void main(void) {
    
    // Set Fosc = 4MHz
    OSCCON1 = 0x60;
    OSCFRQ  = 0x03;
    
    __delay_ms(2000);

    // Setup I/O
    TRISAbits.TRISA0 = 0;   // UART TX output
    TRISAbits.TRISA1 = 1;   // UART RX input
    TRISAbits.TRISA4 = 1;   // Encoder A input
    TRISAbits.TRISA5 = 1;   // Encoder B input

    TRISBbits.TRISB5 = 0;   // Peltier driver direction selection output
    TRISBbits.TRISB6 = 0;   // Thermocouple chip select output
    TRISBbits.TRISB7 = 0;   // Shift register latch output
    

    TRISCbits.TRISC0 = 0;   // Display decoder bit 0 output
    TRISCbits.TRISC1 = 0;   // Display decoder bit 1 output
    TRISCbits.TRISC2 = 0;   // Display decoder bit 2 output
    TRISCbits.TRISC3 = 0;   // Display decoder bit 3 output
    TRISCbits.TRISC4 = 0;   // SPI MOSI output
    TRISCbits.TRISC5 = 1;   // SPI MISO input
    TRISCbits.TRISC6 = 0;   // SPI clock output
    TRISCbits.TRISC7 = 1;   // TC Data ready input

    // Initiate ports
    PORTA = 0x00;
    PORTB = 0x40; // RB6 = 1
    PORTC = 0x00;
    
    // Clear data latches
    LATA = 0x00;
    LATB = 0x00;
    LATC = 0x00;
    
    ANSELA = 0x00;
    ANSELB = 0x00;
    ANSELC = 0x00;

    // Setup display refresh timer
    // Timer0: 8-bit, Fosc/4, 1:16 prescaler, 1:1 postscaler
    // 0.0164s / 61Hz full display refresh
    T0CON0 = 0x80;
    T0CON1 = 0x44;
    
    // Setup PPS
        // PWM output
   
    RA0PPS     = 0x14;  // Set RA0 to TX
    RC6PPS     = 0x18;  // Set RC6 to SCK
    RC4PPS     = 0x19;  // Set RC4 to SDO (MOSI)
    SSP1DATPPS = 0x15;  // Set RC5 to SDI (MISO)
    
    // Setup UART SFRs
    TX1STA   = 0x24;
	RC1STA   = 0x80;
	SP1BRGL  = 0x19;
	BAUD1CON = 0x00;
    
    // Setup PWM
    /*
    TRISBbits.TRISB4 = 1;       // Peltier driver PWM output
    PWM5CONbits.PWM5POL = 0;    // Active high
    PR2     = 0xFF; // 10-bit resolution
    PWM5DCH = 0x4B; // ~30% DC  
    PWM5DCL = 0x00;
    T2CON   = 0x04;
    
    while (!TMR2IF);
    
    TRISBbits.TRISB4 = 0;
    RB4PPS = 0x02;
    PWM5CONbits.PWM5EN = 1;
    
    */
    // Setup SPI SFRs: SPI master mode, FOSC/4, clock idles low
    // Sample at middle of clock, transmit on falling edge
    SSP1CON1 = 0x22;
    SSP1STATbits.SMP = 0;
    SSP1STATbits.CKE = 1;
    
    // Setup encoder IOC
    // RA4, RA5 rising/falling edge IOC
    IOCANbits.IOCAN4 = 1;
    //IOCAPbits.IOCAP5 = 1;
    //IOCANbits.IOCAN5 = 1;
    //IOCAPbits.IOCAP5 = 1;
    
    // Setup TC converter IC
    //CS_TC = 0;
    //SPI_out(0x80);  // Write - CR0
    //SPI_out(0x80);  // CR0 value
    //CS_TC = 1;
    
    //CS_TC = 0;
    
    //while (1) {
    //    char_print(SPI_read(0x00));
    //}
    
    RCLK  = 0;
    
    PORTCbits.RC0 = 0;	
    PORTCbits.RC1 = 0;	
    PORTCbits.RC2 = 0;	
    PORTCbits.RC3 = 0;
    
    // Enable interrupts
    PIE0bits.IOCIE  = 1;
    PIE0bits.TMR0IE = 1;
    //PIE1bits.SSP1IE = 1;
    INTCONbits.GIE  = 1;
    INTCONbits.PEIE = 1;
    
    while (1);
}
