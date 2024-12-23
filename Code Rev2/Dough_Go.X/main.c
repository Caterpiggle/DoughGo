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

#include <xc.h>
#include <stdint.h>
#include <math.h>

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

#define TSET   15 // Set temp 24C = 75F
#define K_p   0.4 // PID controller constants
#define K_i   0.4
#define dt   0.13 // dt val (s) based on TC read rate (7.65Hz)

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

float cum_err = 0; // Cumulative (integral) error
float cum_err_max = 20;
float prev_err = 0; // Previous error calculation storage

struct {
    unsigned int TC_read  : 1; // Interrupt flag for TC read
    unsigned int disp_ref : 1; // Interrupt flag for display refresh
    unsigned int disp_dig : 1; // Display digit tracker
} flags;

// Read from SPI slave
uint8_t SPI_read(void) {
    SSP1BUF = 0;             // Shift out dummy byte
    while(!SSP1STATbits.BF); // Wait for transmission completion
    
    return SSP1BUF;
}

// Shift out to SPI line
void SPI_out(uint8_t dat) {
    SSP1BUF = dat;           // Initiate data shift out
    while(!SSP1STATbits.BF); // Wait for transmission completion
}

// Print value to UART
void char_print(uint8_t dat) {
    while(!PIR1bits.TXIF); // Wait for previous transmission to complete
    TX1REG = dat;          // Move data to TX1REG
}

//
void buff_put(uint16_t dat) {
    temp_buffer[buff_index] = dat;
    buff_index++;
    
    if (buff_index == (N_buff - 1)) {
        buff_index = 0;
        buff_full = 1;
    }
}

// Temperature display refresh
void ref_disp(void) {
    if (!flags.disp_dig) { // determine digit to display and push to SPI
        SPI_out(TEMPF_tens);
    } else {
        SPI_out(TEMPF_ones);
    }
    
    CS_SR = 1; // Latch to SR
    CS_SR = 0;
    
    flags.disp_dig ^= 1;
}

void upd_disp(float dat) {
    //float TEMPC = (float)dat/16;    // Convert to degC (/16)
    float TEMPF = ((float)dat*9/5) + 32; // Convert to degF
    
    TEMPF_ones = (((uint8_t)TEMPF)%10) | 0x20; // Store temp value for display
    TEMPF_tens = (((uint8_t)TEMPF/10)%10) | 0x10;
}

// Read and store thermocouple data
uint16_t read_TC(void) {
    uint16_t DAT_BUFFER = 0;
            
    CS_TC = 0; // Read first 14 bits of MAX31855 data word
    DAT_BUFFER = SPI_read();
    DAT_BUFFER <<= 8;
    DAT_BUFFER |= SPI_read();
    CS_TC = 1;
    
    return DAT_BUFFER;
}

float get_avg_temp(void) {
    float sum = 0;
    
    uint8_t i;
    for (i=0; i<N_buff; i++) {
        sum += temp_buffer[i];
    }
    
    return sum/N_buff/16;
}

void set_PWM(uint16_t PWM) {
    PWM5DCL = PWM & 0xFFFC;
    PWM5DCH = PWM>>2;
}

void upd_PWM(void) {
    float err = TSET - cur_temp; // Calculate error from set temp
    
//    if (!SP_crossed) { // Setpoint crossing detection
//        if (mode && !err) { // Heating mode, crossing TSET
//            SP_crossed = 1;
//            cum_err = 0;
//        }
//        
//        if (!mode && err) { // Cooling mode, crossing TSET
//            SP_crossed = 1;
//            cum_err = 0;
//        }
//    }
    
    if (!mode) { // Flip sign for cooling mode
        err *= -1;
    }
    
    cum_err += dt*(prev_err + err)/2; // Calculate integral
    
    if (cum_err > cum_err_max) {
        cum_err = cum_err_max;
    }
    
    if ((cur_PWM + K_p*err + K_i*cum_err) < 0) { // Fix this to not need double calculation
        cur_PWM = 0;
    } else {
        cur_PWM += K_p*err + K_i*cum_err; // Set new PWM
    }
    
    if (cur_PWM > PWM_max) {
        cur_PWM = PWM_max;
    }
    
    prev_err = err;
}

// Peripheral interrupt handler
__interrupt() void ISR(void) {
    // Timer0 overflow ISR
    if (PIE0bits.TMR0IE && PIR0bits.TMR0IF) {
        flags.TC_read = 1;
        PIR0bits.TMR0IF = 0;
    }
    
    // Timer4 overflow ISR
    if (PIE2bits.TMR4IE && PIR2bits.TMR4IF) {
        ref_disp();
        PIR2bits.TMR4IF = 0;
    }
}

void init(void) {
    // Set Fosc = 16MHz
    OSCCON1 = 0x60;
    OSCFRQ  = 0x06;
    
    TRISA = 0;
    TRISB = 0;
    TRISC = 0;

    // Setup I/O
    
    TRISBbits.TRISB5 = 0; // CS_TC output
    TRISCbits.TRISC5 = 0; // CS_SR output
    TRISCbits.TRISC3 = 0; // IN_A output
    TRISBbits.TRISB7 = 0; // IN_B output
    TRISCbits.TRISC6 = 0; // SEL_0 output
    
    TRISCbits.TRISC0 = 0; // UART TX output
    TRISBbits.TRISB6 = 1; // SPI MISO input
    TRISAbits.TRISA4 = 0; // SCK output
    
    // Clear data latches
    LATA = 0;
    LATB = 0;
    LATC = 0;
    
    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;
    
    // Setup TC temperature read timer on Timer0
    // ~7Hz
    T0CON0bits.T016BIT = 0;   // 8bit
    T0CON0bits.T0OUTPS = 0xF; // 1:16 postscaler
    
    T0CON1bits.T0CS    = 0x2; // Fosc/4
    T0CON1bits.T0ASYNC = 0;   // Sync to Fosc/4
    T0CON1bits.T0CKPS  = 0x7; // 1:128 prescaler
    
    T0CON0bits.T0EN    = 1;   // Enable TMR0
    
    // Setup display refresh timer on Timer4
    // 120Hz (60Hz each digit)
    T4CONbits.T4OUTPS = 0x7;  // 1:8 postscaler
    T4CONbits.T4CKPS  = 0x2;  // 1:16 prescaler
    PR4               = 0xFF; // 8-bit timer period
    T4CONbits.TMR4ON  = 1;    // Enable Timer4
    
    // Setup PWM @ ~15kHz on Timer2
    TRISAbits.TRISA5    = 1;    // Disable PWM5 pin output drivers
    PWM5CONbits.PWM5POL = 0;    // Active high
    PR2                 = 0xFF; // 10-bit resolution
    PWM5DCH             = 0x80; // 50% DC = 512
    PWM5DCL             = 0x00;
    
    PIR1bits.TMR2IF   = 0; // Clear Timer2 interrupt flag
    T2CONbits.T2OUTPS = 0; // 1:1 postscaler
    T2CONbits.T2CKPS  = 0; // 1:1 prescaler
    T2CONbits.TMR2ON  = 1; // Enable TMR2
    
    while(!PIR1bits.TMR2IF);   // Wait for Timer2 overflow interrupt
    PIR1bits.TMR2IF = 0;
    
    TRISAbits.TRISA5   = 0;    // Enable PWM5 pin output drivers
    RC7PPS             = 0x02; // Route PWM5 output to RC7
    PWM5CONbits.PWM5EN = 1;    // Enable PWM5
    
    // Setup UART SFRs
    TX1STAbits.CSRC  = 1; // Master mode
    TX1STAbits.TX9   = 0; // 8bit transmission
    TX1STAbits.SYNC  = 0; // Async
    TX1STAbits.SENDB = 0; // SYNCH BREAK transmission disabled
    TX1STAbits.BRGH  = 0; // Low speed baud rate
    
	RC1STAbits.SPEN   = 1; // Serial port enabled
    BAUD1CONbits.SCKP = 0; // Idle high
    
	SP1BRGL = 25; // 9600 baud rate @ 16MHz
    
    RC0PPS  = 0x14; // Route TX to RC0
    
    TX1STAbits.TXEN  = 1; // Transmit enabled
    
    // Setup SPI SFRs
    SSP1CON1bits.CKP   = 0;   // Clock idle low
    SSP1CON1bits.SSPM  = 0;   // Fosc/4 clock
    
    SSP1STATbits.SMP = 1; // Data sampled at end
    SSP1STATbits.CKE = 1; // Transmit occurs on active->idle transition
    
    RA4PPS     = 0x18;  // Route SCK to RA4
    RC4PPS     = 0x19;  // Route SDO (MOSI) to RC4
    SSP1DATPPS = 0x0E;  // Route SDI (MISO) to RB6
    
    SSP1CON1bits.SSPEN = 1;   // Enable SSP
    
    // Enable interrupts
    PIE0bits.TMR0IE = 1; // Timer0 overflow interrupt enable
    PIE2bits.TMR4IE = 1; // Timer4 overflow interrupt enable
    INTCONbits.PEIE = 1; // Peripheral interrupt enable
    INTCONbits.GIE  = 1; // Global interrupt enable
}

void main(void) {
    
    init();
    
    set_PWM(0); // Reset PWM to 0% DC
    
    while(!buff_full) { // Get ambient temperature reading - hold until temp_buff full
        if (flags.TC_read) {
            buff_put(read_TC());
            flags.TC_read = 0;
        }
    }
    
    cur_temp = get_avg_temp();
    
    upd_disp(cur_temp); // Output average initial temp reading
    
    if (cur_temp >= TSET) { // Set to cooling mode
        IN_A  = 1;
        IN_B  = 0;
        SEL_0 = 0;
        
        mode = 0;
        
        PWM_max = 1023;
    } else { // Set to heating mode
        IN_A  = 0;
        IN_B  = 1;
        SEL_0 = 1;
        
        mode = 1;
        
        PWM_max = 256; // 25% DC
    }
    
    while (1) {
        // Read TC probe temperature
        if (flags.TC_read) {
            buff_put(read_TC());
            
            flags.TC_read = 0;
            
            cur_temp = get_avg_temp();
            
            upd_disp(cur_temp);
            upd_PWM();
            set_PWM(cur_PWM);
        }
    }
}