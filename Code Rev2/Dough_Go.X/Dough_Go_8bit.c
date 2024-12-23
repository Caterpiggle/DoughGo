#include "Dough_Go_8bit.h"

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

// Enter data in buffer
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

// Update display
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

// Get average temperature
float get_avg_temp(void) {
    float sum = 0;
    
    uint8_t i;
    for (i=0; i<N_buff; i++) {
        sum += temp_buffer[i];
    }
    
    return sum/N_buff/16;
}

// Set PWM
void set_PWM(uint16_t PWM) {
    PWM5DCL = PWM & 0xFFFC;
    PWM5DCH = PWM>>2;
}

// Update PWM
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
    
    // Set new PWM
    cur_PWM += K_p*err + K_i*cum_err;
    if (cur_PWM < 0) {
        cur_PWM = 0;
    } else if (cur_PWM > PWM_max) {
        cur_PWM = PWM_max;
    }

    prev_err = err;
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