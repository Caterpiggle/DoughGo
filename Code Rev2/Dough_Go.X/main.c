#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>
#include "Dough_Go_8bit.h"

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
        
        PWM_max = 205; // 20% DC
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