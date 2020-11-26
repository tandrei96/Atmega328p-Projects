
#ifndef F_CPU					// if F_CPU was not defined in Project -> Properties
#define F_CPU 16000000UL			// define it now as 1 MHz unsigned long
#endif

#include <avr/io.h>				// this is always included in AVR programs
#include <avr/interrupt.h>
#include <util/delay.h>

double dutycycle=0;
int main(void) {
	
	DDRB |= (1 << DDB5) ;			// set PC5 (pin 28) and PC4 (pin 27) for output
	/*
	TCCR0A - Timer/Counter 0 Control Register A
	
	bit           7         6         5         4        3       2        1        0
	name        COM0A1    COM0A0    COM0B1    COM0B0     -       -      WGM01    WGM00
	set to        0         0         0         0        0       0        1        0
	
	COM0A1 = 0    normal port operation, OC0A disconnected
	COM0A0 = 0
	
	COM0B1 = 0    normal port operation, OC0B disconnected
	COM0B0 = 0
	
	bit 3 = 0
	bit 2 = 0
	
	WGM01 = 1     CTC (Clear Timer on Compare match) mode, see TCCR0B also
	WGM00 = 0     TCNT0 will count up to value in OCR0A, then signal timer 0 compare interrupt
	*/
	TCCR0A = 0b10000011;
	

	
	/*
	TIMSK0 - Timer/Counter 0 Interrupt Mask Register
	
	bit           7        6        5       4       3       2         1         0
	name          -        -        -       -       -     OCIE0B    OCIE0A    TOIE0
	set to        0        0        0       0       0       0         1         0
	
	bit 7 = 0     don't use Force Output Compare A
	bit 6 = 0
	bit 5 = 0
	bit 4 = 0
	bit 3 = 0
	OCIE0B = 0    don't enable Timer/Counter 0 Output Compare Match B Interrupt
	OCIE0A = 1    enable Timer/Counter 0 Output Compare Match A Interrupt Enable
	TOIE0 = 0     don't enable Timer/Counter 0 Overflow Interrupt
	*/
	TIMSK0 = 0b00000001;
	
	
	
	OCR0A = (dutycycle/100.0)*255.0;		// set compare register to low value to produce fast PWM
	
	sei();				// enable interrupts
		/*
	TCCR0B - Timer/Counter 0 Control Register B
	
	bit           7          6        5       4         3         2         1        0
	name        FOC0A      FOC0B      -       -       WGM02      CS02      CS01     CS00
	set to        0          0        0       0         0         0         1        1
	
	FOC0A = 0     don't use Force Output Compare A
	FOC0B = 0
	
	bit 5 = 0
	bit 4 = 0
	
	WGM02 = 0     CTC (Clear Timer on Compare match) mode, see TCCR0A also
	
	CS02 = 0
	CS01 = 1      clock / 64, should not use less than this to allow ISR enough clock cycles
	CS00 = 1
	*/
	TCCR0B = 0b00001001;
	
	while (1) {
			_delay_ms(15);
			dutycycle+=2;
			OCR0A = (dutycycle/100.0)*255.0;		// set compare register to low value to produce fast PWM

			if(dutycycle==90){
				_delay_ms(30);
			dutycycle=0;
			}
	}
	
	return(0);					
}

////////////////////Interrupt Service Routine//////////////////////////////
ISR(TIMER0_OVF_vect) {
	PORTB ^= (1<<PORTB5);
	
	
}


