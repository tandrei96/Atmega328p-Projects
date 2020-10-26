#include <avr/io.h>
#include <avr/interrupt.h>
#include<avr/delay.h>

uint8_t cnt=0; //asta va stoca de cate ori fac overflow
void Setup_timer (void);
void Setup_External_interrupt(void);


int main(void){
     DDRB |=(1<<DDB5); //set output doar pe DDRB pin 5  
      
    Setup_timer();
    Setup_External_interrupt();
    sei();

         while(1){
             asm ("nop");
         }
}
    void Setup_External_interrupt(void){
        EICRA |= (1<<ISC01);
        EICRA &= ~(1<<ISC00); //enable interrupt on a falling edge
        EIMSK |= (1<<INT0);     //enable external interrupt on a falling edge on PD2
    }
        
    void Setup_timer(void){
            /*setup timer overflow*/

            TCCR0A &= ~(3<<WGM00); //set normal mode. set wgm00 si wgm01 cu 1 dar dupaia aplic TCCR0A and not si astfel ii fac 0 pe bitii aia
            
            TIMSK0 |= (1<<TOIE0);  //set interrupt on overflow

            TCCR0B |= (5<<CS00); //set prescaler de 1024
      
            /*• Bit 0 – TOIE0: Timer/Counter0 Overflow Interrupt Enable
When the TOIE0 bit is written to one, and the I-bit in the Status Register is set (adica comanda sei() asta face), the Timer/Counter0 Overflow interrupt is enabled. The corresponding interrupt is executed if an overflow in Timer/Counter0 occurs, i.e., when the TOV0 bit is set in the Timer/Counter 0 Interrupt Flag Register – TIFR0.*/
            
                /*setup timer interrupt on compare with 194 value*/

    }
        
        
        
         ISR(INT0_vect){
             PORTB |= (1<<PORTB5);
        }

        ISR(TIMER0_OVF_vect){
            cnt++;
                if (cnt==61){
                PORTB ^=(1<<PORTB5); //executes at a rate of 1 Hz   
                cnt=0; //reset sw counter
                }
        }

        


