


// AVR Send Character  string "Hello from ATmega328p" Continously to the PC using Serial port.
// PC receives the data and displays on terminal.
// External Oscillator = 11.0592MHz

//+------------------------------------------------------------------------------------------------+
//| Compiler           : AVR GCC (WinAVR)                                                          |
//| Microcontroller    : ATmega328p                                                                |
//| Programmer         : Rahul.Sreedharan                                                          |
//| Date               : 16-July-2019                                                              |
//+------------------------------------------------------------------------------------------------+

//(C) www.xanthium.in 
// Visit to Learn More 

// #include <stdint.h>
// #include <avr/io.h>
// #include <util/delay.h>

// // +-----------------------------------------------------------------------+ //
// // | ATmega328p Baudrate values for UBRRn for external crystal 11.0592MHz  | //
// // +-----------------------------------------------------------------------+ //
// #ifndef F_CPU
// #define F_CPU 16000000UL
// #endif 

// #define FOSC 16000000 // Clock Speed
// #define BAUD 9600
// #define MYUBRR FOSC/16/BAUD-1

// // // #define BAUD_RATE_4800_BPS  143 // 4800bps
// // // #define BAUD_RATE_9600_BPS  71  // 9600bps

// // // #define BAUD_RATE_14400_BPS  47 // 14.4k bps
// // // #define BAUD_RATE_19200_BPS  35 // 19.2k bps  
// // // #define BAUD_RATE_28800_BPS  23 // 28.8k bps
// // // #define BAUD_RATE_38400_BPS  17 // 38.4k bps
// // // #define BAUD_RATE_57600_BPS  11 // 57.6k bps
// // // #define BAUD_RATE_76800_BPS   8 // 76.8k bps

// // // #define BAUD_RATE_115200_BPS  5 // 115.2k bps
// // // #define BAUD_RATE_230400_BPS  2 // 230.4k bps

// // // +-----------------------------------------------------------------------+ //
// void USART_Init(unsigned char ubrr){
// 	/* Set Baudrate  */
// 	UBRR0H = (ubrr>>8); // Shift the 16bit value ubrr 8 times to the right and transfer the upper 8 bits to UBBR0H register.
// 	UBRR0L = (ubrr);    // Copy the 16 bit value ubrr to the 8 bit UBBR0L register,Upper 8 bits are truncated while lower 8 bits are copied
	
	

// 	UCSR0C = 0x06;       /* Set frame format: 8data, 1stop bit  */
// 	UCSR0B = (1<<TXEN0); /* Enable  transmitter                 */
// }


// int main()
// {
// 	int i = 0;
// 	// unsigned int ubrr = BAUD_RATE_9600_BPS;
	
// 	unsigned char data[] = "Hello from ATmega328p";
	
// 	USART_Init(MYUBRR);
	
	
// 	while(1) /* Loop the messsage continously */
// 	{ 
// 	    i = 0;
// 		while(data[i]!='\0') /* print the String  "Hello from ATmega328p" */
// 		{
// 			while (!( UCSR0A & (1<<UDRE0))); /* Wait for empty transmit buffer       */
			
// 											 /* When UDRE0 = 0,data transmisson ongoing.                         */
// 											 /* So NOT{[UCSR0A & (1<<UDRE0)] = 0} = 1 ,While(1) loop stays there */
											 
// 											 /* When UDRE0 = 1,data transmisson completed.                       */
// 											 /* So NOT{[UCSR0A & (1<<UDRE0)] = 1} = 0 ,While(0) loop fails       */
											 
// 			UDR0 = data[i];					 /* Put data into buffer, sends the data */
// 			i++;                             /* increment counter                    */
// 		}
// 	}
// // 		/*Nu il pun pe asta pentru ca vreau sa transmit date*/
// // 		// while ( !(UCSR0A & (1<<RXC0)) );
// // 		// /* Get and return received data from buffer */
// // 		// return UDR0;
		
		
// 		/* Sending '\n'  '\r' Character pair helps to format the output properly on console putty Screen */
// 		/*************************************************************************************************/
// 		/* Send "\n" Character */
// 		 while (!( UCSR0A & (1<<UDRE0)));   /* Wait for empty transmit buffer       */
// 		 UDR0 = '\n';					    /* Put data into buffer, sends the data */
		
// // 		/* Send "\r" Character */
// // 		 while (!( UCSR0A & (1<<UDRE0)));   /* Wait for empty transmit buffer       */
// // 		 UDR0 = '\r';					    /* Put data into buffer, sends the data */
// // 		/*------------------------------------------------------------------------------------------------*/
		
// 		_delay_ms(300);
		
		
// 	// }	
		
// }

  
/****************SENZOR ULTRASONIC DE DISTANCA CU TRANSMITERE CATRE CALCULATOR A REZULTATELOR + buzzer **********/
/*
Working: The library uses Timer 0 and Timer 1 of the Atmega328p. Timer 0 is used to generate PWM signals on PD6 to trigger the sensor. The echo pulse then generates a pulse whose width is measured by the Input Capture feature of Timer 1.*/
#ifndef F_CPU
#define F_CPU 16000000
#endif //F_CPU

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

#define FOSC 16000000 // Clock Speed
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1

static volatile uint32_t first_reading = 0;
static volatile uint32_t second_reading = 0;
static volatile uint32_t duty_cycle = 0;

void HCSR04_Init(void);
uint8_t getDistance(void);
void USART_Init(unsigned char ubrr);
void my_delay_ms(int ms)
{
  while (0 < ms)
  {  
    _delay_ms(1);
    --ms;
  }
}
 int main(void){
	USART_Init(MYUBRR);
	DDRB|= (1<<DDB5);
	char i,j=0;
	HCSR04_Init();
		while(1){
			char distance = getDistance();

			/*functia care blinkaie*/
			// j=distance;
			// if(distance){
			
			// 	PORTB|= (1<<PORTB5);
			// 	my_delay_ms(j);
			// 	PORTB&= ~(1<<PORTB5);
			// 	my_delay_ms(j);

				
			// }
 			while(distance!=0) /* print the String  "Hello from ATmega328p" */
				{
					/*UCSRnA – USART Control and Status Register n A . Pe bitul 5 avem UDREn register*/
/*If UDREn is one, the buffer is empty, and therefore ready to be written. The UDREn Flag can generate a Data Register 							Empty interrupt (see description of the UDRIEn bit).*/
					while (!( UCSR0A & (1<<UDRE0))); /* Wait for empty transmit buffer       */
					/*daca bitul udre0 nu e setat in registrul ucsr0a inseamna ca nu a terminat de trimis datele si atunci pt ca avem !(0) inseamna while(1) si sta in bucla infinita pana cand avem !(1) adica while(0). Asta se intampla cand e setat bitul de pe registrul ucsr0A, adica ca a trimis toate datele*/


/* [...]pg200- datasheetThe transmit buffer can only be written when the UDREn Flag in the UCSRnA Register is set. Data written to
UDRn when the UDREn Flag is not set, will be ignored by the USART Transmitter.- [...]*/													
					UDR0 = distance;					 /* Put data into data buffer UDRn, sends the data(in cazul asta).Daca voiam sa citim de la slave, tot prin registrul asta citeam pt ca asta e bufferul de la master */
					 distance=0;                           /* increment counter                    */
				}
				// _delay_ms(100);
		}
	
 }


 void USART_Init(unsigned char ubrr){
	/* UBBR = USART BAUD RATE REGISTER. Aici generezi baud rate-ul */
	/*UBBR e un registru de 16biti din care 4 is reserved, deci teoretic 12 dispo: 4 pe UBRRnH si 8 pe UBBRnL*/

	UBRR0H = (ubrr>>8); // Shift the 16bit value ubrr 8 times to the right and transfer the upper 8 bits to UBBR0H register.
	UBRR0L = (ubrr);    // Copy the 16 bit value ubrr to the 8 bit UBBR0L register,Upper 8 bits are truncated while lower 8 bits are copied
	
	/*Double Speed (asynchronous mode only) is controlled by the U2Xn found in the UCSRnA Register.Register for the XCKn pin (DDR_XCKn) controls whether the clock source is internal (Master mode) or external
(Slave mode). The XCKn pin is only active when using synchronous mode.*/

	UCSR0C = 0x06;       /* Registrul UCSR0B are registrul pentru: Set frame format: Setam UCSZ01 and UCZ02 cu 1 ca sa putem sa trimitem pachete de 8 biti, si 1 stop bit.Ultimii 2 biti sunt 0 ca vrea asyncronous comunication, vreau 1 stop bit */
	UCSR0B = (1<<TXEN0); /* Registrul UCSR0B are registrul pentru: Enable  transmitter                 */
}



/******************Initialize ultrasonic sensor**********************/
void HCSR04_Init(){
	cli(); //clear prior interrupts
	/*Fast PWM Configuration pentru ca vreau sa trimit 5v atata timp cat sa treaca 10uS ca senzorul sa intealeaga ca am trimis un puls.Idee e ca vrem sa configuram pwm-ul in asa fel incat in momentul in care atinge valoare de compare cu registrul de OCR0A sa corupa normal pin function. Deci doar atunci, nu cand face overflow. Facem asta pentru ca vrem sa trimitem semnale la precis 10 us si pentru restul timpului in care numara counter-ul sa asteptam */

	DDRD |= (1<<DDD6); //set PD6 as output pentru ca pe asta putem sa facem pwm fara interrupt

	//Set OC0A on Compare Match when up-counting. Clear OC0A on Compare Match when down-counting
	TCCR0A = (1<<COM0A1)|(1<<COM0A0)|(1<<WGM01)|(1<<WGM00);

	TCCR0B |= (1<<CS01); //prescaler = 8 for timer 0

	/*Setam prescaler de 8 pentru ca cu asta, cu un timer pe 8 biti putem sa setam o valoare in output compare register ca sa declansam in semnal odata la 10 microsecunde in real time (pentru ca la asta se asteapta comparatorul din interiorul senzorului, la un puls trimis odata la 10microsecunde)*/
/*De aia punem aici 200, pentru ca vrem cand incepe sa numere counterul, sa treaca 10uS*/
	OCR0A = 200; //10uS trigger pulse, 118uS off-time (128uS repetition rate)
	
	/*Input Capture configuration*/
	//Timer 1 running in normal mode
	DDRB &= ~(1<<DDB0); //PB0 as input (ICP1)
	TCCR1B = (1<<ICNC1)|(1<<ICES1)|(1<<CS11); //noise canceling + positive edge detection for input capture and Prescaler = 8.
	sei();//enable global interrupts
	TIMSK1 |= (1<<ICIE1); //enable timer1 input capture interrupt
}

uint8_t getDistance(){
	static uint32_t echo_pulse_uS;
	static uint8_t distance_cm;
        //32768uS = 65536 clock ticks for Timer 1 with prescaler = 8
	echo_pulse_uS = (float)duty_cycle * 32768 / 65536; //stiind cat timp a durat, impartim la jumtate din cat numara registrul timerului 1 ca sa aflam timpul real in microsecunde
	distance_cm = echo_pulse_uS * 0.034 / 2; //distanta e viteza ori timpul, iar viteza o facem 0,034 a sunetului pt ca vrem distanta in cm
	return distance_cm;
}

ISR(TIMER1_CAPT_vect){
	/*intra in interrupt in momentul in care PB0 primeste semnal, pentru ca am configurat timerul sa fie ca si input capture.Deci cand primeste semnal, atunci declanseaza un interrupt*/
	if ((TCCR1B & (1<<ICES1)) == (1<<ICES1)){ //cand s-a setat pinul ICES1 adica s-a vazut un semnal de input pe pB0
		first_reading = ICR1; //icr1 e registrul de unde copiem valoarea inputului de pe pb0. Si asta o stocam in first_reading
	}
	else{
		second_reading = ICR1; //daca nu avem input capture 
	}
	
	if (first_reading != 0 && second_reading != 0){
		duty_cycle = second_reading - first_reading; //vedem diferenta de timp cat i-a luat sa primeasca de la un semnal pana la urmatorul. Practic vedem cat timp a durat sa se intoarca inapoi sunetul
		first_reading = 0;
		second_reading = 0;
	}
	//dupa ce vedem cat a durat in ICR1 sa numere de prima data

	TCCR1B ^= (1<<ICES1); //toggle edge detection bit
	TIFR1 = (1<<ICF1);//clear Input Capture Flag
}





