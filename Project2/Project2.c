/*
 * Project2.c
 *
 *  Created on: Sep 17, 2021
 *      Author: Sama Mostafa
 */
#include <avr/io.h>
#include <avr/interrupt.h>
# include<util/delay.h>
unsigned char sec = 0, min = 0, hour = 0;
void INT0_Reset(void) {
	DDRD &= ~(1 << PD2); // Configure INT0/PD2 as input pin
	PORTD |= (1 << PD2); // use internal pull up
	GICR |= (1 << INT0); // Enable external interrupt pin INT0
	MCUCR |= (1 << ISC01); // Trigger INT0 with the falling edge
}

void INT1_pause(void) {
	DDRD &= ~(1 << PD3); // Configure INT1/PD3 as input pin
	GICR |= (1 << INT1); // Enable external interrupt pin INT1
	MCUCR |= (1 << ISC10) | (1 << ISC11); // Trigger INT1 with the rising edge
}
void INT2_Resume(void) {
	DDRB &= ~(1 << PB2); // Configure INT2/PB2 as input pin
	PORTB |= (1 << PB2); // use internal pull up
	GICR |= (1 << INT2); // Enable external interrupt pin INT2
	MCUCR |= (1 << ISC2); // Trigger INT2 with the falling edge
}
void Timer1_Init_CTC_Mode(void) {
	TCNT1 = 0; /* Set timer1 initial count to zero */
	OCR1A = 15625; /* Set the Compare value to 15625*/
	TIMSK |= (1 << OCIE1A); /* Enable Timer1 Compare A Interrupt */
	/* Configure timer control register TCCR1A
	 * 1. Disconnect OC1A and OC1B  COM1A1=0 COM1A0=0 COM1B0=0 COM1B1=0
	 * 2. FOC1A=1 FOC1B=0
	 * 3. CTC Mode WGM10=0 WGM11=0 (Mode Number 4)
	 */
	TCCR1A = (1 << FOC1A);
	/* Configure timer control register TCCR1B
	 * 1. CTC Mode WGM12=1 WGM13=0 (Mode Number 4)
	 * 2. Prescaler = F_CPU/64 CS10=1 CS11=1 CS12=0
	 */
	TCCR1B = (1 << WGM12) | (1 << CS10) | (1 << CS11);

}
/* Interrupt Service Routine for interrupt 0 used to reset the timer */
ISR(INT0_vect) {
	sec = 0;
	min = 0;
	hour = 0;
}
/* Interrupt Service Routine for interrupt 1 used to pause the timer */
ISR(INT1_vect) {
	if (PIND & (1 << PD3)) {
		TCCR1B = (1 << WGM12); //turn off the clock to pause
	}
}
/* Interrupt Service Routine for interrupt 2 used to resume the timer */
ISR(INT2_vect) {
	TCCR1B = (1 << WGM12) | (1 << CS10) | (1 << CS11); //turn on the clock to resume
}
/* Interrupt Service Routine for timer1 compare mode */
ISR(TIMER1_COMPA_vect) {
	/*the clock return to zero when second and minutes is greater than 59
	 and hours is greater than 23*/
	if (sec == 59) {
		sec = 0;
		if (min == 59) {
			min = 0;
			if (hour == 23) {
				hour = 0;
			} else {
				hour++;
			}
		} else {
			min++;
		}
	} else {
		sec++;
	}
}
int main(void) {
	DDRA |= 0x3F; //configure first 6 pin in PORTA as output bin
	PORTA |= 0X00; //set value to 0
	DDRC = 0x0F; //configure first 4 pin in PORTc as output bin
	PORTC = 0xF0; //set value to 0
	SREG |= (1 << 7); //enable global interrupt enable
	INT0_Reset();
	INT1_pause();
	INT2_Resume();
	Timer1_Init_CTC_Mode();
	while (1) {
		PORTA = (1 << PA5); //set pin 5 only to one
		PORTC = (PORTC & 0xF0) | ((sec % 10) & 0x0F); //to set units of seconds in 7segment
		_delay_ms(5);
		PORTA = (1 << PA4); //set pin 4 only to one
		PORTC = (PORTC & 0xF0) | ((sec / 10) & 0x0F); //to set tens of seconds in 7segment
		_delay_ms(5);
		PORTA = (1 << PA3); //set pin 3 only to one
		PORTC = (PORTC & 0xF0) | ((min % 10) & 0x0F); //to set units of minutes in 7segment
		_delay_ms(5);
		PORTA = (1 << PA2); //set pin 2 only to one
		PORTC = (PORTC & 0xF0) | ((min / 10) & 0x0F); //to set tens of minutes in 7segment
		_delay_ms(5);
		PORTA = (1 << PA1); //set pin 1 only to one
		PORTC = (PORTC & 0xF0) | ((hour % 10) & 0x0F); //to set units of hours in 7segment
		_delay_ms(5);
		PORTA = (1 << PA0); //set pin 0 only to one
		PORTC = (PORTC & 0xF0) | ((hour / 10) & 0x0F); //to set tens of hours in 7segment
		_delay_ms(5);
	}
}

