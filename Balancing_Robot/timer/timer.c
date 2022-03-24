/*
 * timer.c
 *
 * Created: 2022-03-12 오후 9:55:32
 *  Author: purak
 */ 

#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include "time.h"

//Global Variable Define
volatile double count;

//16Bit Timer/Conuter1
//I/O Clock is 16MHz
void timer_setup(){
	TCCR1A = 0x00;
	TIMSK1 |= _BV(TOIE1);
	
	//Clcok Select I/O Clock
	//Prescaler is 8. So The Timer Clock is 16MHz/8 = 2MHz
	TCCR1B |= _BV(CS11);
	TCCR1B &= ~( _BV(CS12)  | _BV(CS10));
}

void get_time(double * dt){
	cli();
	uint8_t l = TCNT1L;
	uint8_t h = TCNT1H;
	uint16_t step = h<<8 | l;
	*dt = (double)step*5e-7 + count*0.032768;
	count = 0;
	sei();
}

SIGNAL(TIMER1_OVF_vect){
	//uart_putchar(count);
	count += 1;
	TCNT1H = 0x00;
	TCNT1L = 0x00;
}