/*
 * pwm.c
 *
 * Created: 2022-03-20 오후 4:39:25
 *  Author: purak
 */ 

#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include "time.h"

//8Bit Timer/Counter
//I/O Clock is 16MHz
void pwm_init()
{
	//PD5's output is Timer0-B, PD6's output is Timer0-A
	DDRD |= _BV(PD5) | _BV(PD6);
	
	DDRD |= _BV(PD4) | _BV(PD3) | _BV(PD2) | _BV(PD7);
	
	PORTD |= (1 << PD4) | (1 << PD2);
	PORTD &= (~(1 << PD3) | (1 << PD7));
	
	//Fast PWM Mode
	//Prescaler is 0
	//Output Compare & Non-inverted Mode
	TCCR0A |= _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00) | _BV(WGM01);
	TCCR0B |= _BV(CS00);
}

void pmw_setMotorDirection(uint8_t dir)
{
	if(dir == 0)
	{
		PORTD = (1 << PD4) | (1 << PD7);
		PORTD &= ~((1 << PD3) | (1 << PD2));
	}
	else if(dir == 1)
	{
		PORTD = (1 << PD3) | (1 << PD2);
		PORTD &= ~((1 << PD4) | (1 << PD7));
	}
	else
	{
		//do nothing
	}
}

void pmw_setMotorSpeed(int8_t speed)
{
	//120 is speed 0
	//255 is max speed
	uint8_t offset = 130;
	
	if(speed>0)
	{
		pmw_setMotorDirection(1);
		speed = speed + offset;	
	}
	else if(speed<0)
	{
		pmw_setMotorDirection(0);
		speed = speed - offset;
		speed = (-speed);
	}
	
	if(speed >= 255)
	{
		speed = 255;
	}
	
	OCR0A = speed;
	OCR0B = speed;
}
