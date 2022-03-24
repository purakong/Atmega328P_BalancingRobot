/*
 * timer.h
 *
 * Created: 2022-03-12 오후 9:55:41
 *  Author: purak
 */ 


#ifndef TIMER_H_
#define TIMER_H_

extern volatile double count;

void timer_setup();
void get_time(double * dt);


#endif /* TIMER_H_ */