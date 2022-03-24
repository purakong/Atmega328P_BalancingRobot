/*
 * pwm.h
 *
 * Created: 2022-03-20 오후 4:39:33
 *  Author: purak
 */ 


#ifndef PWM_H_
#define PWM_H_

extern void pwm_init();
extern void pmw_setMotorDirection(uint8_t dir);
extern void pmw_setMotorSpeed(uint8_t speed);


#endif /* PWM_H_ */