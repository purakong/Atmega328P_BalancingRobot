/*
 * PID.h
 *
 * Created: 2022-03-20 오후 4:59:28
 *  Author: purak
 */ 


#ifndef PID_H_
#define PID_H_

extern void PID_variableInit(void);
extern void PID_singlePID(double angleX, double dt, double * pid_gain)



#endif /* PID_H_ */