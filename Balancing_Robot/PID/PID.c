/*
 * PID.c
 *
 * Created: 2022-03-20 오후 4:59:19
 *  Author: purak
 */ 

#include <inttypes.h>
#include "../uart/uart.h"

double roll_Kp;
double roll_Ki;
double roll_Kd;

double roll_p;
double roll_i;
double roll_d;

double roll_reference; 
double roll_error = 0;
double roll_error_sum = 0;

double roll_derivative;

double roll_pid;

double lastAngleX = 0;

void PID_variableInit(void)
{
	roll_reference = 0.0;
	roll_Kp = 4;
	roll_Ki = 0.02;
	roll_Kd = 0.4;
}

void PID_singlePID(double angleX, double dt, double * pid_gain)
{
	//Proportional
	roll_error = roll_reference - angleX;
	roll_p = roll_error * roll_Kp;
	
	//Integral
	roll_error_sum = roll_error_sum + roll_error * 0.001;
	roll_i = roll_error_sum * roll_Ki;
	
	
	//Derivative
	roll_derivative = (angleX -lastAngleX)/dt;
	roll_d = -roll_derivative * roll_Kd;
	
	roll_pid = roll_p + roll_i + roll_d;
	
	*pid_gain = roll_pid;
	
	/*UART_printString("Roll PID : ");
	UART_printDouble(roll_pid,3);
	UART_printString("\t");
	UART_transmit(10);
	UART_transmit(10);
	*/
	
	lastAngleX = angleX;
}

//doubllePID(void)