/*
	Kim Tae Gyun
	2022-03
*/

#define F_CPU 16000000UL
#define BAUD 9600

#define ENABLE 1
#define FALSE  0
#define TESTCODE ENABLE

#include <inttypes.h>
#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include <avr/interrupt.h>
#include <math.h>

#include "mpu6050/mpu6050.h"
#include "mpu6050/mpu6050_reg.h"
#include "filter/kalmanFilter.h"
#include "i2c/i2c.h"
#include "uart/uart.h"
#include "timer/timer.h"
#include "pwm/pwm.h"

double accelAngelX, accelAngelY, accelAngelZ;
double gyroAngleX, gyroAngleY, gyroAngleZ;
double kalmanAngleX, kalmanAngleY, kalmanAngleZ;
double complementaryAngleX, complementaryAngleY, complementaryAngleZ;

int16_t averGyX, averGyY, averGyZ;

double dt;

#define RADIAN_TO_DEGREE 180.0/3.141592
#define COMPLMENTARY_ALPHA 0.96

void calibrationGyroSensor(gyroVal * data);

uint8_t ocr_count;

//OCR125 즉, 2.5V부터 모터가 돌아가기 시작함...
//125~255 모터 속도를 125단계로 조정가능함
int invlid_main(void)
{
	uart_init();
	
	DDRD |= _BV(PD5) | _BV(PD6);
	
	DDRD |= _BV(PD4) | _BV(PD3) | _BV(PD2) | _BV(PD7);
	
	PORTD |= (1 << PD4) | (1 << PD2);
	PORTD &= (~(1 << PD3) | (1 << PD7));
	
	TCCR0A |= _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00) | _BV(WGM01);
	TCCR0B |= _BV(CS00);

for(;;) {
		
		if(ocr_count == 0xff)
		{
			ocr_count = 0;
		}
		OCR0A = ocr_count;
		OCR0B = ocr_count;
		_delay_ms(500);
		
		UART_printUnsigned8bitNumber(ocr_count);
		
		ocr_count += 10;

				
		}
}

int main(void){
	uint8_t i2c_init_result;
	
	// gyro, accel Variable
	gyroVal gyro;
	accelVal accel;		
	gyroVal * p_gyro = &gyro;
	accelVal * p_accel = &accel;
	
	//KalmanFilter Variable
	Kalman kalmanX;
	Kalman kalmanY;
	Kalman * p_kalmanX = &kalmanX;
	Kalman * p_kalmanY = &kalmanY;
	
	double roll_pid_gain;
	
	//Interrupt Enable
	sei();
	
	DDRB |= _BV(5);
	
	//Initialize
	uart_init();
	i2c_init();
	pwm_init();
	
	
	//Kalman Variable Initialize
	Kalman_Init(p_kalmanX);
	Kalman_Init(p_kalmanY);
	
	//PID Variable Initialize
	PID_variableInit();
	
	//I2C Start Protocol Test
	i2c_init_result = i2c_start(MPU6050_ADDRESS+I2C_WRITE);
	i2c_start_check(i2c_init_result);

	//MPU6050 Initialize
	mpu6050_init();
	
	//Timer Initialize
	timer_setup();
	
	//Calibration Gyro Sensor
	UART_printString("Calibration Start!");
	UART_transmit(10); UART_transmit(10);
	calibrationGyroSensor(p_gyro);
	UART_printString("Calibration End");
	UART_transmit(10); UART_transmit(10);
	
	//Set Kalman and Gyro Starting Angle
	mpu6050_read_accel_ALL(p_accel);
	
	//accelAngelX is Roll
	accelAngelX = atan(p_accel->accel_y/sqrt(pow(p_accel->accel_x,2)+pow(p_accel->accel_z,2)));
	accelAngelX = accelAngelX * RADIAN_TO_DEGREE;
	Kalman_SetAngle(p_kalmanX, accelAngelX);

	//accelAngelY is Pitch
	accelAngelY = atan(-p_accel->accel_x/sqrt(pow(p_accel->accel_y,2)+pow(p_accel->accel_z,2)));
	accelAngelY = accelAngelY * RADIAN_TO_DEGREE;
	Kalman_SetAngle(p_kalmanY, accelAngelY);	
	
	while(1)
	{
		get_time(&dt);
		
		mpu6050_read_accel_ALL(p_accel);
		mpu6050_read_gyro_ALL(p_gyro);
		
		//Only use Acceleration Sensor
		accelAngelX = atan(p_accel->accel_y/sqrt(pow(p_accel->accel_x,2)+pow(p_accel->accel_z,2)));
		accelAngelX = accelAngelX * RADIAN_TO_DEGREE;
		accelAngelY = atan(-p_accel->accel_x/sqrt(pow(p_accel->accel_y,2)+pow(p_accel->accel_z,2)));
		accelAngelY = accelAngelY * RADIAN_TO_DEGREE;
		
		//Kalman Filter
		kalmanAngleX = Kalman_GetAngle(p_kalmanX,accelAngelX,((double)(p_gyro->gyro_x) / DEGREE_PER_SEC),dt);
		kalmanAngleY = Kalman_GetAngle(p_kalmanY,accelAngelY,((double)(p_gyro->gyro_y) / DEGREE_PER_SEC),dt);
		
		//Only use Gyro Sensor
		gyroAngleX += ((double)(p_gyro->gyro_x- averGyX) / DEGREE_PER_SEC) * dt;
		gyroAngleY += ((double)(p_gyro->gyro_y- averGyY) / DEGREE_PER_SEC) * dt;
		gyroAngleZ += ((double)(p_gyro->gyro_z- averGyZ) / DEGREE_PER_SEC) * dt;
		
		//PID Control
		PID_singlePID(kalmanAngleX,dt,&roll_pid_gain);

	
		//pmw_setMotorSpeed((uint8_t));
		pmw_setMotorSpeed((uint8_t)roll_pid_gain);
		
		/*
		UART_printString("Roll PID : ");
		UART_printDouble(roll_pid_gain,3);
		UART_printString("\t");
		UART_transmit(10);
		UART_transmit(10);
		*/
		/*		
		UART_printString("Kalman Angel X : ");
		UART_printDouble(kalmanAngleX,3);
		UART_printString("\t");
		
		UART_printString("PID ROLL GAIN : ");
		UART_printDouble(roll_pid_gain,3);
		UART_printString("\n");
		*/
		
		//Complimentary Filter
		//complementaryAngleX = COMPLMENTARY_ALPHA * gyroAngleX + (1.0 - COMPLMENTARY_ALPHA) * accelAngelX;

		//complementaryAngleY = COMPLMENTARY_ALPHA * gyroAngleY + (1.0 - COMPLMENTARY_ALPHA) * accelAngelY;
		
		
		
		
		//For the Debug
		//UART_AngelGyroPrint();
		/*
		UART_printString("Accel Angel X : ");
		UART_printDouble(accelAngelX,3);
		UART_printString("\t");
		UART_printString("Kalman Angel X : ");
		UART_printDouble(kalmanAngleX + 100.0,3);
		UART_printString("\t");
		UART_printString("Gyro Angel X : ");
		UART_printDouble(gyroAngleX + 200.0,3);
		UART_printString("\t");
		UART_printString("Gyro Angel X : ");
		UART_printDouble(complementaryAngleX + 300.0,3);
		UART_printString("\t");
		UART_transmit(10);
		UART_transmit(10);
		*/
	}
}

void UART_AngelAccelPrint(void)
{
	UART_printString("Angle Accel X :");
	UART_printDouble(accelAngelX,3);
	UART_printString("\t");
	UART_printString("Angle Accel Y :");
	UART_printDouble(accelAngelY,3);
	UART_printString("\t");
	//Transmit newline
	//10 = '\n'
	UART_transmit(10);UART_transmit(10);
}

void UART_AngelGyroPrint(void)
{
	UART_printString("Angle Gyro X :");
	UART_printDouble(gyroAngleX,3);
	UART_printString("\t");
	UART_printString("Angle GyroY :");
	UART_printDouble(gyroAngleY,3);
	UART_printString("\t");
	UART_printString("Angle GyroZ :");
	UART_printDouble(gyroAngleZ,3);
	UART_printString("\t");
	//Transmit newline
	//10 = '\n'
	UART_transmit(10);UART_transmit(10);
}

void UART_RAWAngelGyroPrint(gyroVal * data)
{
	UART_printString("RAW Gyro X :");
	UART_printSigned16bitNumber(data->gyro_x- averGyX);
	UART_printString("\t");
	UART_printString("RAW GyroY :");
	UART_printSigned16bitNumber(data->gyro_y- averGyY);
	UART_printString("\t");
	UART_printString("RAW GyroZ :");
	UART_printSigned16bitNumber(data->gyro_z- averGyZ);
	UART_printString("\t");
	//Transmit newline
	//10 = '\n'
	UART_transmit(10);UART_transmit(10);
}

void calibrationGyroSensor(gyroVal * data)
{
  int16_t sumGyX = 0 , sumGyY = 0, sumGyZ = 0;
  
  for (int i=0;i<100;i++) {
	  mpu6050_read_gyro_ALL(data);
	  sumGyX+=data->gyro_x;  sumGyY+=data->gyro_y;  sumGyZ+=data->gyro_z;
	  _delay_ms(50);
  }
  
  //The average value of number 10
  averGyX=sumGyX/100;  
  averGyY=sumGyY/100;  
  averGyZ=sumGyZ/100;
}


