/*
 * kalmanFilter.h
 *
 * Created: 2022-03-12 오후 4:10:03
 *  Author: KimTaeKyun
 */ 


#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_

typedef struct _eulerAngle{
	double euler_phi;
	double euler_theta;
}eulerAngle;

typedef struct _degreeAngle{
	double degree_phi;
	double degree_theta;
}degreeAngle;

typedef struct _Kalman {

	/* Kalman filter variables */

	double Q_angle; // Process noise variance for the accelerometer
	double Q_bias; // Process noise variance for the gyro bias
	double R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

	double angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
	double bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
	double rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

	double P[2][2]; // Error covariance matrix - This is a 2x2 matrix
	double K[2]; // Kalman gain - This is a 2x1 vector
	double y; // Angle difference
	double S; // Estimate error
}Kalman;

void Kalman_Init(Kalman* klm);

double Kalman_GetAngle(Kalman * klm, double newAngle, double newRate, double dt);

// Used to set angle, this should be set as the starting angle
void Kalman_SetAngle(Kalman* klm, double newAngle);

// Return the unbiased rate
double Kalman_GetRate(Kalman* klm);

/* These are used to tune the Kalman filter */
void Kalman_SetQangle(Kalman* klm, double newQ_angle);


/* Default value is (0.003f), raise this to follow input more closely, lower this to smooth result of kalman filter */
void Kalman_SetQbias(Kalman* klm, double newQ_bias);

void Kalman_SetRmeasure(Kalman* klm, double newR_measure);

double Kalman_GetQangle(Kalman* klm);

double Kalman_GetQbias(Kalman* klm);

double Kalman_GetRmeasure(Kalman* klm);




#endif /* MPU6050_API_H_ */