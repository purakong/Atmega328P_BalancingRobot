/*
 * mpu6050.h
 *
 * Created: 2022-03-12 오후 4:10:03
 *  Author: purak
 */ 


#ifndef MPU6050_API_H_
#define MPU6050_API_H_

#define RADIAN_TO_DEGREE 180/3.141592

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

void KalmanInit(Kalman* klm);

double getAngle(Kalman * klm, double newAngle, double newRate, double dt);

// Used to set angle, this should be set as the starting angle
void setAngle(Kalman* klm, double newAngle);

// Return the unbiased rate
double getRate(Kalman* klm);

/* These are used to tune the Kalman filter */
void setQangle(Kalman* klm, double newQ_angle);


/* Default value is (0.003f), raise this to follow input more closely, lower this to smooth result of kalman filter */
void setQbias(Kalman* klm, double newQ_bias);

void setRmeasure(Kalman* klm, double newR_measure);

double getQangle(Kalman* klm);

double getQbias(Kalman* klm);

double getRmeasure(Kalman* klm);




#endif /* MPU6050_API_H_ */