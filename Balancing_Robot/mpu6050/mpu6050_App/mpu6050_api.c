/*
 * mpu6050_api.c
 *
 * Created: 2022-03-12 오후 4:09:52
 *  Author: purak
 */ 
#include "mpu6050_api.h"
#include "../mpu6050.h"
#include <math.h>

//Euler Unit is Radian
//This function is for getting Euler Angle using mpu6050 acceleration value
//Angle X is Phi (Roll)
//Angle Y is theta (Pitch)
void MPU6050_AccelToEuler(accelVal * p_accel, eulerAngle * e_angle, degreeAngle * d_angle)
{
		mpu6050_read_accel_ALL(p_accel);
		
		//Angle X
		e_angle->euler_phi = atan(p_accel->accel_y/sqrt(pow(p_accel->accel_x,2)+pow(p_accel->accel_z,2)));
		d_angle->degree_phi = e_angle->euler_phi * RADIAN_TO_DEGREE;
		
		//Angle Y
		e_angle->euler_theta = atan(-p_accel->accel_x/sqrt(pow(p_accel->accel_y,2)+pow(p_accel->accel_z,2)));
		d_angle->degree_theta = e_angle->euler_theta * RADIAN_TO_DEGREE;
}
/*
void MPU6050_KalmanFilter(void)
{
	//Step 1 (Prediction)
	//Calculation state variable
	gyroRate = gyroNewRate - gyroBias;
	gyroAngle += dt*gyroRate;
	
	//Step 2 (Prediction)
	//Calcaulation Pk-
	P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
	P[0][1] -= dt * P[1][1];
	P[1][0] -= dt * P[1][1];
	P[1][1] += Q_gyroBias * dt;
	
	//Step3 (Calculation)
	//Calculation yk
	y = gyroNewAngle - gyroAngle;
	
	//Step4,5 (Prediction)
	//Calculation Kalman Gain
	S = P[0][0] + R_measure;
	K[0] = P[0][0] / S;
	K[1] = P[1][0] / S;
	
	//Step6 (Calculation)
	//Calculation Final Calcaulation Value
	gyroAngle += K[0] * y;
	gyroBias += K[1] * y;
	
	//Step7 (Prediction)
	//Calculation P
	float P00_temp = P[0][0];
	float P01_temp = P[0][1];
	
	P[0][0] -= K[0] * P00_temp;
	P[0][1] -= K[0] * P01_temp;
	P[1][0] -= K[1] * P00_temp;
	P[1][1] -= K[1] * P01_temp;
}
*/
void Init(Kalman* klm){

    /* We will set the variables like so, these can also be tuned by the user */

    klm->Q_angle = 0.001;

    klm->Q_bias = 0.003;

    klm->R_measure = 0.03;



    klm->angle = 0; // Reset the angle

    klm->bias = 0; // Reset bias

   klm->P[0][0] = 0; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en->wikipedia->org/wiki/Kalman_filter#Example_application->2C_technical

    klm->P[0][1] = 0;

    klm->P[1][0] = 0;

    klm->P[1][1] = 0;

}



// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds

double getAngle(Kalman * klm, double newAngle, double newRate, double dt) {

    // KasBot V2  -  Kalman filter module - http://www->x-firm->com/?page_id=145

    // Modified by Kristian Lauszus

   // See my blog post for more information: http://blog->tkjelectronics->dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it



    float P00_temp;

    float P01_temp;


    // Discrete Kalman filter time update equations - Time Update ("Predict")

    // Update xhat - Project the state ahead

    /* Step 1 */

    klm->rate = newRate - klm->bias;

    klm->angle += dt * klm->rate;

    

    // Update estimation error covariance - Project the error covariance ahead

    /* Step 2 */

    klm->P[0][0] += dt * (dt*klm->P[1][1] - klm->P[0][1] - klm->P[1][0] + klm->Q_angle);

    klm->P[0][1] -= dt * klm->P[1][1];

    klm->P[1][0] -= dt * klm->P[1][1];

    klm->P[1][1] += klm->Q_bias * dt;

    

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")

    // Calculate Kalman gain - Compute the Kalman gain

    /* Step 4 */

    klm->S = klm->P[0][0] + klm->R_measure;



    /* Step 5 */

    klm->K[0] = klm->P[0][0] / klm->S;

    klm->K[1] = klm->P[1][0] / klm->S;

    

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)

    /* Step 3 */

    klm->y = newAngle - klm->angle;



    /* Step 6 */

    klm->angle += klm->K[0] * klm->y;

    klm->bias += klm->K[1] * klm->y;

    

    // Calculate estimation error covariance - Update the error covariance

    /* Step 7 */

   P00_temp = klm->P[0][0];

   P01_temp = klm->P[0][1];



    klm->P[0][0] -= klm->K[0] * P00_temp;

    klm->P[0][1] -= klm->K[0] * P01_temp;

    klm->P[1][0] -= klm->K[1] * P00_temp;

    klm->P[1][1] -= klm->K[1] * P01_temp;



    return klm->angle;

}



 // Used to set angle, this should be set as the starting angle

void setAngle(Kalman* klm, double newAngle) { klm->angle = newAngle; }

// Return the unbiased rate

double getRate(Kalman* klm) { return klm->rate; }
 
/* These are used to tune the Kalman filter */

void setQangle(Kalman* klm, double newQ_angle) { klm->Q_angle = newQ_angle; }



/* Default value is (0.003f), raise this to follow input more closely, lower this to smooth result of kalman filter */

void setQbias(Kalman* klm, double newQ_bias) { klm->Q_bias = newQ_bias; }



void setRmeasure(Kalman* klm, double newR_measure) { klm->R_measure = newR_measure; }

double getQangle(Kalman* klm) { return klm->Q_angle; }

double getQbias(Kalman* klm) { return klm->Q_bias; }

double getRmeasure(Kalman* klm) { return klm->R_measure; }