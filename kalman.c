#include "kalman.h"

Kalman *newKalman() {
	Kalman *k = NULL;
	k = (Kalman *) malloc(sizeof(Kalman));

	/* We will set the variables like so, these can also be tuned by the user */
	k->Q_angle = 0.001f;
	k->Q_bias = 0.003f;
	k->R_measure = 0.03f;

	k->angle = 0.0f; // Reset the angle
	k->bias = 0.0f; // Reset bias

	k->P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
	k->P[0][1] = 0.0f;
	k->P[1][0] = 0.0f;
	k->P[1][1] = 0.0f;
	return k;
}

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float getAngle(Kalman *k, float newAngle, float newRate, float dt) {
	// KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
	// Modified by Kristian Lauszus
	// See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

	// Discrete Kalman filter time update equations - Time Update ("Predict")
	// Update xhat - Project the state ahead
	/* Step 1 */
	k->rate = newRate - k->bias;
	k->angle += dt * k->rate;

	// Update estimation error covariance - Project the error covariance ahead
	/* Step 2 */
	k->P[0][0] += dt * (dt * k->P[1][1] - k->P[0][1] - k->P[1][0] + k->Q_angle);
	k->P[0][1] -= dt * k->P[1][1];
	k->P[1][0] -= dt * k->P[1][1];
	k->P[1][1] += k->Q_bias * dt;

	// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
	// Calculate Kalman gain - Compute the Kalman gain
	/* Step 4 */
	float S = k->P[0][0] + k->R_measure; // Estimate error
	/* Step 5 */
	float K[2]; // Kalman gain - This is a 2x1 vector
	K[0] = k->P[0][0] / S;
	K[1] = k->P[1][0] / S;

	// Calculate angle and bias - Update estimate with measurement zk (newAngle)
	/* Step 3 */
	float y = newAngle - k->angle; // Angle difference
	/* Step 6 */
	k->angle += K[0] * y;
	k->bias += K[1] * y;

	// Calculate estimation error covariance - Update the error covariance
	/* Step 7 */
	float P00_temp = k->P[0][0];
	float P01_temp = k->P[0][1];

	k->P[0][0] -= K[0] * P00_temp;
	k->P[0][1] -= K[0] * P01_temp;
	k->P[1][0] -= K[1] * P00_temp;
	k->P[1][1] -= K[1] * P01_temp;

	return k->angle;
}

// Used to set angle, this should be set as the starting angle
void setAngle(Kalman *k, float angle) {
	k->angle = angle;
}

// Return the unbiased rate
float getRate(Kalman *k) {
	return k->rate;
}

/* These are used to tune the Kalman filter */
void setQangle(Kalman *k, float Q_angle) {
	k->Q_angle = Q_angle;
}

void setQbias(Kalman *k, float Q_bias) {
	k->Q_bias = Q_bias;
}

void setRmeasure(Kalman *k, float R_measure) {
	k->R_measure = R_measure;
}

float getQangle(Kalman *k) {
	return k->Q_angle;
}

float getQbias(Kalman *k) {
	return k->Q_bias;
}

float getRmeasure(Kalman *k) {
	return k->R_measure;
}
