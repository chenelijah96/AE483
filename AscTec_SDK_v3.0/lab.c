/*
 * lab.c
 *
 *  Created on: Jan 26, 2015
 *      Author: hanley6
 */

#include "lab.h"
#include "main.h"
#include "sdk.h"
#include "math.h"

// PARAMETERS FROM ACI TOOL
float k1 = 1000.0;					// first element of 1x2 matrix K
float k2 = 1000.0;					// second element of 1x2 matrix K
float period = 5;				// seconds

// ALL OTHER PARAMETERS
float kF = 4.5508e-6;					// motor constant (squared spin rate to force)
float alpha = 3.8467;				// for converting spin rate to motor command
float beta = 100.5216;					// for converting spin rate to motor command
float l = 0.17145;					// spar length (m)
float m = 0.7092;					// mass (kg)
//float m = 0.5;
float g = 9.81;					// acceleration of gravity (m / s^2)
// - for computing desired pitch angle (sinusoidal with time)
float input_amplitude = 0;		// amplitude of sinusoid (rad)
int sample_rate = 1000;			// number of times per second that control loop runs (Hz)

// VARIABLES TO BE SENT OFF-BOARD
int counter = 0;				// number of times through control loop
float angle_pitch_desired = 0;	// desired pitch angle (rad)
int mu1 = 0;					// motor command applied to rotor 1
int mu2 = 0;					// motor command applied to rotor 2

// ALL OTHER VARIABLES
float t = 0;					// current on-board time (s)
float angle_pitch = 0;			// current pitch angle (rad)
float angvel_pitch = 0;			// current pitch angular velocity (rad / s)
float u2 = 0;					// total torque due to rotors about y axis
float u4 = 0;					// total force due to rotors along -z axis
float f1 = 0;					// force due to rotor 1 (front, along +x)
float f2 = 0;					// force due to rotor 2 (back, along -x)
float sigma1 = 0;				// spin rate of rotor 1 (rad / s)
float sigma2 = 0;				// spin rate of rotor 2 (rad / s)

void lab(void) {

	//
	// DO NOT CHANGE (SHOULD ALREADY BE HERE FROM LAB 1)
	//

	// increment counter
	counter++;

	//
	// CHANGE ALL OF THIS
	//

	// get current time (from counter and sample_rate)
	t = ((float) counter) / ((float) sample_rate);

	// get desired pitch angle (from input_amplitude, t, and period)
	angle_pitch_desired = ((float) (input_amplitude / 100.0)) * sin(( 2.0*M_PI ) / ((float) period) * t);

	// get pitch angle and angular velocity from sensor data:
	// 		RO_ALL_Data.angle_pitch
	//		RO_ALL_Data.angvel_pitch
	angle_pitch = ((float) RO_ALL_Data.angle_pitch) * 0.001 *  M_PI / 180.0;
	angvel_pitch = ((float) RO_ALL_Data.angvel_pitch) * 0.0154 *  M_PI / 180.0;

	// get total torque due to rotors about y axis
	// from k1, k2, angle_pitch, angle_pitch_desired, and angvel_pitch
	u2 = (-k1 / 1000.0)*(angle_pitch - angle_pitch_desired) - ((k2 / 1000.0) * angvel_pitch);

	// get total force due to rotors along -z axis
	// from m, g, and angle_pitch
	u4 = (m * g) / cos(angle_pitch);

	// get forces from u2, u4, and l
	f1 = (u4 / 2.0) + (u2 / (2.0*l));
	f2 = (u4 / 2.0) - (u2 / (2.0*l));

	// get spin rates from f1, f2, and kF
	sigma1 = sqrt(f1 / kF);
	sigma2 = sqrt(f2 / kF);

	// get motor commands from alpha, beta, sigma1, and sigma2
	mu1 = (sigma1 - beta) / alpha;
	mu2 = (sigma2 - beta) / alpha;

	//
	// DO NOT CHANGE
	//

	// bound motor commands (VERY IMPORTANT - DO NOT CHANGE)
	if (mu1 > 200) {
		mu1 = 200;
	} else if (mu1 < 1) {
		mu1 = 1;
	}
	if (mu2 > 200) {
		mu2 = 200;
	} else if (mu2 < 1) {
		mu2 = 1;
	}

	// make sure motor command settings are correct (VERY IMPORTANT - DO NOT CHANGE)
	WO_SDK.ctrl_mode = 0x00;	// 0x00: direct individual motor control
	WO_SDK.ctrl_enabled = 1;	// 1: enable control by higher-level processor (this code)

	// apply motor commands (numbers are different because C starts index at 0)
	WO_Direct_Individual_Motor_Control.motor[0] = mu1;
    WO_Direct_Individual_Motor_Control.motor[1] = mu2;
    WO_Direct_Individual_Motor_Control.motor[2] = 0;
	WO_Direct_Individual_Motor_Control.motor[3] = 0;
}

