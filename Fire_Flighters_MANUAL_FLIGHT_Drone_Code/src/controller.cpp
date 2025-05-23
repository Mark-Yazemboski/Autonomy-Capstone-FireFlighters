#include "controller_ref.h"
#include "controller.h"

#include <iostream>
#include <Arduino.h>
#include <cmath>

#define MIN_PWM 1000
#define MAX_PWM 2000

#ifndef C_PI
#define C_PI 3.14159265358979323846264338327950288419716939937511
#endif
#define LIMIT(x,xl,xu) ((x)>=(xu)?(xu):((x)<(xl)?(xl):(x)))

double hmodRad(double h) {

	double dh;
	int i;

	if (h > 0)
		i = (int)(h / (2 * C_PI) + 0.5);
	else
		i = (int)(h / (2 * C_PI) - 0.5);
	dh = h - C_PI * 2 * i;

	return dh;
}

void updateControl(const float dt, const float phiCmd,
	const float posDes_x, const float posDes_y, const float posDes_z,
	const float nav_p_x, const float nav_p_y,   const float nav_p_z,
	const float nav_v_x, const float nav_v_y,   const float nav_v_z,
	const float nav_w_x, const float nav_w_y,   const float nav_w_z,
	const float nav_phi, const float nav_theta, const float nav_psi,
	const float nav_a_x, const float nav_a_y, const float nav_a_z,
	float* c_delf, float* c_delm0, float* c_delm1, float* c_delm2,float Throttle,float Roll,float Pitch,float Yaw,bool Manual, bool Killed)
{
	struct onboardControl_ref* cntrl = &onboardControl; // controller


	if (Killed){
		cntrl->Angle_integral[0] = 0;
		cntrl->Angle_integral[1] = 0;
		cntrl->Angle_integral[2] = 0;

		*c_delf = -1;
		*c_delm0 = 0;
		*c_delm1 = 0;
		*c_delm2 = 0;
	}else{





		if (Manual) {

			float des_roll_angle = Roll*20*PI/180; 
			float des_pitch_angle = Pitch*20*PI/180;
			float des_yaw_rate = Yaw*.3;

			float Error_Roll_Angle = des_roll_angle - nav_phi;
			float Error_Pitch_Angle = des_pitch_angle - nav_theta;

			float Error_Yaw_Rate = des_yaw_rate - ((nav_w_z));

			float Derivative_Angle_Roll = nav_w_x;
			float Derivative_Angle_Pitch = nav_w_y;


			cntrl->Angle_integral[0] += Error_Roll_Angle * cntrl->work->dt;
			cntrl->Angle_integral[1] += Error_Pitch_Angle * cntrl->work->dt;
			cntrl->Angle_integral[2] += Error_Yaw_Rate * cntrl->work->dt;

			*c_delf = (Throttle) - cntrl->Throttle_Trim;
			*c_delm0 = cntrl->KP[0] * Error_Roll_Angle  - cntrl->KD[0] * Derivative_Angle_Roll    + cntrl->KI[0] * cntrl->Angle_integral[0];
			*c_delm1 = cntrl->KP[1] * Error_Pitch_Angle - cntrl->KD[1] * Derivative_Angle_Pitch   + cntrl->KI[1] * cntrl->Angle_integral[1];
			*c_delm2 = -(cntrl->KP[2] * Error_Yaw_Rate + cntrl->KI[2] * cntrl->Angle_integral[2]);

			// Serial.println("Error_Yaw_Rate");
			// Serial.println(Error_Yaw_Rate);
			// Serial.println("nav_w_z");
			// Serial.println(nav_w_z);
			// Serial.println("des_yaw_rate");
			// Serial.println(des_yaw_rate);
			// Serial.println("c_delm2");
			// Serial.println((cntrl->KP[2] * Error_Yaw_Rate + cntrl->KI[2] * cntrl->Angle_integral[2]));

			// Debugging output
			// Serial.println("----- Control Debug Output -----");
			// Serial.print("Desired Angles (rad): Roll = "); Serial.print(des_roll_angle*180/PI, 6);
			// Serial.print(", Pitch = "); Serial.println(des_pitch_angle*180/PI, 6);

			// Serial.print("Error Angles (rad): Roll = "); Serial.print(Error_Roll_Angle, 6);
			// Serial.print(", Pitch = "); Serial.println(Error_Pitch_Angle, 6);

			// Serial.print("Derivatives (rad/s): Roll = "); Serial.print(Derivative_Angle_Roll, 6);
			// Serial.print(", Pitch = "); Serial.println(Derivative_Angle_Pitch, 6);

			// Serial.print("Integral Terms: Roll = "); Serial.print(cntrl->Angle_integral[0], 6);
			// Serial.print(", Pitch = "); Serial.println(cntrl->Angle_integral[1], 6);

			// Serial.println("--------------------------------");
			// float angle_error_Phi = 0 - nav_phi;
			// float angle_error_Theta = 0 - nav_theta;
			// float angle_error_Psi = 0; //NOOOO PSI ERROR 
			
			

			// float des_roll_rate = Roll; //These values are already between -1 and 1, and 1 rad/s max angular vel is good
			// float des_pitch_rate = Pitch;
			// float des_yaw_rate = Yaw;

			// //Rate controller
			// float Roll_Rate_Error = (des_roll_rate - nav_w_x);
			// float Pitch_Rate_Error = (des_pitch_rate - nav_w_y);
			// float Yaw_Rate_Error = (des_yaw_rate - nav_w_z);


			// float Derivative_roll = (Roll_Rate_Error - cntrl->Prev_Angular_Vel_error[0]) / dt;
			// float Derivative_pitch = (Pitch_Rate_Error - cntrl->Prev_Angular_Vel_error[1]) / dt;
			// float Derivative_yaw = (Yaw_Rate_Error- cntrl->Prev_Angular_Vel_error[2]) / dt;

			// cntrl->Rate_integral[0] += Roll_Rate_Error * cntrl->work->dt;
			// cntrl->Rate_integral[1] += Pitch_Rate_Error * cntrl->work->dt;
			// cntrl->Rate_integral[2] += Yaw_Rate_Error * cntrl->work->dt;

			// *c_delf = (Throttle) - cntrl->Throttle_Trim;
			// *c_delm0 = cntrl->KP[0] * Roll_Rate_Error  - cntrl->KD[0] * Derivative_roll    + cntrl->KI[0] * cntrl->Rate_integral[0];
			// *c_delm1 = cntrl->KP[1] * Pitch_Rate_Error - cntrl->KD[1] * Derivative_pitch   + cntrl->KI[1] * cntrl->Rate_integral[1];
			// *c_delm2 = cntrl->KP[2] * Yaw_Rate_Error   - cntrl->KD[2] * Derivative_yaw     + cntrl->KI[2] * cntrl->Rate_integral[2];

			
			// cntrl->Prev_Angular_Vel_error[0] = Roll_Rate_Error;
			// cntrl->Prev_Angular_Vel_error[1] = Pitch_Rate_Error;
			// cntrl->Prev_Angular_Vel_error[2] = Yaw_Rate_Error;


			
		}else {

			Serial.print("Auto Controler");

		}
	}

}