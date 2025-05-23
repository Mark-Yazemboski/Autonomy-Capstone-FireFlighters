#include "controller_ref.h"
#include "controller.h"

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

		cntrl->Rate_integral[0] = 0;
		cntrl->Rate_integral[1] = 0;
		cntrl->Rate_integral[2] = 0;

		*c_delf = -1;
		*c_delm0 = 0;
		*c_delm1 = 0;
		*c_delm2 = 0;
	}else{





		if (Manual) {

			//**************************ANGLE CONTROLLER************************************************
			float des_roll_Angle = Roll*20*3.1415926535 / 180;
			float des_pitch_Angle = Pitch * 20 * 3.1415926535 / 180;
			//float des_yaw_Angle = data->up0->rudderPedal * 20 * 3.1415926535 / 180;


			float Roll_Angle_Error = (des_roll_Angle - nav_phi);
			float Pitch_Angle_Error = (des_pitch_Angle - nav_theta);
			//float Yaw_Angle_Error = (des_yaw_Angle - nav_psi);


			float Derivative_roll_Angle = nav_w_x;
			float Derivative_pitch_Angle = nav_w_y;
			//float Derivative_yaw_Angle = nav_w_z;

			cntrl->Angle_integral[0] += Roll_Angle_Error * cntrl->work->dt;
			cntrl->Angle_integral[1] += Pitch_Angle_Error * cntrl->work->dt;
			//cntrl->Angle_integral[2] += Yaw_Angle_Error * cntrl->work->dt;

			float des_roll_rate = cntrl->KP_Angle[0] * Roll_Angle_Error - cntrl->KD_Angle[0] * Derivative_roll_Angle + cntrl->KI_Angle[0] * cntrl->Angle_integral[0];
			float des_pitch_rate = cntrl->KP_Angle[1] * Pitch_Angle_Error - cntrl->KD_Angle[1] * Derivative_pitch_Angle + cntrl->KI_Angle[1] * cntrl->Angle_integral[1];
			//float des_yaw_rate = cntrl->KP_Angle[2] * Yaw_Angle_Error - cntrl->KD_Angle[2] * Derivative_yaw_Angle + cntrl->KI_Angle[2] * cntrl->Angle_integral[2];


			//**************************RATE CONTROLLER************************************************
			float des_yaw_rate = Yaw*.2;//These values are already between -1 and 1, and 1 rad/s max angular vel is good
			/*float des_roll_rate = data->up0->rollStick*.2;
			float des_pitch_rate = data->up0->pitchStick*.2;*/


			float Roll_Rate_Error = (des_roll_rate - nav_w_x);
			float Pitch_Rate_Error = (des_pitch_rate - nav_w_y);
			float Yaw_Rate_Error = (des_yaw_rate - nav_w_z);


			float Derivative_roll_Rate = (cntrl->Prev_Rate_error[0] - Roll_Rate_Error) / dt;
			float Derivative_pitch_Rate = (cntrl->Prev_Rate_error[1] - Pitch_Rate_Error) / dt;
			float Derivative_yaw_Rate = (cntrl->Prev_Rate_error[2] - Yaw_Rate_Error) / dt;

			cntrl->Rate_integral[0] += Roll_Rate_Error * cntrl->work->dt;
			cntrl->Rate_integral[1] += Pitch_Rate_Error * cntrl->work->dt;
			cntrl->Rate_integral[2] += Yaw_Rate_Error * cntrl->work->dt;

			*c_delm0 = cntrl->KP_Rate[0] * Roll_Rate_Error  - cntrl->KD_Rate[0] * Derivative_roll_Rate + cntrl->KI_Rate[0] * cntrl->Rate_integral[0];
			*c_delm1 = cntrl->KP_Rate[1] * Pitch_Rate_Error - cntrl->KD_Rate[1] * Derivative_pitch_Rate + cntrl->KI_Rate[1] * cntrl->Rate_integral[1];
			*c_delm2 = cntrl->KP_Rate[2] * Yaw_Rate_Error   - cntrl->KD_Rate[2] * Derivative_yaw_Rate + cntrl->KI_Rate[2] * cntrl->Rate_integral[2];

			cntrl->Prev_Rate_error[0] = Roll_Rate_Error;
			cntrl->Prev_Rate_error[1] = Pitch_Rate_Error;
			cntrl->Prev_Rate_error[2] = Yaw_Rate_Error;


			
		}else {
			
			

		}
	}

}