#ifndef rmax_controller_h
#define rmax_controller_h

#if defined(__cplusplus)
extern "C"
{
#endif
	void updateControl(const float dt, const float phiCmd,
		const float posDes_x, const float posDes_y, const float posDes_z,
		const float nav_p_x, const float nav_p_y,   const float nav_p_z,
		const float nav_v_x, const float nav_v_y,   const float nav_v_z,
		const float nav_w_x, const float nav_w_y,   const float nav_w_z,
		const float nav_phi, const float nav_theta, const float nav_psi,
		const float nav_a_x, const float nav_a_y, const float nav_a_z,
		float* c_delf, float* c_delm0, float* c_delm1, float* c_delm2,float Throttle,float Roll,float Pitch,float Yaw,bool Manual, bool Killed);

#if defined(__cplusplus)
}
#endif



#endif