/* Created by ESim dbp version $Revision: 2.1$, Wed Mar 19 22:20:50 2025 */

#define REF_FILE 1
#ifndef C__Users_myaze_Drone_Code_Code_pace_pace_myekf_ref_PROTECT
#define C__Users_myaze_Drone_Code_Code_pace_pace_myekf_ref_PROTECT 1

#if defined(__cplusplus)
extern "C"
{
#endif


#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(push)
#pragma pack(1)
#endif
struct My_Kalman_Filter2 {
  float Orientation_State[7]; /* Orientation State Vector */
  float Position_State[9]; /* Position State Vector */
  float Orientation_Covariance[7*7]; /* Orientation covariance matrix */
  float Position_Covariance[9*9]; /* Position covariance matrix */
  float State_Vector_Derivative_Orientation[7]; /* Derivative Orientation */
  float State_Vector_Derivative_Position[9]; /* Derivative Position */
  float Orientation_Motion_Capture_Sigma[4]; /* Error orentation for motion capture system */
  float Euler_Angles[3]; /* Euler angles */
  float Angular_Rates_Avg[3]; /* Angular Rates(average of 2 frames) */
  float Angular_Rates_Current_Frame[3]; /* Angular Rates Current Frame */
  int EKF_Counter; /* Counts how many times the EKF function is called */
  int IMU_Counter; /* Saves the last IMU Count */
  int GPS_Counter; /* Saves the last GPS Count */
}
#if defined(__GNUC__)  && !defined(__FreeBSD__) && !defined(__arm__)
__attribute__((packed))
#endif
;
#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(pop)
#endif


extern struct dir_ref MyEKF_Function_Variables_dir;
extern struct My_Kalman_Filter2 MyEKF_Function_Variables;




#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(push)
#pragma pack(1)
#endif
struct My_navout_ref {
  double p_b_e_L[3]; /* position of body wrt Earth expressed in local frame */
  double v_b_e_L[3]; /* velocity of body wrt Earth expressed in local frame */
  double a_b_e_L[3]; /* accel of body wrt Earth expressed in local frame */
  double v_b_e_B[3]; /* velocity of body wrt Earth expressed in body */
  double a_b_e_B[3]; /* accel of body wrt Earth expressed in body frame */
  double w_b_e_B[3]; /* angular rate of body wrt Earth expressed in body (average of 2 frames) */
  double q[4]; /* attitude quaternion */
  double phi; /*  */
  double theta; /*  */
  double psi; /*  */
  double time; /* time */
}
#if defined(__GNUC__)  && !defined(__FreeBSD__) && !defined(__arm__)
__attribute__((packed))
#endif
;
#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(pop)
#endif


extern struct dir_ref My_navout_dir;
extern struct My_navout_ref My_navout;



#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(push)
#pragma pack(1)
#endif
struct My_Kalman_ref {
  struct My_Kalman_Filter2 *MyEKF_Function_Variables; /* State */
  struct My_navout_ref *out; /* output structure (to control) */
}
#if defined(__GNUC__)  && !defined(__FreeBSD__) && !defined(__arm__)
__attribute__((packed))
#endif
;
#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(pop)
#endif


extern struct dir_ref MyEKF_dir;
extern struct My_Kalman_ref MyEKF;

extern struct My_Kalman_Filter2 MyEKF_Function_Variables;
extern struct My_navout_ref My_navout;



extern struct dir_ref MyEKF_Function_Variables_dir;
extern struct dir_ref My_navout_dir;

#if defined(__cplusplus)
}
#endif

#endif
