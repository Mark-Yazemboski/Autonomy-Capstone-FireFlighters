/* Created by ESim dbp version $Revision: 2.1$, Wed Mar 19 22:20:50 2025 */

#include "../include/myekf_ref.h"
#define DEF_FILE 1


struct My_Kalman_Filter2 MyEKF_Function_Variables = {
  {1,0,0,0,0,0,0}, /* double Orientation_State[7] */
  {0,0,0,0,0,0,0,0,0}, /* double Position_State[9] */
  {0}, /* double Orientation_Covariance[7*7] */
  {0}, /* double Position_Covariance[9*9] */
  {0}, /* double State_Vector_Derivative_Orientation[7] */
  {0}, /* double State_Vector_Derivative_Position[9] */
  {0.001,0.001,0.001,0.001}, /* double Orientation_Motion_Capture_Sigma[4] */
  {0,0,0}, /* double Euler_Angles[3] */
  {0,0,0}, /* double Angular_Rates_Avg[3] */
  {0,0,0}, /* double Angular_Rates_Current_Frame[3] */
  0, /* int EKF_Counter */
  0, /* int IMU_Counter */
  0, /* int GPS_Counter */
};



struct My_navout_ref My_navout = {
  {0,0,0} , /* double p_b_e_L[3] */
  {0,0,0} , /* double v_b_e_L[3] */
  {0,0,0} , /* double a_b_e_L[3] */
  {0,0,0} , /* double v_b_e_B[3] */
  {0,0,0} , /* double a_b_e_B[3] */
  {0,0,0} , /* double w_b_e_B[3] */
  {1,0,0,0} , /* double q[4] */
  0 , /* double phi */
  0 , /* double theta */
  0 , /* double psi */
  0, /* double Dt */
  0, /* bool fire found*/
  {0,0}, /*float Fire Location*/
  0, /*bool Payload Dropped*/
};



struct My_Kalman_ref MyEKF = {
  &MyEKF_Function_Variables, /* dir MyEKF_Function_Variables */
  &My_navout, /* dir out */
};

