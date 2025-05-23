/* Created by ESim dbp version $Revision: 2.1$, Mon Mar 03 23:50:28 2025 */

#include "../include/controller_ref.h"
#define DEF_FILE 1


struct controlWork_ref controlWork = {
  0.0    , /* double time */
  0 , /* ulong iLastUpdate */
  0.02  , /* double dtDes */
  0.04  , /* double dtMax */
  0.02 , /* double dtFull */
  0.02     , /* double dt */
};



struct onboardControl_ref onboardControl = {
  0 , /* uchar hitl */
  {0,0,0} , /* float posError[3] */
  {0,0,-10} , /* float posDes[3] */
  0 , /* float psiCmd */
  3 , /* float Max_Vel_XY */
  3 , /* float Max_Vel_Z */
  {0}, /* float Prev_Angular_Vel_error[3] */
  {0}, /* float Rate_integral[3] */
  0.0, /* float Throttle_Trim */
  {.5,.5,.5}, /* float Stick_Scale[3] */



  {0.019,0.024,0.15} , /* float KP[3] 0.019,0.024,0.15*/                          // Proportional gains
  {0.016,0.016,0.0} , /* float KD[3] 0.016,0.016,0.0*/
  {0.019,0.024,0.01}, /* float KI[3] 0.019,0.024,0.01*/




  {0,0,0,0}, /* float del_stuff[4] */
  {0}, /* float Angle_Integral_Manual_Control[3] */
  {0.4,0.4,.5} , /* float KP_V[3] */
  {0.4,0.4,.4} , /* float KD_V[3] */
  {0.01,0.01,0.01} , /* float KI_V[3] */
  {0.3,0.3,1} , /* float KP_A[3] */
  {0.03,0.03,0.4} , /* float KD_A[3] */
  {0.01,0.01,0.01} , /* float KI_A[3] */
  {0.2,0.2,0.4} , /* float KP_T[3] */
  {.8,.8,1.3} , /* float KD_T[3] */
  {0.01,0.01,0.01} , /* float KI_T[3] */
  {0,0,0} , /* float integralPos[3] */
  {0,0,0} , /* float integralAngle[3] */
  {0,0,0} , /* float integralVel[3] */
  {0,0,0}, /* float Vel_Des[3] */
  0 , /* float phiCmd */
  0 , /* float thetaCmd */
  {0,0,0} , /* float posError_Hdg[3] */
  {0,0,0} , /* float vel_Hdg[3] */
  {0,0,0}, /* float vel_Error_Hdg[3] */
  {0,0,0} , /* float accel_Hdg[3] */
  &controlWork, /* dir work */
};

