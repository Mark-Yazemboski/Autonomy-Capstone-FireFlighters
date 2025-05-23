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
  3 , /* float Max_Vel_XY */
  3 , /* float Max_Vel_Z */
  {0,0,-10} , /* float posDes[3] */
  0 , /* float psiCmd */
  {.3,.3,.8} , /* float KP_Pos[3] */
  {.1,.1,.2} , /* float KD_Pos[3] */
  {0.001,0.001,0}, /* float KI_Pos[3] */
  {3,3,1.5} , /* float KP_Vel[3] */
  {0.7,0.7,0.3} , /* float KD_Vel[3] */
  {0.5,0.5,0.001}, /* float KI_Vel[3] */
  {0.6,0.6,0.01} , /* float KP_Accel[3] */
  {0,0,0} , /* float KD_Accel[3] */
  {0.01,0.01,0.1}, /* float KI_Accel[3] */
  {3.1,3.2,.3} , /* float KP_Angle[3] */
  {.09,.09,0.2} , /* float KD_Angle[3] */
  {0.001,0.001,0.001}, /* float KI_Angle[3] */
  {.012,.012,.1} , /* float KP_Rate[3] */
  {0.003,0.003,0.01} , /* float KD_Rate[3] */
  {0.001,0.001,0.01}, /* float KI_Rate[3] */
  0 , /* float phiCmd */
  0 , /* float thetaCmd */
  {0,0,0} , /* float posError[3] */
  {0,0,0} , /* float posError_Hdg[3] */
  {0,0,0} , /* float Pos_Hdg[3] */
  {0,0,0} , /* float vel_Hdg[3] */
  {0,0,0} , /* float accel_Hdg[3] */
  {0}, /* float Prev_Rate_error[3] */
  {0}, /* float Rate_integral[3] */
  {0}, /* float Prev_Angle_error[3] */
  {0}, /* float Angle_integral[3] */
  {0}, /* float Prev_Accel_error[3] */
  {0,0,-8.1}, /* float Accel_integral[3] */
  {0}, /* float Prev_Vel_error[3] */
  {0}, /* float Vel_integral[3] */
  {0}, /* float Prev_Pos_error[3] */
  {0}, /* float Pos_integral[3] */
  0 , /*waypoint number*/
  0 , /*number of waypoints*/
  &controlWork, /* dir work */
};

