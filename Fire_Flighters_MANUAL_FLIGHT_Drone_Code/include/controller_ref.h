/* Created by ESim dbp version $Revision: 2.1$, Mon Mar 03 23:50:28 2025 */

#define REF_FILE 1
#ifndef C__Users_myaze_Drone_Code_Code_pace_pace_controller_ref_PROTECT
#define C__Users_myaze_Drone_Code_Code_pace_pace_controller_ref_PROTECT 1

#if defined(__cplusplus)
extern "C"
{
#endif


#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(push)
#pragma pack(1)
#endif
struct controlWork_ref {
  double time; /* current time */
  unsigned long iLastUpdate; /* index of last update */
  double dtDes; /* time step desired per update */
  double dtMax; /* max time step to ever use (protect integration scheme) */
  double dtFull; /* actual time step */
  double dt; /* used in controller time step */
}
#if defined(__GNUC__)  && !defined(__FreeBSD__) && !defined(__arm__)
__attribute__((packed))
#endif
;
#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(pop)
#endif


extern struct dir_ref controlWork_dir;
extern struct controlWork_ref controlWork;




#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(push)
#pragma pack(1)
#endif
struct onboardControl_ref {
  unsigned char hitl; /*  */
  float posError[3]; /* Position error in inertial frame  */
  float posDes[3]; /* desired position */
  float psiCmd; /* desired heading (rad) */
  float Max_Vel_XY; /* Max vel In X,Y (ft/s) */
  float Max_Vel_Z; /* Max vel in Z (ft/s) */
  float Prev_Angular_Vel_error[3]; /* previous angular vel roll and pitch used for deriv calc. */
  float Angle_integral[3]; /* integral value for the rate controller. */
  float Throttle_Trim; /* Throtle Trim */
  float Stick_Scale[3]; /* Scaling stick inputs */
  float KP[3]; /* angle feedback OLD .3,.3,.3 */
  float KD[3]; /* (rad) rate feedback OLD .55,.55,.3 */
  float KI[3]; /* Integral gain */
  float del_stuff[4]; /* del_Stuff */
  float Angle_Integral_Manual_Control[3]; /* manual control integral value */
  float KP_V[3]; /* P Gain Des Velocity  */
  float KD_V[3]; /* D Gain Des Velocity */
  float KI_V[3]; /* I Gain Des Velocity */
  float KP_A[3]; /* P Gain Des Angle */
  float KD_A[3]; /* D Gain Des Angle */
  float KI_A[3]; /* I Gain Des Angle */
  float KP_T[3]; /* P Gain Des Throttle */
  float KD_T[3]; /* D Gain Des Throttle */
  float KI_T[3]; /* I Gain Des Throttle */
  float integralPos[3]; /* Integral position error */
  float integralAngle[3]; /* Integral angle error */
  float integralVel[3]; /* Integral veloity error */
  float Vel_Des[3]; /* Desired vel of the drone */
  float phiCmd; /* Commanded roll angle (rad) */
  float thetaCmd; /* Commanded pitch angle (rad) */
  float posError_Hdg[3]; /* Position error in body frame */
  float vel_Hdg[3]; /* Velocity in body frame */
  float vel_Error_Hdg[3]; /* Velocity error in body frame */
  float accel_Hdg[3]; /* Acceleration in body frame */
  struct controlWork_ref *work; /* work area (timing, flags, etc.) */
}
#if defined(__GNUC__)  && !defined(__FreeBSD__) && !defined(__arm__)
__attribute__((packed))
#endif
;
#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(pop)
#endif


extern struct dir_ref onboardControl_dir;
extern struct onboardControl_ref onboardControl;

extern struct controlWork_ref controlWork;



extern struct dir_ref controlWork_dir;

#if defined(__cplusplus)
}
#endif

#endif
