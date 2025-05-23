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
  float Max_Vel_XY; /* Max vel In X,Y (ft/s) */
  float Max_Vel_Z; /* Max vel in Z (ft/s) */
  float posDes[3]; /* desired position */
  float psiCmd; /* desired heading (rad) */
  float KP_Pos[3]; /* Proportional Pos gain 3rd is throttle */
  float KD_Pos[3]; /* Derivative Pos gain 3rd is throttle */
  float KI_Pos[3]; /* Integral Pos gain 3rd is throttle */
  float KP_Vel[3]; /* Proportional Vel gain 3rd is throttle */
  float KD_Vel[3]; /* Derivative Vel gain 3rd is throttle */
  float KI_Vel[3]; /* Integral Vel gain 3rd is throttle */
  float KP_Accel[3]; /* Proportional Acel gain 3rd is throttle */
  float KD_Accel[3]; /* Derivative Acel gain 3rd is throttle */
  float KI_Accel[3]; /* Integral Acel gain 3rd is throttle */
  float KP_Angle[3]; /* Proportional Angle gain 3rd is yaw */
  float KD_Angle[3]; /* Derivative Angle gain 3rd is yaw */
  float KI_Angle[3]; /* Integral Angle gain 3rd is yaw */
  float KP_Rate[3]; /* Proportional Rate gain 3rd is yaw */
  float KD_Rate[3]; /* Derivative Rate gain 3rd is yaw */
  float KI_Rate[3]; /* Integral Rate gain 3rd is yaw */
  float phiCmd; /* Commanded roll angle (rad) */
  float thetaCmd; /* Commanded pitch angle (rad) */
  float posError[3]; /* Position error in global frame */
  float posError_Hdg[3]; /* Position error in body frame */
  float Pos_Hdg[3]; /* Position error in body frame */
  float vel_Hdg[3]; /* Velocity in body frame */
  float accel_Hdg[3]; /* Acceleration in body frame */
  float Prev_Rate_error[3]; /* previous angular vel roll and pitch used for deriv calc. */
  float Rate_integral[3]; /* integral value for the rate controller. */
  float Prev_Angle_error[3]; /* previous angle for derivative calc */
  float Angle_integral[3]; /* integral value for the angle controller. */
  float Prev_Accel_error[3]; /* previous Acel for derivative calc */
  float Accel_integral[3]; /* integral value for the Acel controller. */
  float Prev_Vel_error[3]; /* previous Vel for derivative calc */
  float Vel_integral[3]; /* integral value for the Vel controller. */
  float Prev_Pos_error[3]; /* previous Pos for derivative calc */
  float Pos_integral[3]; /* integral value for the Pos controller. */
  int Waypoint_Number; /* Waypoint Number */
  int Number_of_Waypoints; /* number of waypoints */
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
