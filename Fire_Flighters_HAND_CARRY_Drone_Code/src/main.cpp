/***
 * Copyright 2022, The Pennsylvania State University, All Rights Reserved.
 * Unauthorized use and/or redistribution is disallowed.
 * This library is distributed without any warranty; without even
 * the implied warranty of fitness for a particular purpose.
 *
 * Pennsylvania State University Unmanned Aerial System Research Laboratory (PURL)
 * Department of Aerospace Engineering
 * 229 Hammond
 * The Pennsylvania State University
 * University Park, PA 16802
 * http://purl.psu.edu
 *
 * Contact Information:
 * Dr. Thanakorn Khamvilai (thanakorn.khamvilai@ttu.edu)
 * Dr. Vitor T. Valente (vitor.valente@psu.edu)
 *
 * EndCopyright
 ***/

#include "../include/main.h"
#include "../include/Pozyx_definitions.h"
//#include "avr8-stub.h"


 #define HAVE_DATALINK    1
 #define HAVE_MOCAP       true
 #define HAVE_IMU         1
 #define HAVE_RC_RECEIVER 0
 #define HAVE_MOTORS      0
 #define HAVE_THERMAL     1
 #define HAVE_CONTROLLER  1
 #define HAVE_KALMAN      1
 #define PRINT_DT         0
 #define PRINT_KILL_AUTO_PAYLOAD 0

#define MIN_PWM 1000 //I ADDED THIS
#define MAX_PWM 1980//I ADDED THIS
#define LIMIT(x,xl,xu) ((x)>=(xu)?(xu):((x)<(xl)?(xl):(x)))//I ADDED THIS

/* Values in ms -> for Freq = 1000/value */
const unsigned long intervalIMU = 10;
const unsigned long intervalRC = 10;
const unsigned long intervalMotors = 10;
const unsigned long intervalDatalink_Recieve = 10;
const unsigned long intervalThermal = 1200;
const unsigned long intervalPrint = 100;
const unsigned long intervalCtrl = 10;
const unsigned long intervalNav = 10;
const unsigned long intervalData_Link_Send = 10;

unsigned long previousMillisIMU = 0;
unsigned long previousMillisRC = 0;
unsigned long previousMillisMotors = 0;
unsigned long previousMillisDatalink_Recieve = 0;
unsigned long previousMillisThermal = 0;
unsigned long previousMillisCntrl = 0;
unsigned long previousMillisPrint = 0;
unsigned long previousMillisNav = 0;
unsigned long previousData_Link_Send = 0;
bool Fire_Found = false;
float Fire_Position[3] = { 0,0,0 };
float Opened_Time = 0;
bool Opened = false;
float Open_Time = 0;

// float Gyro_Corrected_wx;
// float Gyro_Corrected_wy;
// float Gyro_Corrected_wz;

// float Gyro_Corrected_euler_phi;
// float Gyro_Corrected_euler_theta;
// float Gyro_Corrected_euler_psi;


void update_Previous_Time_Values()
{
  previousMillisIMU = millis();
  previousMillisRC = millis();
  previousMillisMotors = millis();
  previousMillisDatalink_Recieve = millis();
  previousMillisThermal = millis();
  previousMillisCntrl = millis();
  previousMillisPrint = millis();
  previousMillisNav = millis();
  previousData_Link_Send = millis();
}




void updateMixer( const char button0, char* LaunchState,
  float c_delf, float c_delm0, float c_delm1, float c_delm2,
  float* PWM)//I ADDED THIS
{
  // mixer
  if ( 1 == button0 ) // arm
  {
  *LaunchState = 1;

  // only throttle and roll
  float thr_pwm = ( c_delf - ( -1 ) ) * ( 2000 - 1000 ) / ( 1 - ( -1 ) ) + 1000; // from [-1,1] to [1000,2000]
  float roll_pwm = ( c_delm0 - ( -1 ) ) * ( 500 - ( -500 ) ) / ( 1 - ( -1 ) ) + ( -500 ); // from [-1,1] to [-500,500]
  float pitch_pwm = ( c_delm1 - ( -1 ) ) * ( 500 - ( -500 ) ) / ( 1 - ( -1 ) ) + ( -500 ); // from [-1,1] to [-500,500]
  float yaw_pwm = ( c_delm2 - ( -1 ) ) * ( 500 - ( -500 ) ) / ( 1 - ( -1 ) ) + ( -500 ); // from [-1,1] to [-500,500]

  PWM[0] = ( float ) LIMIT( thr_pwm - roll_pwm - pitch_pwm - yaw_pwm, MIN_PWM, MAX_PWM ); // front-right CCW
  PWM[1] = ( float ) LIMIT( thr_pwm - roll_pwm + pitch_pwm + yaw_pwm, MIN_PWM, MAX_PWM ); // back-right  CW
  PWM[2] = ( float ) LIMIT( thr_pwm + roll_pwm + pitch_pwm - yaw_pwm, MIN_PWM, MAX_PWM ); // back-left   CCW
  PWM[3] = ( float ) LIMIT( thr_pwm + roll_pwm - pitch_pwm + yaw_pwm, MIN_PWM, MAX_PWM ); // front-left  CW
  }
  else // disarm
  {

  #if PRINT_KILL_AUTO_PAYLOAD  
    Serial.println("KILLED");
  #endif
  *LaunchState = 0;
  PWM[0] = MIN_PWM;
  PWM[1] = MIN_PWM;
  PWM[2] = MIN_PWM;
  PWM[3] = MIN_PWM;
  }
}

float mapStickInput(float input) {
  return (((input - 900) * 2.0 / (2000 - 900)) - 1.0);
}

void setup(){
    //Initialize serial and wait for port to open:
    Serial.begin(9600);
    //debug_init();
    while (!Serial) {
      ;
    }
    

    
    
    #if HAVE_THERMAL
    Serial.println("Initilizing thermal");
    // initialize thermal camera
    thermal.init();
    Serial.println("Finished Initilizing thermal");
    #endif
    
    
    
    #if HAVE_DATALINK
    Serial.println("Initilizing datalink");
    // initialize wifi
    datalink.init();
    Serial.println("Finished Initilizing datalink");
    #endif
    

    
    #if HAVE_IMU
    Serial.println("Initilizing sens");
    // initialize sensors
    sens.init();
    Serial.println("Finished Initilizing sens");
    #endif
    
    
    
    #if HAVE_RC_RECEIVER
    Serial.println("Initilizing rc");
    // initialize rc
    rc.init();
    Serial.println("Finished Initilizing rc");
    #endif
    


    

    
    
    


    Serial.println("Initilizing motors");
    // initialize motors/servos
    for (int i=0; i<NUM_MOTORS; i++){
        pwm[i] = MIN_PWM_OUT;
    }
    
    motors.init();
    //motors.calibrate();
    Serial.println("Finished Initilizing motors");
   
    update_Previous_Time_Values();
    Serial.println("Starting Drone!");
 }

void loop(){
    struct onboardControl_ref* cntrl = &onboardControl;
    struct My_Kalman_ref* EKF = &MyEKF;
    struct My_navout_ref* My_Out = EKF->out;
    
  
    float c_delf = 0, c_delm[3] = { 0,0,0 };
    /* The order should be:
        1-) Read all sensors and data
        2-) Perform calculations
        3-) Motor actuation
        4-) Update data on GCS
    */
    unsigned long currentMillis = millis();


    #if HAVE_IMU
    currentMillis = millis();
    sens.update();
    #if PRINT_DT
    Serial.println("Sensors");
    Serial.println(millis()- currentMillis);
    #endif
    #endif
    
    #if HAVE_RC_RECEIVER
    currentMillis = millis();
    if (currentMillis - previousMillisRC >= intervalRC) {
      previousMillisRC = currentMillis;
        rc.update();
    }
    #if PRINT_DT
    Serial.println("RC");
    Serial.println(millis()- currentMillis);
    #endif
    #endif

    


    #if HAVE_DATALINK
    currentMillis = millis();
    if (currentMillis - previousMillisDatalink_Recieve >= intervalDatalink_Recieve) {
      previousMillisDatalink_Recieve = currentMillis;
      datalink.recv_update(&sens, HAVE_MOCAP);
    }

    // Serial.print(sens.data_MoCap.Euler_Angles[0]);
    // Serial.print(" , ");
    // Serial.print(sens.data_POZYX.euler[0]);
    // Serial.println();
    // Serial.println("Begin");
    // Serial.print(sens.data_MoCap.Pos[0]);
    // Serial.print("\t ");
    // Serial.print(sens.data_MoCap.Pos[1]);
    // Serial.print("\t ");
    // Serial.print(sens.data_MoCap.Pos[2]);
    // Serial.print("\t ");
    // Serial.print(sens.data_MoCap.Quat[0]);
    // Serial.print("\t ");
    // Serial.print(sens.data_MoCap.Quat[1]);
    // Serial.print("\t ");
    // Serial.print(sens.data_MoCap.Quat[2]);
    // Serial.print("\t ");
    // Serial.println(sens.data_MoCap.Quat[3]);
    // Serial.println("End");
    #if PRINT_DT
    Serial.println("Datalink");
    Serial.println(millis()- currentMillis);
    #endif
    #endif

    #if HAVE_KALMAN
    currentMillis = millis();
    /* Navigation stuff goes here */
    if (currentMillis - previousMillisNav >= intervalNav) {
      double dt = (currentMillis - previousMillisNav)/1000.0;
      previousMillisNav = currentMillis;
      updateNavigation(dt,sens,Fire_Found,(float)My_Out->Fire_Location[0],(float)My_Out->Fire_Location[1],My_Out->Payload_Dropped);
      //Serial.println(dt);
    }
    #if PRINT_DT
    Serial.println("Navigation");
    Serial.println(millis()- currentMillis);
    #endif
    #endif

    

    /* Controller stuff goes here */
    
    #if HAVE_CONTROLLER
    currentMillis = millis();
    if (currentMillis - previousMillisCntrl >= intervalCtrl) {
      //I ADDED ALL OF THIS
      float dt = (currentMillis - previousMillisCntrl)/1000;
      previousMillisCntrl = currentMillis;

      //This will recieve the 900-2000 pwm signals from the reciever
      float Throttle_Stick = rc.rc_in.THR;
      float Roll_Stick = rc.rc_in.ROLL;
      float Pitch_Stick = rc.rc_in.PITCH;
      float Yaw_Stick = rc.rc_in.YAW;
      float KILL_SWITCH_Value = rc.rc_in.AUX;
      float Auto_Manual_Payload_SWITCH_Value = rc.rc_in.AUX2;

      //This will map those values between -1 and 1
      float Throttle_Stick_Mapped = mapStickInput(Throttle_Stick);
      float Roll_Stick_Mapped     = mapStickInput(Roll_Stick);
      float Pitch_Stick_Mapped    = mapStickInput(Pitch_Stick);
      float Yaw_Stick_Mapped      = mapStickInput(Yaw_Stick);



      
      
      bool Arm_Switch_State; //Arm is True; kill is false
      //Added detection to see if the reciever lost power, or the kill wire came loose. This will auto kill the drone.
      if (rc.rc_in.Good_Reciever && rc.rc_in.Good_Kill_Wire){
        if (KILL_SWITCH_Value > 1500){
          Arm_Switch_State = false;
        }else{
          Arm_Switch_State = true;
        }
      }else{
        Arm_Switch_State = false;
      }

      int Auto_Manual_Payload_State;//2 is manual Payload drop; 1 is manual; 0 is auto
      bool Manual;
      if (Auto_Manual_Payload_SWITCH_Value > 1700){
        Auto_Manual_Payload_State = 2;
        Manual = true;
      }else if (Auto_Manual_Payload_SWITCH_Value > 1200){
        Auto_Manual_Payload_State = 1;
        Manual = true;
      }else{
        Auto_Manual_Payload_State = 0;
        Manual = false;
      }
      
      if (Auto_Manual_Payload_State == 1 || Auto_Manual_Payload_State == 2){
          
        //   updateControl(dt, (float)cntrl->phiCmd,
				// (float)cntrl->posDes[0], (float)cntrl->posDes[1], (float)cntrl->posDes[2],
				// (float)My_Out->p_b_e_L[0], (float)My_Out->p_b_e_L[1], (float)My_Out->p_b_e_L[2],
				// (float)My_Out->v_b_e_L[0], (float)My_Out->v_b_e_L[1], (float)My_Out->v_b_e_L[2],
				// (float)My_Out->w_b_e_B[0], (float)My_Out->w_b_e_B[1], (float)My_Out->w_b_e_B[2],
				// (float)My_Out->phi, (float)My_Out->theta, (float)My_Out->psi,
				// (float)My_Out->a_b_e_L[0], (float)My_Out->a_b_e_L[1], (float)My_Out->a_b_e_L[2],
				// &c_delf, &c_delm[0], &c_delm[1], &c_delm[2],
        // Throttle_Stick_Mapped,Roll_Stick_Mapped,Pitch_Stick_Mapped,Yaw_Stick_Mapped,Manual,!Arm_Switch_State);

        updateControl(dt, (float)cntrl->phiCmd,
				(float)cntrl->posDes[0], (float)cntrl->posDes[1], (float)cntrl->posDes[2],
				(float)My_Out->p_b_e_L[0], (float)My_Out->p_b_e_L[1], (float)My_Out->p_b_e_L[2],
				(float)My_Out->v_b_e_L[0], (float)My_Out->v_b_e_L[1], (float)My_Out->v_b_e_L[2],
				sens.data_POZYX.gyr_rad[0],sens.data_POZYX.gyr_rad[1],sens.data_POZYX.gyr_rad[2],
				sens.data_POZYX.euler_rad[0], sens.data_POZYX.euler_rad[1],sens.data_POZYX.euler_rad[2],
				(float)My_Out->a_b_e_L[0], (float)My_Out->a_b_e_L[1], (float)My_Out->a_b_e_L[2],
				&c_delf, &c_delm[0], &c_delm[1], &c_delm[2],
        Throttle_Stick_Mapped,Roll_Stick_Mapped,Pitch_Stick_Mapped,Yaw_Stick_Mapped,Manual,!Arm_Switch_State);



        if (Auto_Manual_Payload_State == 2){
          #if PRINT_KILL_AUTO_PAYLOAD  
          Serial.println("PAYLOAD DROP");
          #endif
          motors.Servo_Open();
        }else{
          motors.Servo_Closed();
        }


      }else{
        #if PRINT_KILL_AUTO_PAYLOAD  
        Serial.println("AUTO");
        #endif
        c_delf = -1;
        c_delm[0] = 0;
        c_delm[1] = 0;
        c_delm[2] = 0;
      }


      char LaunchState;
      updateMixer( Arm_Switch_State, &LaunchState,
        c_delf, c_delm[0], c_delm[1], c_delm[2],
        pwm);

        
    }
    #if PRINT_DT
    Serial.println("Controller");
    Serial.println(millis()- currentMillis);
    #endif
    #endif


    #if HAVE_THERMAL
    currentMillis = millis();
    if (currentMillis - previousMillisThermal >= intervalThermal) {
      previousMillisThermal = currentMillis;
        thermal.update();
        Fire_Found = thermal.Find_Fire(My_Out->p_b_e_L[2], 55, 35, Fire_Position);
        float Global_Fire_X_Error = cos(sens.data_POZYX.euler_rad[2]) * Fire_Position[0] - sin(sens.data_POZYX.euler_rad[2]) * Fire_Position[1];
        float Global_Fire_Y_Error = sin(sens.data_POZYX.euler_rad[2]) * Fire_Position[0] + cos(sens.data_POZYX.euler_rad[2]) * Fire_Position[1];
        Open_Time = millis() - Opened_Time;

        
        if (Fire_Found){
          My_Out->Fire_Detected = 1;
          My_Out->Fire_Location[0] = Global_Fire_X_Error + My_Out->p_b_e_L[0];
          My_Out->Fire_Location[1] = Global_Fire_Y_Error + My_Out->p_b_e_L[1];
          // cntrl->posDes[0] = My_Out->Fire_Location[0];
          // cntrl->posDes[1] = My_Out->Fire_Location[1];
          // cntrl->posDes[2] = 3;


          // Serial.print("Fire Found at body frame Delta: ");
          // Serial.print(Fire_Position[0]);
          // Serial.print(" , ");
          // Serial.print(Fire_Position[1]);
          // // Serial.print(" , ");
          // // Serial.print(sens.data_POZYX.euler_rad[2]);
          // Serial.println();

          // Serial.print("Gloabal Fire Position: ");
          // Serial.print(Global_Fire_X_Error);
          // Serial.print(" , ");
          // Serial.print(Global_Fire_Y_Error);
          // // Serial.print(" , ");
          // // Serial.print(My_Out->p_b_e_L[2]);
          // Serial.println();

          float Total_Global_Fire_Error = sqrt((Global_Fire_X_Error * Global_Fire_X_Error) + (Global_Fire_Y_Error * Global_Fire_Y_Error));
          float Acceptable_Fire_Error = .2;
          if (Total_Global_Fire_Error < Acceptable_Fire_Error || (Open_Time < 5000 && Opened == true)){
            //Serial.println("DROPPED");
            My_Out->Payload_Dropped = 1;
            motors.Servo_Open();
            if (Open_Time>5000){
              Opened_Time = millis();
            }
            
            Opened = true;
          }else{
            //Serial.println("CLOSED");
            motors.Servo_Closed();
            My_Out->Payload_Dropped = 0;
          }
        }else{
          My_Out->Fire_Detected = 0;
          //Serial.println("No Fire Found");
          if (Open_Time>3000){
            My_Out->Payload_Dropped = 0;
            //Serial.println("CLOSED");
            motors.Servo_Closed();
          }
          
          
        }
        

        


        #if PRINT_DT
        Serial.println("Thermal");
        Serial.println(millis()- currentMillis);
        #endif


    }
    #endif

    #if HAVE_DATALINK
    currentMillis = millis();
    if (currentMillis - previousData_Link_Send >= intervalData_Link_Send) {
      previousData_Link_Send = currentMillis;
      datalink.send_update(sens, c_delf, c_delm[0], c_delm[1], c_delm[2],rc.rc_in.THR,rc.rc_in.ROLL,rc.rc_in.PITCH,rc.rc_in.YAW,rc.rc_in.AUX2,rc.rc_in.AUX,pwm[0],pwm[1],pwm[2],pwm[3]);
    }

    #if PRINT_DT
    Serial.println("Data Link Send");
    Serial.println(millis()- currentMillis);
    #endif

    #endif


    #if HAVE_MOTORS
    currentMillis = millis();
    if (currentMillis - previousMillisMotors >= intervalMotors) {
      previousMillisMotors = currentMillis;
      motors.update(pwm);
    }
    #if PRINT_DT
    Serial.println("Motors");
    Serial.println(millis()- currentMillis);
    #endif
    #endif

    currentMillis = millis();
    if (currentMillis - previousMillisPrint >= intervalPrint) {
      previousMillisPrint = currentMillis;

      // Serial.print("KP: [");
      // Serial.print(cntrl->KP[0], 4); Serial.print(", ");
      // Serial.print(cntrl->KP[1], 4); Serial.print(", ");
      // Serial.print(cntrl->KP[2], 4); Serial.println("]");

      // Serial.print("KI: [");
      // Serial.print(cntrl->KI[0], 4); Serial.print(", ");
      // Serial.print(cntrl->KI[1], 4); Serial.print(", ");
      // Serial.print(cntrl->KI[2], 4); Serial.println("]");

      // Serial.print("KD: [");
      // Serial.print(cntrl->KD[0], 4); Serial.print(", ");
      // Serial.print(cntrl->KD[1], 4); Serial.print(", ");
      // Serial.print(cntrl->KD[2], 4); Serial.println("]");


      // Serial.println(rc.rc_in.Good_Reciever);
      // Serial.println(rc.rc_in.Good_Kill_Wire);
      //Serial.println(rc.rc_in.AUX);
      //Serial.println(rc.rc_in.Same_Count);
      //rc.print();
      //sens.print();
      // Serial.print("pwm: [");
      // Serial.print(pwm[0]);
      // Serial.print(", ");
      // Serial.print(pwm[1]);
      // Serial.print(", ");
      // Serial.print(pwm[2]);
      // Serial.print(", ");
      // Serial.print(pwm[3]);
      // Serial.println("]\n");

      // Serial.print("wx: ");
      // Serial.print(sens.data_POZYX.gyr_rad[0]);
      // Serial.print("wy: ");
      // Serial.print(sens.data_POZYX.gyr_rad[1]);
      // Serial.print("wz: ");
      // Serial.println(sens.data_POZYX.gyr_rad[2]);
      // Serial.print("UPDATE COUNTER:");
      // Serial.print(sens.data_POZYX.POZYX_Update_Counter);

      // Serial.print("c_del: [");
      // Serial.print(c_delf);
      // Serial.print(", ");
      // Serial.print(c_delm[0]);
      // Serial.print(", ");
      // Serial.print(c_delm[1]);
      // Serial.print(", ");
      // Serial.print(c_delm[2]);
      // Serial.println("]");

      // Serial.print("Stick_Values: [");
      // Serial.print(Throttle_Stick);
      // Serial.print(", ");
      // Serial.print(Roll_Stick);
      // Serial.print(", ");
      // Serial.print(Pitch_Stick);
      // Serial.print(", ");
      // Serial.print(Yaw_Stick);
      // Serial.println("]");

      // Serial.print("Stick_Values_MAPPED: [");
      // Serial.print(Throttle_Stick_Mapped);
      // Serial.print(", ");
      // Serial.print(Roll_Stick_Mapped);
      // Serial.print(", ");
      // Serial.print(Pitch_Stick_Mapped);
      // Serial.print(", ");
      // Serial.print(Yaw_Stick_Mapped);
      // Serial.println("]");

    }
 }