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

#ifndef AERSP_DLINK_H
#define AERSP_DLINK_H

#include <stdint.h>
#include <stdlib.h>
#include "wifi.h"
#include "rc_pilot.h"
#include "sensors.h"

#define DATALINK_SYNC0 0xa3
#define DATALINK_SYNC1 0xb2
#define DATALINK_SYNC2 0xc1

#define DATALINK_MESSAGE0 0
#define DATALINK_MESSAGE1 1
#define DATALINK_MESSAGE_UP0 12
#define DATALINK_MESSAGE_PWM 56
#define DATALINK_MESSAGE_AUTOPILOTDELS 173
#define DATALINK_MESSAGE_TRUTH 221
#define DATALINK_MESSAGE_HITL_SIM2ONBOARD 222
#define DATALINK_MESSAGE_HITL_ONBOARD2SIM 223
#define DATALINK_MESSAGE_OPTITRACK 230
#define DATALINK_MESSAGE_DRONE_STATE 4
#define DATALINK_MESSAGE_SENSOR_DATA 5
#define DATALINK_MESSAGE_PID 250

#define DATALINK_INTERVALUP0 100
#define DATALINK_INTERVALPWM 100
#define DATALINK_INTERVALAUTOPILOTDELS 10
#define DATALINK_INTERVALTRUTH 100
#define DATALINK_INTERVALHITL_SIM2ONBOARD 100
#define DATALINK_INTERVALHITL_ONBOARD2SIM 100
#define DATALINK_INTERVALOPTITRACK 10
#define DATALINK_INTERVAL_DRONE_STATE 10
#define DATALINK_INTERVAL_SENSOR_DATA 10
#define DATALINK_INTERVAL_PID 10


struct obDatalink_ref
{
  struct datalinkWork_ref                   *work;          /* working area */
	struct datalinkHeader_ref                 *header;        /* raw message header   */
  struct datalinkMessage0_ref               *m0;            /* raw message sent     */
  struct datalinkMessage1_ref               *m1;            /* raw message sent     */
  struct datalinkMessageUp0_ref             *up0;           /* raw message sent     */
  struct datalinkMessagePWM_ref             *pwm;           /* raw message sent     */
  struct datalinkMessageAutopilotDels_ref   *autopilotDels; /* raw message sent     */
  struct datalinkMessageTruth_ref           *truthSim;      /* raw message received */
  struct datalinkMessageHITLSim2Onboard_ref *sim2onboard;   /* raw message received */
  struct datalinkMessageHITLOnboard2Sim_ref *onboard2sim;   /* raw message sent     */
  struct datalinkMessageOptitrack_ref       *optitrack;     /* raw message received */
  struct datalinkMessage_My_navout_ref      *state;
  struct datalinkMessage_Sensor_Data        *Sensor_Data;
  struct datalinkMessage_PID                *PID_Tune;
};

struct datalinkWork_ref {
	uint32_t itime; /* number of messages received */
	int32_t badChecksums; /* */
	int32_t badHeaderChecksums; /*  */
};

struct datalinkHeader_ref {
	unsigned char sync1; /*  */
	unsigned char sync2; /*  */
	unsigned char sync3; /*  */
	unsigned char spare; /*  */
	int32_t messageID; /* id # */
	int32_t messageSize; /* including header */
	uint32_t hcsum; /*  */
	uint32_t csum; /*  */
};

struct datalinkMessage0_ref {
  unsigned char sync1; /*  */
  unsigned char sync2; /*  */
  unsigned char sync3; /*  */
  unsigned char spare; /*  */
  int messageID; /* id # */
  int messageSize; /* including header */
  unsigned int hcsum; /*  */
  unsigned int csum; /*  */
  char navStatus; /* status of nav system */
  char gpsStatus; /* status of gps */
  char aglStatus; /* status of AGL sensor in bits (0,1), which AGL sensor source in bits (2-4) - see AGL_SENSOR_SOURCE enum in sensors.db */
  unsigned char overrun; /* frame overrun */
  char wow; /* weight on skids (0=weight off of skids = in air, 1=weight on skids = on ground, 2=uncertain) */
  char autopilot; /* autopilot engaged (0=rx,1=rx no integral, 2=auto,3=auto no integral,4=external,5=external no integral,6=auto no outer loop) */
  char LaunchState; /* 1=arm, 0=disarm */
  unsigned char motor; /* motor state */
  float time; /* onboard time */
  float pos[3]; /* position of vehicle */
  float vel[3]; /* velocity of vehicle */
  float q[4]; /* attitude */
  float altitudeAGL; /* altitude above terrain */
};

struct datalinkMessage1_ref {
  unsigned char sync1; /*  */
  unsigned char sync2; /*  */
  unsigned char sync3; /*  */
  unsigned char spare; /*  */
  int messageID; /* id # */
  int messageSize; /* including header */
  unsigned int hcsum; /*  */
  unsigned int csum; /*  */
  float time; /* onboard time */
  char numberOfSats; /* number of sats for GPS */
  char datarecordStatus; /* status of data recording */
  char safemode; /* safe mode status */
  unsigned char type; /* vehicle type */
  float delm[3]; /* actuators */
  float delf[1]; /* actuators */
  float delt[1]; /* actuators */
  float delc[1]; /* actuators */
  int battery; /* battery voltage (mV) */
  int current; /* total current (mA) */
  int rpm; /* rpm from hub */
  char tx; /* transmitter status */
  char fuel; /* fuel status */
  char ycsStatus; /* YCS status */
  unsigned char uniqueID; /* tail number */
  char yrdStatus; /* YRD status */
  char hubStatus; /* RPM status */
  char rangeFinderStatus; /* status by bits (0,1) = number of AGL sensors (0-3), (2,3) = 2nd AGL sensor status,(4,5) = 3rd AGL sensor status, (6,7) = status of a range finder not being used as an AGL sensor (off if there isn't one) */
  char magnetStatus; /* status of magnetometer */
  float traj_x[3]; /* trajectory */
  float traj_v[3]; /* trajectory */
  float traj_a[3]; /* trajectory */
  float traj_q[4]; /* trajectory */
  float traj_psi; /* trajectory */
  float traj_vscale; /* trajectory */
  short traj_manIndex; /* trajectory */
  unsigned char align[1]; /*  */
  unsigned char actuatorInterfaceStatus; /* Status for low-level actuator interface (0=armed,1=unarmed,2=armed without SAS,3=unarmed without SAS,4=failure/no response) */
  unsigned char imuStatus; /* imu status */
  unsigned char traj_status; /* trajectory */
  unsigned char visionStatus; /* vision satus */
  unsigned char missionStatus; /* mision status */
  unsigned char otherStatus; /* other Status */
  unsigned char cameraControlStatus; /* cameraControl Status */
  unsigned char historyStatus; /* onboard computers history status */
  unsigned char batteryStatus; /*  */
  unsigned char uplinkStatus[2]; /* uplink status */
  unsigned char lostComm; /* lost comm triggered */
  unsigned char hokuyoLaserStatus; /* Hokuyo laser status  */
  unsigned char ubloxSNR; /* average */
  unsigned char ubloxHacc; /* (dm) horizontal accuracy */
  unsigned char ubloxSacc; /* (dm/s) speed accuracy */
  unsigned char ubloxPDOP; /* (*10) */
  float pan; /* (deg) */
  float tilt; /* (deg) */
  float roll; /* (deg) */
  float fovy; /* (deg) */
  float G; /* load factor */
  float wind[3]; /* (ft/sec) */
  float pointPos[3]; /* (ft) */
  float datumLat; /* datum latitude (deg-N) */
  float datumLon; /* datum longitude (deg-E) */
  float datumAlt; /* datum altitude (ft) */
};

struct datalinkMessageUp0_ref {
  unsigned char sync1; /*  */
  unsigned char sync2; /*  */
  unsigned char sync3; /*  */
  unsigned char spare; /*  */
  int messageID; /* id # */
  int messageSize; /* including header */
  unsigned int hcsum; /*  */
  unsigned int csum; /*  */
  int k; /* discrete time onboard */
  float time; /* onboard time */
  float throttleLever; /*  */
  float rollStick; /*  */
  float pitchStick; /*  */
  float rudderPedal; /*  */
  char button[16]; /* button flags */
};

struct datalinkMessagePWM_ref {
  unsigned char sync1; /*  */
  unsigned char sync2; /*  */
  unsigned char sync3; /*  */
  unsigned char spare; /*  */
  int messageID; /* id # */
  int messageSize; /* including header */
  unsigned int hcsum; /*  */
  unsigned int csum; /*  */
  unsigned short raw_pwm[17]; /* */
  unsigned short align; /* */
};

struct datalinkMessageAutopilotDels_ref {
  unsigned char sync1; /*  */
  unsigned char sync2; /*  */
  unsigned char sync3; /*  */
  unsigned char spare; /*  */
  int messageID; /* id # */
  int messageSize; /* including header */
  unsigned int hcsum; /*  */
  unsigned int csum; /*  */
  float time; /* onboard time */
  float c_delm[3]; /*  */
  float c_delf[1]; /*  */
  float c_delt[1]; /*  */
  float PWM[4];
  float Roll;
  float Pitch;
  float Yaw;
  float Throttle;
  float Kill;
  float Auto;

};

struct datalinkMessage_My_navout_ref {
  unsigned char sync1; /*  */
  unsigned char sync2; /*  */
  unsigned char sync3; /*  */
  unsigned char spare; /*  */
  int messageID; /* id # */
  int messageSize; /* including header */
  unsigned int hcsum; /*  */
  unsigned int csum; /*  */
  float p_b_e_L[3]; /* position of body wrt Earth expressed in local frame  */
  float v_b_e_L[3]; /* velocity of body wrt Earth expressed in local frame */
  float a_b_e_L[3]; /* accel of body wrt Earth expressed in local frame */
  float v_b_e_B[3]; /* velocity of body wrt Earth expressed in body */
  float a_b_e_B[3]; /* accel of body wrt Earth expressed in body frame */
  float w_b_e_B[3]; /* angular rate of body wrt Earth expressed in body (average of 2 frames) */
  float q[4]; /* attitude quaternion */
  float phi; /*  */
  float theta; /*  */
  float psi; /*  */
  float time; /*  */
};

struct datalinkMessageTruth_ref {
  unsigned char sync1; /*  */
  unsigned char sync2; /*  */
  unsigned char sync3; /*  */
  unsigned char spare; /*  */
  int messageID; /* id # */
  int messageSize; /* including header */
  unsigned int hcsum; /*  */
  unsigned int csum; /*  */
  unsigned int align; /*  */
  float p_b_e_L[3]; /*  */
  float v_b_e_L[3]; /*  */
  float q[4]; /*  */
};

struct datalinkMessageHITLSim2Onboard_ref {
  unsigned char sync1; /*  */
  unsigned char sync2; /*  */
  unsigned char sync3; /*  */
  unsigned char spare; /*  */
  int messageID; /* id # */
  int messageSize; /* including header */
  unsigned int hcsum; /*  */
  unsigned int csum; /*  */
  float phiCmd; /*  */
  float posDes_x; /*  */
  float posDes_y; /*  */
  float posDes_z; /*  */
  float nav_p_x; /*  */
  float nav_p_y; /*  */
  float nav_p_z; /*  */
  float nav_v_x; /*  */
  float nav_v_y; /*  */
  float nav_v_z; /*  */
  float nav_w_x; /*  */
  float nav_w_y; /*  */
  float nav_w_z; /*  */
  float nav_phi; /*  */
  float nav_theta; /*  */
  float nav_psi; /*  */
};

struct datalinkMessageHITLOnboard2Sim_ref {
  unsigned char sync1; /*  */
  unsigned char sync2; /*  */
  unsigned char sync3; /*  */
  unsigned char spare; /*  */
  int messageID; /* id # */
  int messageSize; /* including header */
  unsigned int hcsum; /*  */
  unsigned int csum; /*  */
  float c_delf; /*  */
  float c_delm0; /*  */
  float c_delm1; /*  */
  float c_delm2; /*  */
};

struct datalinkMessageOptitrack_ref {
  unsigned char sync1; /*  */
  unsigned char sync2; /*  */
  unsigned char sync3; /*  */
  unsigned char spare; /*  */
  int messageID; /* id # */
  int messageSize; /* including header */
  unsigned int hcsum; /*  */
  unsigned int csum; /*  */
  float pos_x; /*  */
  float pos_y; /*  */
  float pos_z; /*  */
  float qx; /*  */
  float qy; /*  */
  float qz; /*  */
  float qw; /*  */
  int frameNum; /*  */
  int valid; /*   */
};

struct datalinkMessage_Sensor_Data {
  unsigned char sync1; /*  */
  unsigned char sync2; /*  */
  unsigned char sync3; /*  */
  unsigned char spare; /*  */
  int messageID; /* id # */
  int messageSize; /* including header */
  unsigned int hcsum; /*  */
  unsigned int csum; /*  */
  float IMU_Gyro_Data[3]; /*  */
  float IMU_Accel_Data[3]; /*  */
  int IMU_Count; /*  */
  float Mo_Cap_Pos_Data[3]; /*  */
  float Mo_Cap_Orientation_Data[4]; /*  */
  int Mo_Cap_Update; /*  */
};

struct datalinkMessage_PID {
  unsigned char sync1; /*  */
  unsigned char sync2; /*  */
  unsigned char sync3; /*  */
  unsigned char spare; /*  */
  int messageID; /* id # */
  int messageSize; /* including header */
  unsigned int hcsum; /*  */
  unsigned int csum; /*  */
  float KP[3]; /*  */
  float KD[3]; /*  */
  float KI[3]; /*  */
};

extern struct obDatalink_ref obDatalink;
extern struct datalinkHeader_ref obDatalinkMessageHeader;
extern struct datalinkMessage0_ref obDatalinkMessage0;
extern struct datalinkMessage1_ref obDatalinkMessage1;
extern struct datalinkMessageUp0_ref obDatalinkMessageUp0;
extern struct datalinkMessagePWM_ref obDatalinkMessagePWM;
extern struct datalinkMessageAutopilotDels_ref obDatalinkMessageAutopilotDels;
extern struct datalinkMessageTruth_ref obDatalinkMessageTruth;
extern struct datalinkMessageHITLSim2Onboard_ref obDatalinkMessageSim2Onboard;
extern struct datalinkMessageHITLOnboard2Sim_ref obDatalinkMessageOnboard2Sim;
extern struct datalinkMessageOptitrack_ref obDatalinkMessageOptitrack;
extern struct datalinkMessage_My_navout_ref gcs0DatalinkMessage_Drone_State;
extern struct datalinkMessage_Sensor_Data gcs0DatalinkMessage_Sensor_Data;
extern struct datalinkMessage_PID gcs0DatalinkMessage_PID;

extern wifi ether;
extern RC_PILOT rc;

class Dlink
{
public:
  Dlink();
  ~Dlink();

  void init();
  void recv_update(Sensors* sens );
  void send_update(Sensors sens, double c_delf, double c_delm_0, double c_delm_1, double c_delm_2,float Throttle, float Roll, float Pitch,float Yaw,float Manual,float Kill,float PWM0,float PWM1,float PWM2,float PWM3);

  void set_interval( long intervalX, int type );
private:
  long intervalUp0;
  long intervalPWM;
  long intervalAutopilotDels;
  long intervalTruthSim;
  long intervalSim2Onboard;
  long intervalOnboard2Sim;
  long intervalOptitrack;
  long intervalDrone_State;
  long intervalSensor_Data;
  long intervalPID;
  unsigned long previousMillisUp0;
  unsigned long previousMillisPWM;
  unsigned long previousMillisAutopilotDels;
  unsigned long previousMillisTruthSim;
  unsigned long previousMillisSim2Onboard;
  unsigned long previousMillisOnboard2Sim;
  unsigned long previousMillisOptitrack;
  unsigned long previousMillisDrone_State;
  unsigned long previousMillisSensor_Data;
  unsigned long previousMillisPID;
};

uint32_t datalinkCheckSumCompute(unsigned char* buf, int32_t byteCount);
void datalinkCheckSumEncode(unsigned char* buf, uint32_t byteCount);

void readDatalink( WiFiUDP* wf,Sensors* sens );
void writeM0( WiFiUDP* wf );
void writeM1( WiFiUDP* wf );
void writeUP0( WiFiUDP* wf, RC_PILOT* rc );
void writePWM( WiFiUDP* wf );
void writeAutopilotDels( WiFiUDP* wf ,double c_delf, double c_delm_0, double c_delm_1, double c_delm_2,float Throttle, float Roll, float Pitch,float Yaw,float Manual,float Kill,float PWM_0,float PWM_1,float PWM_2,float PWM_3);
void write_Drone_State( WiFiUDP* wf );
void write_Sensor_Data( WiFiUDP* wf,Sensors sens );
#endif