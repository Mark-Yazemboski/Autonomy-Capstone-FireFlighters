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


#include "../include/datalink.h"
#include "../include/wifi.h"
#include "../include/rc_pilot.h"
#include "myekf_ref.h"
#include "sensors.h"
#include "controller_ref.h"
#include "controller.h"
#define METERS_TO_FEET 3.28084

void quat2euler2(double* q, double* phi, double* theta, double* psi) {
  // Extract quaternion elements
  float q0 = q[0]; // w (scalar)
  float q1 = q[1]; // x
  float q2 = q[2]; // y
  float q3 = q[3]; // z

  // Roll (phi) - rotation around X-axis
  *phi = atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));

  // Pitch (theta) - rotation around Y-axis
  float sin_theta = 2.0 * (q0 * q2 - q3 * q1);
  if (fabs(sin_theta) >= 1.0) {
      *theta = copysign(M_PI / 2.0, sin_theta); // Use 90 degrees if out of range
  }
  else {
      *theta = asin(sin_theta);
  }

  // Yaw (psi) - rotation around Z-axis
  *psi = atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));
}

struct datalinkWork_ref obDatalinkWork = {
	0 , /* uint itime */
	0 , /* int badChecksums */
	0 , /* int badHeaderChecksums */
};

struct datalinkHeader_ref obDatalinkMessageHeader = {
	0xa3 , /* uchar sync1 */
	0xb2 , /* uchar sync2 */
	0xc1 , /* uchar sync3 */
	0 , /* uchar spare */
	0 , /* int messageID */
	0 , /* int messageSize */
	0 , /* uint hcsum */
	0 , /* uint csum */
};

struct datalinkMessage0_ref obDatalinkMessage0 = {
  0xa3 , /* uchar sync1 */
  0xb2 , /* uchar sync2 */
  0xc1 , /* uchar sync3 */
  0 , /* uchar spare */
  0 , /* int messageID */
  0 , /* int messageSize */
  0 , /* uint hcsum */
  0 , /* uint csum */
  0 , /* char navStatus */
  0 , /* char gpsStatus */
  0 , /* char aglStatus */
  0 , /* uchar overrun */
  0 , /* char wow */
  0 , /* char autopilot */
  0 , /* char LaunchState */
  0 , /* uchar motor */
  0 , /* float time */
  {0,0,-2} , /* float pos[3] */
  {0,0,0}  , /* float vel[3] */
  {1,0,0,0} , /* float q[4] */
  2 , /* float altitudeAGL */
};

struct datalinkMessage1_ref obDatalinkMessage1 = {
  0xa3 , /* uchar sync1 */
  0xb2 , /* uchar sync2 */
  0xc1 , /* uchar sync3 */
  0 , /* uchar spare */
  0 , /* int messageID */
  0 , /* int messageSize */
  0 , /* uint hcsum */
  0 , /* uint csum */
  0 , /* float time */
  0 , /* char numberOfSats */
  0 , /* char datarecordStatus */
  0 , /* char safemode */
  0 , /* uchar type */
  {0,0,0} , /* float delm[3] */
  {0} , /* float delf[1] */
  {0} , /* float delt[1] */
  {0} , /* float delc[1] */
  12000  , /* int battery */
  0      , /* int current */
  0          , /* int rpm */
  0          , /* char tx */
  0        , /* char fuel */
  0   , /* char ycsStatus */
  0   , /* uchar uniqueID */
  0   , /* char yrdStatus */
  0   , /* char hubStatus */
  0 , /* char rangeFinderStatus */
  0 , /* char magnetStatus */
  {0,0,0}   , /* float traj_x[3] */
  {0,0,0}   , /* float traj_v[3] */
  {0,0,0}   , /* float traj_a[3] */
  {0,0,0,0} , /* float traj_q[4] */
  0         , /* float traj_psi */
  1.0     , /* float traj_vscale */
  0     , /* short traj_manIndex */
  {0} , /* uchar align[1] */
  0 , /* uchar actuatorInterfaceStatus */
  0 , /* uchar imuStatus */
  0 , /* uchar traj_status */
  0 , /* uchar visionStatus */
  0 , /* uchar missionStatus */
  0 , /* uchar otherStatus */
  0  , /* uchar cameraControlStatus */
  0  , /* uchar historyStatus */
  0 , /* uchar batteryStatus */
  {0,0} , /* uchar uplinkStatus[2] */
  0 , /* uchar lostComm */
  0 , /* uchar hokuyoLaserStatus */
  0 , /* uchar ubloxSNR */
  0 , /* uchar ubloxHacc */
  0 , /* uchar ubloxSacc */
  0 , /* uchar ubloxPDOP */
  0.0 , /* float pan */
  0.0 , /* float tilt */
  0.0 , /* float roll */
  58.5 , /* float fovy */
  1.0 , /* float G */
  {0.0,0.0,0.0} , /* float wind[3] */
  {30.0,0.0,0.0} , /* float pointPos[3] */
  33.659653f , /* float datumLat */
  -84.663333f , /* float datumLon */
  745.00f     , /* float datumAlt */
};

struct datalinkMessageUp0_ref obDatalinkMessageUp0 = {
  0xa3 , /* uchar sync1 */
  0xb2 , /* uchar sync2 */
  0xc1 , /* uchar sync3 */
  0 , /* uchar spare */
  0 , /* int messageID */
  0 , /* int messageSize */
  0 , /* uint hcsum */
  0 , /* uint csum */
  0 , /* int k */
  0 , /* float time */
  0 , /* float throttleLever */
  0 , /* float rollStick */
  0 , /* float pitchStick */
  0 , /* float rudderPedal */
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} , /* char button[16] */
};

struct datalinkMessagePWM_ref obDatalinkMessagePWM = {
  0xa3 , /* uchar sync1 */
  0xb2 , /* uchar sync2 */
  0xc1 , /* uchar sync3 */
  0 , /* uchar spare */
  0 , /* int messageID */
  0 , /* int messageSize */
  0 , /* uint hcsum */
  0 , /* uint csum */
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} , /* raw_pwm[17] */
  0, /* align */
};

struct datalinkMessageAutopilotDels_ref gcs0DatalinkMessageAutopilotDels = {
  0xa3 , /* uchar sync1 */
  0xb2 , /* uchar sync2 */
  0xc1 , /* uchar sync3 */
  0 , /* uchar spare */
  0 , /* int messageID */
  0 , /* int messageSize */
  0 , /* uint hcsum */
  0 , /* uint csum */
  0 , /* float time */
  {0,0,0} , /* float c_delm[3] */
  {0} , /* float c_delf[1] */
  {0} , /* float c_delt[1] */
  {0,0,0,0} , /* float PWM[4] */
  {0}, /* float KP[3] */
  {0}, /* float KD[3] */
  {0}, /* float KI[3] */
  0 , /* float Roll */
  0 , /* float Pitch */
  0 , /* float Yaw */
  0 , /* float Throttle */
  0 , /* float Kill */
  0 , /* float Auto */
};

struct datalinkMessageTruth_ref obDatalinkMessageTruth = {
  0xa3 , /* uchar sync1 */
  0xb2 , /* uchar sync2 */
  0xc1 , /* uchar sync3 */
  0 , /* uchar spare */
  0 , /* int messageID */
  0 , /* int messageSize */
  0 , /* uint hcsum */
  0 , /* uint csum */
  0 , /* uint align  */
  {0,0,0} , /* float p_b_e_L[3]  */
  {0,0,0} , /* float v_b_e_L[3]  */
  {0,0,0,0} , /* float q[4]  */
};

struct datalinkMessageHITLSim2Onboard_ref obDatalinkMessageSim2Onboard = {
  0xa3 , /* uchar sync1 */
  0xb2 , /* uchar sync2 */
  0xc1 , /* uchar sync3 */
  0 , /* uchar spare */
  0 , /* int messageID */
  0 , /* int messageSize */
  0 , /* uint hcsum */
  0 , /* uint csum */
  0 , /* float phiCmd */
  0 , /* float posDes_x */
  0 , /* float posDes_y */
  0 , /* float posDes_z */
  0 , /* float nav_p_x  */
  0 , /* float nav_p_y  */
  0 , /* float nav_p_z  */
  0 , /* float nav_v_x  */
  0 , /* float nav_v_y  */
  0 , /* float nav_v_z  */
  0 , /* float nav_w_x  */
  0 , /* float nav_w_y  */
  0 , /* float nav_w_z  */
  0 , /* float nav_phi  */
  0 , /* float nav_theta  */
  0 , /* float nav_psi  */
};

struct datalinkMessageHITLOnboard2Sim_ref obDatalinkMessageOnboard2Sim = {
  0xa3 , /* uchar sync1 */
  0xb2 , /* uchar sync2 */
  0xc1 , /* uchar sync3 */
  0 , /* uchar spare */
  0 , /* int messageID */
  0 , /* int messageSize */
  0 , /* uint hcsum */
  0 , /* uint csum */
  0 , /* float c_delf; */
  0 , /* float c_delm0; */
  0 , /* float c_delm1; */
  0 , /* float c_delm2; */
};

struct datalinkMessageOptitrack_ref obDatalinkMessageOptitrack = {
  0xa3 , /* uchar sync1 */
  0xb2 , /* uchar sync2 */
  0xc1 , /* uchar sync3 */
  0 , /* uchar spare */
  0 , /* int messageID */
  0 , /* int messageSize */
  0 , /* uint hcsum */
  0 , /* uint csum */
  0,
  0,
  0,
  0,
  0,
  0,
  0,
};


struct datalinkMessage_My_navout_ref gcs0DatalinkMessage_Drone_State = {
  0xa3 , /* uchar sync1 */
  0xb2 , /* uchar sync2 */
  0xc1 , /* uchar sync3 */
  0 , /* uchar spare */
  0 , /* int messageID */
  0 , /* int messageSize */
  0 , /* uint hcsum */
  0 , /* uint csum */
  {0}, /* double p_b_e_L[3] */
  {0}, /* double v_b_e_L[3] */
  {0}, /* double a_b_e_L[3] */
  {0}, /* double v_b_e_B[3] */
  {0}, /* double a_b_e_B[3] */
  {0}, /* double w_b_e_B[3] */
  {0}, /* double q[4] */
  0, /* double phi */
  0, /* double theta */
  0, /* double psi */
  0, /* double Dt */
  0, /* bool fire found*/
  {0,0}, /*float Fire Location*/
  0, /*bool Payload Dropped*/
  {0,0,0}, /*float Desired Location*/
  0, /*int Waypoint Number*/
  0, /*Numer Of Waypoints*/
};


struct datalinkMessage_Sensor_Data gcs0DatalinkMessage_Sensor_Data = {
  0xa3 , /* uchar sync1 */
  0xb2 , /* uchar sync2 */
  0xc1 , /* uchar sync3 */
  0 , /* uchar spare */
  0 , /* int messageID */
  0 , /* int messageSize */
  0 , /* uint hcsum */
  0 , /* uint csum */
  {0}, /* float IMU_Gyro_Data[3] */
  {0}, /* float IMU_Accel_Data[3] */
  {0}, /* float IMU_Angles[3]*/
  0 , /* int IMU_Count */
  {0}, /* float Mo_Cap_Pos_Data[3] */
  {0}, /* float Mo_Cap_Orientation_Data[4] */
  {0}, /* float Mo_Cap_Euler_Angles[3] */
  0 , /* int Mo_Cap_Update */
};

struct datalinkMessage_PID gcs0DatalinkMessage_PID = {
  0xa3 , /* uchar sync1 */
  0xb2 , /* uchar sync2 */
  0xc1 , /* uchar sync3 */
  0 , /* uchar spare */
  0 , /* int messageID */
  0 , /* int messageSize */
  0 , /* uint hcsum */
  0 , /* uint csum */
  {0}, /* float KP[3] */
  {0}, /* float KD[3] */
  {0}, /* float KI[3] */
};

struct obDatalink_ref obDatalink = {
  &obDatalinkWork, /* dir work */
	&obDatalinkMessageHeader, /* dir header */
  &obDatalinkMessage0, /* dir m0 */
  &obDatalinkMessage1, /* dir m1 */
  &obDatalinkMessageUp0, /* dir up0 */
  &obDatalinkMessagePWM, /* dir pwm */
  &gcs0DatalinkMessageAutopilotDels, /* dir autopilotDels */
  &obDatalinkMessageTruth, /* dir truthSim */
  &obDatalinkMessageSim2Onboard, /* dir sim2onboard */
  &obDatalinkMessageOnboard2Sim, /* dir onboard2sim */
  &obDatalinkMessageOptitrack, /* dir optitrack */
  &gcs0DatalinkMessage_Drone_State,
  &gcs0DatalinkMessage_Sensor_Data,
  &gcs0DatalinkMessage_PID,
};

extern unsigned short pwm_out[4];

Dlink::Dlink(){

}

Dlink::~Dlink(){}

void Dlink::init(){
  // initialize wifi
  ether.init();
  // initialize datalink
  obDatalink.work = &obDatalinkWork;
  obDatalink.header = &obDatalinkMessageHeader;
  obDatalink.m0 = &obDatalinkMessage0;
  obDatalink.m1 = &obDatalinkMessage1;
  obDatalink.up0 = &obDatalinkMessageUp0;
  obDatalink.pwm = &obDatalinkMessagePWM;
  obDatalink.autopilotDels = &gcs0DatalinkMessageAutopilotDels;
  obDatalink.truthSim = &obDatalinkMessageTruth;
  obDatalink.sim2onboard = &obDatalinkMessageSim2Onboard;
  obDatalink.onboard2sim = &obDatalinkMessageOnboard2Sim;
  obDatalink.optitrack = &obDatalinkMessageOptitrack;
  obDatalink.state = &gcs0DatalinkMessage_Drone_State;

  // set_interval( DATALINK_INTERVALUP0, DATALINK_MESSAGE_UP0 );
  // set_interval( DATALINK_INTERVALPWM, DATALINK_MESSAGE_PWM );
  // set_interval( DATALINK_INTERVALAUTOPILOTDELS, DATALINK_MESSAGE_AUTOPILOTDELS );
  // set_interval( DATALINK_INTERVALTRUTH, DATALINK_MESSAGE_TRUTH );
  // set_interval( DATALINK_INTERVALHITL_SIM2ONBOARD, DATALINK_MESSAGE_HITL_SIM2ONBOARD );
  // set_interval( DATALINK_INTERVALHITL_ONBOARD2SIM, DATALINK_MESSAGE_HITL_ONBOARD2SIM );
  // set_interval( DATALINK_INTERVALOPTITRACK, DATALINK_MESSAGE_OPTITRACK );
  // set_interval( DATALINK_INTERVAL_DRONE_STATE, DATALINK_MESSAGE_DRONE_STATE );
}

// void Dlink::set_interval( long intervalX, int type ){
//   switch ( type ){
//     case DATALINK_MESSAGE_UP0:
//       obDatalink.up0->messageSize = sizeof( struct datalinkMessageUp0_ref );
//       obDatalink.up0->messageID = DATALINK_MESSAGE_UP0;
//       this->intervalUp0 = intervalX;
//       this->previousMillisUp0 = 0;
//       break;
//     case DATALINK_MESSAGE_PWM:
//       obDatalink.pwm->messageSize = sizeof( struct datalinkMessagePWM_ref );
//       obDatalink.pwm->messageID = DATALINK_MESSAGE_PWM;
//       this->intervalPWM = intervalX;
//       this->previousMillisPWM = 0;
//       break;
//     case DATALINK_MESSAGE_AUTOPILOTDELS:
//       obDatalink.autopilotDels->messageSize = sizeof( struct datalinkMessageAutopilotDels_ref );
//       obDatalink.autopilotDels->messageID = DATALINK_MESSAGE_AUTOPILOTDELS;
//       this->intervalAutopilotDels = intervalX;
//       this->previousMillisAutopilotDels = 0;
//       break;
//     case DATALINK_MESSAGE_TRUTH:
//       obDatalink.truthSim->messageSize = sizeof( struct datalinkMessageTruth_ref );
//       obDatalink.truthSim->messageID = DATALINK_MESSAGE_TRUTH;
//       this->intervalTruthSim = intervalX;
//       this->previousMillisTruthSim = 0;
//       break;
//     case DATALINK_MESSAGE_HITL_SIM2ONBOARD:
//       obDatalink.sim2onboard->messageSize = sizeof( struct datalinkMessageHITLSim2Onboard_ref );
//       obDatalink.sim2onboard->messageID = DATALINK_MESSAGE_HITL_SIM2ONBOARD;
//       this->intervalSim2Onboard = intervalX;
//       this->previousMillisSim2Onboard = 0;
//       break;
//     case DATALINK_MESSAGE_HITL_ONBOARD2SIM:
//       obDatalink.onboard2sim->messageSize = sizeof( struct datalinkMessageHITLOnboard2Sim_ref );
//       obDatalink.onboard2sim->messageID = DATALINK_MESSAGE_HITL_ONBOARD2SIM;
//       this->intervalOnboard2Sim = intervalX;
//       this->previousMillisOnboard2Sim = 0;
//       break;
//     case DATALINK_MESSAGE_OPTITRACK:
//       obDatalink.optitrack->messageSize = sizeof( struct datalinkMessageOptitrack_ref );
//       obDatalink.optitrack->messageID = DATALINK_MESSAGE_OPTITRACK;
//       this->intervalOptitrack = intervalX;
//       this->previousMillisOptitrack = 0;
//       break;
//     case DATALINK_MESSAGE_DRONE_STATE:
//       obDatalink.state->messageSize = sizeof( struct datalinkMessage_My_navout_ref );
//       obDatalink.state->messageID = DATALINK_MESSAGE_DRONE_STATE;
//       this->intervalDrone_State = intervalX;
//       this->previousMillisDrone_State = 0;
//       break;
//     case DATALINK_MESSAGE_SENSOR_DATA:
//       obDatalink.state->messageSize = sizeof( struct datalinkMessage_Sensor_Data );
//       obDatalink.state->messageID = DATALINK_MESSAGE_SENSOR_DATA;
//       this->intervalSensor_Data = intervalX;
//       this->previousMillisSensor_Data = 0;
//       break;

//     case DATALINK_MESSAGE_PID:
//       obDatalink.PID_Tune->messageSize = sizeof( struct datalinkMessage_PID );
//       obDatalink.PID_Tune->messageID = DATALINK_MESSAGE_PID;
//       this->intervalPID = intervalX;
//       this->previousMillisSensor_Data = 0;
//       break;
//     default:
//       Serial.println("Error: Invalid type for datalink interval");
//       break;
//   }
// }

void Dlink::recv_update(Sensors* sens_pointer,bool MoCap){
  // messages to receive from GCS: TruthSim, Sim2Onboard, Optitrack
  // read datalink optitrack
  if (MoCap){
    readDatalink( &ether.UdpGPS,sens_pointer );
  }
  
  // read datalink gcs
  readDatalink( &ether.UdpGCS,sens_pointer );
}

void Dlink::send_update(Sensors sens,double c_delf, double c_delm_0, double c_delm_1, double c_delm_2,float Throttle, float Roll, float Pitch,float Yaw,float Manual,float Kill,float PWM_0,float PWM_1,float PWM_2,float PWM_3){
  // messages to send to GCS: AutopiloDels, Up0, PWM, Onboard2Sim
  // Serial.println("Sending_Data");
  // unsigned long currentMillis = millis();
  // if (currentMillis - this->previousMillisAutopilotDels >= this->intervalAutopilotDels) {
  //   this->previousMillisAutopilotDels = currentMillis;
  //writeAutopilotDels( &ether.UdpGCS, c_delf,  c_delm_0,  c_delm_1,  c_delm_2, Throttle,  Roll,  Pitch, Yaw, Manual, Kill,PWM_0,PWM_1,PWM_2,PWM_3);
  // }
  // currentMillis = millis();
  // if (currentMillis - this->previousMillisDrone_State >= this->intervalDrone_State) {
  //   this->previousMillisDrone_State = currentMillis;
  write_Drone_State( &ether.UdpGCS );
  // }

  // currentMillis = millis();
  // if (currentMillis - this->previousMillisSensor_Data >= this->intervalSensor_Data) {
  //   this->previousMillisSensor_Data = currentMillis;
  //write_Sensor_Data( &ether.UdpGCS, sens);
  // }

  writeM0( &ether.UdpGCS);

}


uint32_t datalinkCheckSumCompute(unsigned char* buf, int32_t byteCount) {

	uint32_t sum1 = 0xffff;
	uint32_t sum2 = 0xffff;
	uint32_t tlen = 0;
	uint32_t shortCount = byteCount / sizeof(short);
	uint32_t oddLength = byteCount % 2;


	/* this is Fletcher32 checksum modified to handle buffers with an odd number of bytes */

	while (shortCount)
	{
		/* 360 is the largest number of sums that can be performed without overflow */
		tlen = shortCount > 360 ? 360 : shortCount;
		shortCount -= tlen;
		do
		{
			sum1 += *buf++;
			sum1 += ((uint32_t)*buf++ << 8);
			sum2 += sum1;
		} while (--tlen);

		/* add last byte if there's an odd number of bytes (equivalent to appending a zero-byte) */
		if ((oddLength == 1) && (shortCount < 1))
		{
			sum1 += (uint32_t)*buf++;
			sum2 += sum1;
		}

		sum1 = (sum1 & (uint32_t)0xffff) + (sum1 >> 16);
		sum2 = (sum2 & (uint32_t)0xffff) + (sum2 >> 16);
	}

	/* Second reduction step to reduce sums to 16 bits */
	sum1 = (sum1 & (uint32_t)0xffff) + (sum1 >> 16);
	sum2 = (sum2 & (uint32_t)0xffff) + (sum2 >> 16);

	return(sum2 << 16 | sum1);
}

void datalinkCheckSumEncode(unsigned char* buf, uint32_t byteCount) {

	struct datalinkHeader_ref* h = (struct datalinkHeader_ref*)buf;

	h->sync1 = DATALINK_SYNC0;
	h->sync2 = DATALINK_SYNC1;
	h->sync3 = DATALINK_SYNC2;

	h->messageSize = byteCount;

	h->hcsum = datalinkCheckSumCompute(buf, sizeof(struct datalinkHeader_ref) - sizeof(int32_t) * 2);
	h->csum = datalinkCheckSumCompute(&(buf[sizeof(struct datalinkHeader_ref)]), byteCount - sizeof(struct datalinkHeader_ref));
}

void readDatalink( WiFiUDP* wf, Sensors* sens_pointer )
{
	struct obDatalink_ref* data = &obDatalink;

	int index, done;
	unsigned char* bf;
	void* dataPtr;
	int size;
	FILE* filep;

	int packetSize = wf->parsePacket();

	if ( packetSize == 0 ) return;
	done = 0;
	index = 0;

  int bytesread = wf->read(ether.buffer, BUFFERSIZE);

	while ( ( index <= bytesread - ( int ) sizeof( struct datalinkHeader_ref ) ) && !done )
	{
		if ( ( ether.buffer[index] == DATALINK_SYNC0 ) &&
				 ( ether.buffer[index + 1] == DATALINK_SYNC1 ) &&
				 ( ether.buffer[index + 2] == DATALINK_SYNC2 ) )
		{
			bf = &( ether.buffer[index] );
			memcpy( data->header, bf, sizeof( struct datalinkHeader_ref ) );

			if ( datalinkCheckSumCompute( bf, sizeof( struct datalinkHeader_ref ) - sizeof( int ) * 2 ) == data->header->hcsum &&
					 data->header->messageSize >= sizeof( struct datalinkHeader_ref ) &&
					 data->header->messageSize < BUFFERSIZE )
			{
				if ( data->header->messageSize + index <= bytesread )
				{
					/* have read in the entire message */

					/*((struct datalinkHeader_ref *)bf)->hcsum = 0;*/
					if ( datalinkCheckSumCompute( &bf[sizeof( struct datalinkHeader_ref )], data->header->messageSize - sizeof( struct datalinkHeader_ref ) ) == data->header->csum )
					{
            //Serial.println(data->header->messageID);
            switch ( data->header->messageID )
						{
              case DATALINK_MESSAGE_OPTITRACK:
							if ( data->header->messageSize == sizeof( struct datalinkMessageOptitrack_ref ) )
							{
                memcpy( data->optitrack, bf, sizeof( struct datalinkMessageOptitrack_ref ) );
                sens_pointer->data_MoCap.MoCap_Valid = data->optitrack->valid;
                if (sens_pointer->data_MoCap.MoCap_Valid){
                  if (sens_pointer->data_MoCap.First){
                    sens_pointer->data_MoCap.First = false;
                    sens_pointer->data_MoCap.Calibration_Pos[0] = data->optitrack->pos_x*METERS_TO_FEET;
                    sens_pointer->data_MoCap.Calibration_Pos[1] = data->optitrack->pos_y*METERS_TO_FEET;
                    sens_pointer->data_MoCap.Calibration_Pos[2] = data->optitrack->pos_z*METERS_TO_FEET;
                  }
                  //Serial.println("Good data and recieving");
                  sens_pointer->data_MoCap.Pos[0] = data->optitrack->pos_x*METERS_TO_FEET - sens_pointer->data_MoCap.Calibration_Pos[0];
                  sens_pointer->data_MoCap.Pos[1] = data->optitrack->pos_y*METERS_TO_FEET - sens_pointer->data_MoCap.Calibration_Pos[1];
                  sens_pointer->data_MoCap.Pos[2] = data->optitrack->pos_z*METERS_TO_FEET - sens_pointer->data_MoCap.Calibration_Pos[2];
                      
                  sens_pointer->data_MoCap.Quat[0] = data->optitrack->qw;
                  sens_pointer->data_MoCap.Quat[1] = data->optitrack->qx;
                  sens_pointer->data_MoCap.Quat[2] = data->optitrack->qy;
                  sens_pointer->data_MoCap.Quat[3] = data->optitrack->qz;
                  if (data->optitrack->frameNum != sens_pointer->data_MoCap.MoCap_Frame_Counter){
                    sens_pointer->data_MoCap.MoCap_Frame_Counter = data->optitrack->frameNum;
                    sens_pointer->data_MoCap.MoCap_Update_Counter ++;
                  }

                  quat2euler2(sens_pointer->data_MoCap.Quat, &sens_pointer->data_MoCap.Euler_Angles[0], &sens_pointer->data_MoCap.Euler_Angles[1], &sens_pointer->data_MoCap.Euler_Angles[2]);
                  sens_pointer->data_MoCap.Euler_Angles[0] = sens_pointer->data_MoCap.Euler_Angles[0]*180/PI;
                  sens_pointer->data_MoCap.Euler_Angles[1] = sens_pointer->data_MoCap.Euler_Angles[1]*180/PI;
                  sens_pointer->data_MoCap.Euler_Angles[2] = sens_pointer->data_MoCap.Euler_Angles[2]*180/PI;

                  
                }
                //Serial.println("Valid");
                //Serial.println(data->optitrack->frameNum);
                // Serial.println("Begin");
                // Serial.print(sens_pointer->data_MoCap.Pos[0]);
                // Serial.print("\t ");
                // Serial.print(sens_pointer->data_MoCap.Pos[1]);
                // Serial.print("\t ");
                // Serial.print(sens_pointer->data_MoCap.Pos[2]);
                // Serial.print("\t ");
                // Serial.print(sens_pointer->data_MoCap.Quat[0]);
                // Serial.print("\t ");
                // Serial.print(sens_pointer->data_MoCap.Quat[1]);
                // Serial.print("\t ");
                // Serial.print(sens_pointer->data_MoCap.Quat[2]);
                // Serial.print("\t ");
                // Serial.println(sens_pointer->data_MoCap.Quat[3]);
                // Serial.println("End");
							}
							break;

              case DATALINK_MESSAGE_TRUTH:
							if ( data->header->messageSize == sizeof( struct datalinkMessageTruth_ref ) )
							{
                memcpy( data->truthSim, bf, sizeof( struct datalinkMessageTruth_ref ) );

							}
							break;

              case DATALINK_MESSAGE_HITL_SIM2ONBOARD:
							if ( data->header->messageSize == sizeof( struct datalinkMessageHITLSim2Onboard_ref ) )
							{
                memcpy( data->sim2onboard, bf, sizeof( struct datalinkMessageHITLSim2Onboard_ref ) );

							}
							break;

              case DATALINK_MESSAGE_PID:
							if ( data->header->messageSize == sizeof( struct datalinkMessage_PID ) )
							{
                memcpy( data->PID_Tune, bf, sizeof( struct datalinkMessage_PID ) );
                struct onboardControl_ref* cntrl = &onboardControl;
                int i;
                for (i = 0; i < 3; i++)
                  {
                     cntrl->KP_Rate[i] = data->PID_Tune->KP[i];
                     cntrl->KD_Rate[i] = data->PID_Tune->KD[i];
                     cntrl->KI_Rate[i] = data->PID_Tune->KI[i];

                  }

							}
							break;

							default:
							/* unrecognized message */
							break;
						}

						data->work->itime++;
					}
					else
					{ /* checksum bad */
						data->work->badChecksums++;
					}
					index += data->header->messageSize - 1;

				}
				else
				{ /* end of buffer includes a partial message - come back later... */
					index--;
					done = 1;
				}
			}
			else
			{ /* header checksum is bad */
				index += sizeof( struct datalinkHeader_ref ) - 1;
				data->work->badHeaderChecksums++;
			}
		}
		index++; /* start seq not found, go to next byte */

		if ( index < 0 ) index = BUFFERSIZE - 1;
	}
	// clearPort( port, index );
}


void writeAutopilotDels( WiFiUDP* wf ,double c_delf, double c_delm_0, double c_delm_1, double c_delm_2,float Throttle, float Roll, float Pitch,float Yaw,float Auto,float Kill,float PWM_0,float PWM_1,float PWM_2,float PWM_3)
{
  struct obDatalink_ref* data = &obDatalink;
  struct onboardControl_ref* cntrl = &onboardControl;
  data->autopilotDels->c_delf[0] = c_delf; // sample, need to be changed
  data->autopilotDels->c_delm[0] = c_delm_0; // sample, need to be changed
  data->autopilotDels->c_delm[1] = c_delm_1; // sample, need to be changed
  data->autopilotDels->c_delm[2] = c_delm_2; // sample, need to be changed
  data->autopilotDels->Throttle = Throttle;
  data->autopilotDels->Roll = Roll;
  data->autopilotDels->Pitch = Pitch;
  data->autopilotDels->Yaw = Yaw;
  data->autopilotDels->Auto = Auto;
  data->autopilotDels->Kill = Kill;
  data->autopilotDels->PWM[0] = PWM_0;
  data->autopilotDels->PWM[1] = PWM_1;
  data->autopilotDels->PWM[2] = PWM_2;
  data->autopilotDels->PWM[3] = PWM_3;
  
  // for (int i = 0; i < 3; i++) {
  //   data->autopilotDels->KP[i] = cntrl->KP[i];
  //   data->autopilotDels->KD[i] = cntrl->KD[i];
  //   data->autopilotDels->KI[i] = cntrl->KI[i];
  // }




  data->autopilotDels->messageID = DATALINK_MESSAGE_AUTOPILOTDELS;
	datalinkCheckSumEncode ( ( unsigned char* ) data->autopilotDels, sizeof ( struct datalinkMessageAutopilotDels_ref) );

  //wf->beginPacket(IPAddress(192,168,8,142), 10000); //448
  wf->beginPacket(IPAddress(192,168,1,2), 10000); //001
  wf->write(( char* ) data->autopilotDels, sizeof ( struct datalinkMessageAutopilotDels_ref));
  wf->endPacket();
}

void write_Drone_State( WiFiUDP* wf )
{
  struct obDatalink_ref* data = &obDatalink;
  struct My_Kalman_ref* EKF = &MyEKF;
  struct My_navout_ref* out = EKF->out;
  struct onboardControl_ref* cntrl = &onboardControl;
  
  for (int i = 0; i < 3; i++) {
    data->state->p_b_e_L[i] = out->p_b_e_L[i];
    data->state->v_b_e_L[i] = out->v_b_e_L[i];
    data->state->a_b_e_L[i] = out->a_b_e_L[i];
    data->state->v_b_e_B[i] = out->v_b_e_B[i];
    data->state->a_b_e_B[i] = out->a_b_e_B[i];
    data->state->w_b_e_B[i] = out->w_b_e_B[i];
  }
  for (int i = 0; i < 4; i++) {
    data->state->q[i] = out->q[i];
  }

  data->state->Fire_Detected = out->Fire_Detected;
  data->state->Payload_Dropped = out->Payload_Dropped;
  data->state->Fire_Location[0] = out->Fire_Location[0];
  data->state->Fire_Location[1] = out->Fire_Location[1];
  
  data->state->phi = out->phi;
  data->state->theta = out->theta;
  data->state->psi = out->psi;
  data->state->Dt = out->Dt;


  data->state->Desired_Location[0] = cntrl->posDes[0];
  data->state->Desired_Location[1] = cntrl->posDes[1];
  data->state->Desired_Location[2] = cntrl->posDes[2];
  data->state->Waypoint_Number = cntrl->Waypoint_Number;
  data->state->Number_of_Waypoints = cntrl->Number_of_Waypoints;
  
  data->state->messageID = DATALINK_MESSAGE_DRONE_STATE;
  //Serial.println(sizeof ( struct datalinkMessage_My_navout_ref));
	datalinkCheckSumEncode ( ( unsigned char* ) data->state, sizeof ( struct datalinkMessage_My_navout_ref) );

  //wf->beginPacket(IPAddress(192,168,8,142), 10000); //448
  wf->beginPacket(IPAddress(192,168,1,2), 10000); //001
  wf->write(( char* ) data->state, sizeof ( struct datalinkMessage_My_navout_ref));
  wf->endPacket();
}



void write_Sensor_Data( WiFiUDP* wf,Sensors sens )
{
  struct obDatalink_ref* data = &obDatalink;
  
  
  data->Sensor_Data->messageID = DATALINK_MESSAGE_SENSOR_DATA;

  for (int i = 0; i < 3; i++) {
    data->Sensor_Data->IMU_Gyro_Data[i] = sens.data_POZYX.gyr_rad[i];
    data->Sensor_Data->IMU_Accel_Data[i] = sens.data_POZYX.acc[i];
    data->Sensor_Data->IMU_Angles[i] = sens.data_POZYX.euler[i];
    data->Sensor_Data->Mo_Cap_Pos_Data[i] = sens.data_MoCap.Pos[i];
    data->Sensor_Data->Mo_Cap_Euler_Angles[i] = sens.data_MoCap.Euler_Angles[i];
  }

  for (int i = 0; i < 4; i++) {
    data->Sensor_Data->Mo_Cap_Orientation_Data[i] = sens.data_MoCap.Quat[i];
  }

  data->Sensor_Data->IMU_Count = sens.data_POZYX.POZYX_Update_Counter;
  data->Sensor_Data->Mo_Cap_Update = sens.data_MoCap.MoCap_Update_Counter;
  

	datalinkCheckSumEncode ( ( unsigned char* ) data->Sensor_Data, sizeof ( struct datalinkMessage_Sensor_Data) );

  //wf->beginPacket(IPAddress(192,168,8,142), 10000); //448
  wf->beginPacket(IPAddress(192,168,1,2), 10000); //001
  wf->write(( char* ) data->Sensor_Data, sizeof ( struct datalinkMessage_Sensor_Data));
  wf->endPacket();
}



void writeM0( WiFiUDP* wf )
{
  struct My_Kalman_ref* EKF = &MyEKF;
  struct My_navout_ref* out = EKF->out;
  struct obDatalink_ref* data = &obDatalink;
  data->m0->messageID = DATALINK_MESSAGE0;
  data->m0->pos[0] = out->p_b_e_L[0];
  data->m0->pos[1] = -out->p_b_e_L[1];
  data->m0->pos[2] = -out->p_b_e_L[2];

  for (int i = 0; i < 4; i++) {
    data->m0->q[i] = out->q[i];
  }


  datalinkCheckSumEncode ( ( unsigned char* ) data->m0, sizeof ( struct datalinkMessage0_ref) );

  //wf->beginPacket(IPAddress(192,168,8,142), 10000); //448
  wf->beginPacket(IPAddress(192,168,1,2), 10000); //001
  wf->write(( char* ) data->m0, sizeof ( struct datalinkMessage0_ref));
  wf->endPacket();
  

}