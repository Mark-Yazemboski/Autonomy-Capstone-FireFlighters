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

struct datalinkMessageAutopilotDels_ref obDatalinkMessageAutopilotDels = {
  0xa3 , /* uchar sync1 */
  0xb2 , /* uchar sync2 */
  0xc1 , /* uchar sync3 */
  0 ,    /* uchar spare */
  0 ,    /* int messageID */
  0 ,    /* int messageSize */
  0 ,    /* uint hcsum */
  0 ,    /* uint csum */
  0 ,    /* float time */
  {0, 0, 0} , /* float c_delm[3] */
  {0} , /* float c_delf[1] */
  {0} , /* float c_delt[1] */
  {0, 0, 0, 0}, /* uint16_t PWM[4] */
  0 , /* float Roll */
  0 , /* float Pitch */
  0 , /* float Yaw */
  0 , /* float Throttle */
  0 , /* float Kill */
  0   /* float Auto */
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
  0, /* double time */
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
  0 , /* int IMU_Count */
  {0}, /* float Mo_Cap_Pos_Data[3] */
  {0}, /* float Mo_Cap_Orientation_Data[4] */
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
  &obDatalinkMessageAutopilotDels, /* dir autopilotDels */
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
  obDatalink.autopilotDels = &obDatalinkMessageAutopilotDels;
  obDatalink.truthSim = &obDatalinkMessageTruth;
  obDatalink.sim2onboard = &obDatalinkMessageSim2Onboard;
  obDatalink.onboard2sim = &obDatalinkMessageOnboard2Sim;
  obDatalink.optitrack = &obDatalinkMessageOptitrack;
  obDatalink.state = &gcs0DatalinkMessage_Drone_State;

  set_interval( DATALINK_INTERVALUP0, DATALINK_MESSAGE_UP0 );
  set_interval( DATALINK_INTERVALPWM, DATALINK_MESSAGE_PWM );
  set_interval( DATALINK_INTERVALAUTOPILOTDELS, DATALINK_MESSAGE_AUTOPILOTDELS );
  set_interval( DATALINK_INTERVALTRUTH, DATALINK_MESSAGE_TRUTH );
  set_interval( DATALINK_INTERVALHITL_SIM2ONBOARD, DATALINK_MESSAGE_HITL_SIM2ONBOARD );
  set_interval( DATALINK_INTERVALHITL_ONBOARD2SIM, DATALINK_MESSAGE_HITL_ONBOARD2SIM );
  set_interval( DATALINK_INTERVALOPTITRACK, DATALINK_MESSAGE_OPTITRACK );
  set_interval( DATALINK_INTERVAL_DRONE_STATE, DATALINK_MESSAGE_DRONE_STATE );
}

void Dlink::set_interval( long intervalX, int type ){
  switch ( type ){
    case DATALINK_MESSAGE_UP0:
      obDatalink.up0->messageSize = sizeof( struct datalinkMessageUp0_ref );
      obDatalink.up0->messageID = DATALINK_MESSAGE_UP0;
      this->intervalUp0 = intervalX;
      this->previousMillisUp0 = 0;
      break;
    case DATALINK_MESSAGE_PWM:
      obDatalink.pwm->messageSize = sizeof( struct datalinkMessagePWM_ref );
      obDatalink.pwm->messageID = DATALINK_MESSAGE_PWM;
      this->intervalPWM = intervalX;
      this->previousMillisPWM = 0;
      break;
    case DATALINK_MESSAGE_AUTOPILOTDELS:
      obDatalink.autopilotDels->messageSize = sizeof( struct datalinkMessageAutopilotDels_ref );
      obDatalink.autopilotDels->messageID = DATALINK_MESSAGE_AUTOPILOTDELS;
      this->intervalAutopilotDels = intervalX;
      this->previousMillisAutopilotDels = 0;
      break;
    case DATALINK_MESSAGE_TRUTH:
      obDatalink.truthSim->messageSize = sizeof( struct datalinkMessageTruth_ref );
      obDatalink.truthSim->messageID = DATALINK_MESSAGE_TRUTH;
      this->intervalTruthSim = intervalX;
      this->previousMillisTruthSim = 0;
      break;
    case DATALINK_MESSAGE_HITL_SIM2ONBOARD:
      obDatalink.sim2onboard->messageSize = sizeof( struct datalinkMessageHITLSim2Onboard_ref );
      obDatalink.sim2onboard->messageID = DATALINK_MESSAGE_HITL_SIM2ONBOARD;
      this->intervalSim2Onboard = intervalX;
      this->previousMillisSim2Onboard = 0;
      break;
    case DATALINK_MESSAGE_HITL_ONBOARD2SIM:
      obDatalink.onboard2sim->messageSize = sizeof( struct datalinkMessageHITLOnboard2Sim_ref );
      obDatalink.onboard2sim->messageID = DATALINK_MESSAGE_HITL_ONBOARD2SIM;
      this->intervalOnboard2Sim = intervalX;
      this->previousMillisOnboard2Sim = 0;
      break;
    case DATALINK_MESSAGE_OPTITRACK:
      obDatalink.optitrack->messageSize = sizeof( struct datalinkMessageOptitrack_ref );
      obDatalink.optitrack->messageID = DATALINK_MESSAGE_OPTITRACK;
      this->intervalOptitrack = intervalX;
      this->previousMillisOptitrack = 0;
      break;
    case DATALINK_MESSAGE_DRONE_STATE:
      obDatalink.state->messageSize = sizeof( struct datalinkMessage_My_navout_ref );
      obDatalink.state->messageID = DATALINK_MESSAGE_DRONE_STATE;
      this->intervalDrone_State = intervalX;
      this->previousMillisDrone_State = 0;
      break;
    case DATALINK_MESSAGE_SENSOR_DATA:
      obDatalink.state->messageSize = sizeof( struct datalinkMessage_Sensor_Data );
      obDatalink.state->messageID = DATALINK_MESSAGE_SENSOR_DATA;
      this->intervalSensor_Data = intervalX;
      this->previousMillisSensor_Data = 0;
      break;

    case DATALINK_MESSAGE_PID:
      obDatalink.PID_Tune->messageSize = sizeof( struct datalinkMessage_PID );
      obDatalink.PID_Tune->messageID = DATALINK_MESSAGE_PID;
      this->intervalPID = intervalX;
      this->previousMillisSensor_Data = 0;
      break;
    default:
      Serial.println("Error: Invalid type for datalink interval");
      break;
  }
}

void Dlink::recv_update(Sensors* sens_pointer){
  // messages to receive from GCS: TruthSim, Sim2Onboard, Optitrack
  // read datalink optitrack
  readDatalink( &ether.UdpGPS,sens_pointer );
  // read datalink gcs
  readDatalink( &ether.UdpGCS,sens_pointer );
}

void Dlink::send_update(Sensors sens,double c_delf, double c_delm_0, double c_delm_1, double c_delm_2,float Throttle, float Roll, float Pitch,float Yaw,float Manual,float Kill,float PWM_0,float PWM_1,float PWM_2,float PWM_3){
  // messages to send to GCS: AutopiloDels, Up0, PWM, Onboard2Sim
  // Serial.println("Sending_Data");
  unsigned long currentMillis = millis();
  if (currentMillis - this->previousMillisAutopilotDels >= this->intervalAutopilotDels) {
    this->previousMillisAutopilotDels = currentMillis;
    writeAutopilotDels( &ether.UdpGCS, c_delf,  c_delm_0,  c_delm_1,  c_delm_2, Throttle,  Roll,  Pitch, Yaw, Manual, Kill,PWM_0,PWM_1,PWM_2,PWM_3);
  }
  currentMillis = millis();
  if (currentMillis - this->previousMillisDrone_State >= this->intervalDrone_State) {
    this->previousMillisDrone_State = currentMillis;
    write_Drone_State( &ether.UdpGCS );
  }

  currentMillis = millis();
  if (currentMillis - this->previousMillisSensor_Data >= this->intervalSensor_Data) {
    this->previousMillisSensor_Data = currentMillis;
    write_Sensor_Data( &ether.UdpGCS, sens);
  }

  
  // currentMillis = millis();
  // if (currentMillis - this->previousMillisUp0 >= this->intervalUp0) {
  //   this->previousMillisUp0 = currentMillis;
  //   writeUP0( &ether.UdpGCS, &rc );
  // }

  // currentMillis = millis();
  // if (currentMillis - this->previousMillisPWM >= this->intervalPWM) {
  //   this->previousMillisPWM = currentMillis;
  //   writePWM( &ether.UdpGCS );
  // }
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
                  //Serial.println("Good data and recieving");
                  sens_pointer->data_MoCap.Pos[0] = data->optitrack->pos_x;
                  sens_pointer->data_MoCap.Pos[1] = data->optitrack->pos_y;
                  sens_pointer->data_MoCap.Pos[2] = data->optitrack->pos_z;
                      
                  sens_pointer->data_MoCap.Quat[0] = data->optitrack->qw;
                  sens_pointer->data_MoCap.Quat[1] = data->optitrack->qx;
                  sens_pointer->data_MoCap.Quat[2] = data->optitrack->qy;
                  sens_pointer->data_MoCap.Quat[3] = data->optitrack->qz;
                  if (data->optitrack->frameNum != sens_pointer->data_MoCap.MoCap_Frame_Counter){
                    sens_pointer->data_MoCap.MoCap_Frame_Counter = data->optitrack->frameNum;
                    sens_pointer->data_MoCap.MoCap_Update_Counter ++;
                  }
                }
                //Serial.println("Valid");
                //Serial.println(data->optitrack->valid);
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
                     cntrl->KP[i] = data->PID_Tune->KP[i];
                     cntrl->KD[i] = data->PID_Tune->KD[i];
                     cntrl->KI[i] = data->PID_Tune->KI[i];

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

void writeM0( WiFiUDP* wf )
{

}

void writeM1( WiFiUDP* wf )
{

}

void writeUP0( WiFiUDP* wf, RC_PILOT* rc )
{
  struct obDatalink_ref* data = &obDatalink;

  data->up0->throttleLever = rc->rc_in.THR;
  data->up0->rollStick = rc->rc_in.ROLL;
  data->up0->pitchStick = rc->rc_in.PITCH;
  data->up0->rudderPedal = rc->rc_in.YAW;

  data->up0->button[0] = rc->rc_in.AUX;
  data->up0->button[1] = rc->rc_in.AUX2;

  data->up0->messageID = DATALINK_MESSAGE_UP0;
  datalinkCheckSumEncode ( ( unsigned char* ) data->up0, sizeof ( struct datalinkMessageUp0_ref) );

  wf->beginPacket(wf->remoteIP(), wf->remotePort());
  wf->write(( char* ) data->up0, sizeof ( struct datalinkMessageUp0_ref));
  wf->endPacket();
}

void writePWM( WiFiUDP* wf ){
  struct obDatalink_ref* data = &obDatalink;

  data->pwm->raw_pwm[0] = 0; /* M1 */
  data->pwm->raw_pwm[1] = 0; /* M2 */
  data->pwm->raw_pwm[2] = 0; /* M3 */
  data->pwm->raw_pwm[3] = 0; /* M4 */

  data->pwm->messageID = DATALINK_MESSAGE_PWM;
  datalinkCheckSumEncode ( ( unsigned char* ) data->pwm, sizeof ( struct datalinkMessagePWM_ref) );

  wf->beginPacket(wf->remoteIP(), wf->remotePort());
  wf->write(( char* ) data->pwm, sizeof ( struct datalinkMessagePWM_ref));
  wf->endPacket();
}

void writeAutopilotDels( WiFiUDP* wf ,double c_delf, double c_delm_0, double c_delm_1, double c_delm_2,float Throttle, float Roll, float Pitch,float Yaw,float Auto,float Kill,float PWM_0,float PWM_1,float PWM_2,float PWM_3)
{
  struct obDatalink_ref* data = &obDatalink;

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




  data->autopilotDels->messageID = DATALINK_MESSAGE_AUTOPILOTDELS;
	datalinkCheckSumEncode ( ( unsigned char* ) data->autopilotDels, sizeof ( struct datalinkMessageAutopilotDels_ref) );

  //wf->beginPacket(IPAddress(192,168,8,142), 10000); //448
  wf->beginPacket(IPAddress(192,168,1,12), 10000); //001
  wf->write(( char* ) data->autopilotDels, sizeof ( struct datalinkMessageAutopilotDels_ref));
  wf->endPacket();
}

void write_Drone_State( WiFiUDP* wf )
{
  struct obDatalink_ref* data = &obDatalink;
  struct My_Kalman_ref* EKF = &MyEKF;
  struct My_navout_ref* out = EKF->out;
  
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
  
  data->state->phi = out->phi;
  data->state->theta = out->theta;
  data->state->psi = out->psi;
  data->state->time = out->time;
  
  data->state->messageID = DATALINK_MESSAGE_DRONE_STATE;
  //Serial.println(sizeof ( struct datalinkMessage_My_navout_ref));
	datalinkCheckSumEncode ( ( unsigned char* ) data->state, sizeof ( struct datalinkMessage_My_navout_ref) );

  //wf->beginPacket(IPAddress(192,168,8,142), 10000); //448
  wf->beginPacket(IPAddress(192,168,1,12), 10000); //001
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
    data->Sensor_Data->Mo_Cap_Pos_Data[i] = sens.data_MoCap.Pos[i];
  }

  for (int i = 0; i < 4; i++) {
    data->Sensor_Data->Mo_Cap_Orientation_Data[i] = sens.data_MoCap.Quat[i];
  }

  data->Sensor_Data->IMU_Count = sens.data_POZYX.POZYX_Update_Counter;
  data->Sensor_Data->Mo_Cap_Update = sens.data_MoCap.MoCap_Update_Counter;
  

	datalinkCheckSumEncode ( ( unsigned char* ) data->Sensor_Data, sizeof ( struct datalinkMessage_Sensor_Data) );

  //wf->beginPacket(IPAddress(192,168,8,142), 10000); //448
  wf->beginPacket(IPAddress(192,168,1,12), 10000); //001
  wf->write(( char* ) data->Sensor_Data, sizeof ( struct datalinkMessage_Sensor_Data));
  wf->endPacket();
}