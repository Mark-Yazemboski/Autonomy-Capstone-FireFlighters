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

#include "../include/sensors.h"
#include "../include/Pozyx.h"
#include "../include/Pozyx_definitions.h"
#include <Wire.h>

#define POZYX_GYR_SCALE 0.0625
#define POZYX_MAG_SCALE 0.0625
#define POZYX_EULER_SCALE 0.0625
#define POZYX_QUAT_SCALE 1.0/16384.0

float wrapAngleDeg(float angle) {
  angle = fmod(angle + 180.0, 360.0);
  if (angle < 0)
      angle += 360.0;
  return angle - 180.0;
}

Sensors::Sensors()
{
  for(uint8_t i = 0; i < 3; i++)
  {
    this->data_POZYX.gyr[i]   = 0;
    this->data_POZYX.acc[i]   = 0;
    this->data_POZYX.mag[i]   = 0;
    this->data_POZYX.euler[i] = 0;
    this->data_POZYX.quat[i]  = 0;

    this->bias.gyr[i]   = 0;
    this->bias.acc[i]   = 0;
    this->bias.mag[i]   = 0;
    this->bias.euler[i] = 0;
    this->bias.quat[i]  = 0;
  }
  this->data_POZYX.quat[0] = 1;
  this->calibration_flag = 0;
}

Sensors::~Sensors() {}

void Sensors::init()
{
  Serial.println("Init");
  if(Pozyx.begin(true, MODE_POLLING, POZYX_INT_MASK_IMU, 0x02) == POZYX_FAILURE)
  {
    Serial.println("ERROR: Unable to connect to POZYX shield");
    Serial.println("Reset required");
    delay(100);
    // abort();
  }
  if (NUM_CALIBRATION != 0){
    for(unsigned long i = 0; i < NUM_CALIBRATION; i++)
    {
      this->update();
      for(uint8_t j = 0; j < 3; j++)
      {
        this->bias.gyr[j]   += this->data_POZYX.gyr[j];
        this->bias.euler[j] += this->data_POZYX.euler[j];
      }
    }
    for(uint8_t i = 0; i < 3; i++)
    {
      this->bias.gyr[i]   /= NUM_CALIBRATION;
      this->bias.euler[i] /= NUM_CALIBRATION;

      this->data_POZYX.gyr[i]   = 0;
      this->data_POZYX.euler[i] = 0;
    }
}
  this->calibration_flag = 1;
}

void Sensors::update()
{
  sensor_raw_t sensor_raw;
  if (Pozyx.waitForFlag(POZYX_INT_STATUS_IMU, 1) == POZYX_SUCCESS){
    Pozyx.getRawSensorData(&sensor_raw);
  }else{
    uint8_t interrupt_status = 0;
    Pozyx.getInterruptStatus(&interrupt_status);
    return;
  }

  // Accelerometer Data needs to be added: CONVERTS from mg to g also has Z down X forward Y to complete set
  this->data_POZYX.acc[0] = sensor_raw.acceleration[1]/1000.0;
  this->data_POZYX.acc[1] = -sensor_raw.acceleration[0]/1000.0;
  this->data_POZYX.acc[2] = sensor_raw.acceleration[2]/1000.0;

  //Converts from g to ft/s^2
  this->data_POZYX.acc[0] = this->data_POZYX.acc[0]*32.1740;
  this->data_POZYX.acc[1] = this->data_POZYX.acc[1]*32.1740;
  this->data_POZYX.acc[2] = this->data_POZYX.acc[2]*32.1740;

  // YPR to RPY and NED
  this->data_POZYX.euler[0] = wrapAngleDeg(sensor_raw.euler_angles[1] * POZYX_EULER_SCALE); // convert to deg
  this->data_POZYX.euler[1] = wrapAngleDeg(sensor_raw.euler_angles[2] * POZYX_EULER_SCALE); // convert to deg
  this->data_POZYX.euler[2] = wrapAngleDeg(sensor_raw.euler_angles[0] * POZYX_EULER_SCALE); // convert to deg


  if(this->calibration_flag) // regular reading
  {
    float temp_gyr[3];
    temp_gyr[0] = sensor_raw.angular_vel[1] * POZYX_GYR_SCALE * -1.0; // convert to deg/s
    temp_gyr[1] = sensor_raw.angular_vel[0] * POZYX_GYR_SCALE * -1.0; // convert to deg/s
    temp_gyr[2] = sensor_raw.angular_vel[2] * POZYX_GYR_SCALE * -1.0; // convert to deg/s

    for(uint8_t i = 0; i < 3; i++)
    {
      this->data_POZYX.euler[i] -= this->bias.euler[i];
      temp_gyr[i]         -= this->bias.gyr[i];
      

    }

    // Serial.println(this->bias.euler[0]);
    // Serial.println(this->bias.euler[1]);
    // Serial.println(this->bias.euler[2]);
    // Serial.println("A");
    // Serial.println(this->bias.gyr[0]);
    // Serial.println(this->bias.gyr[1]);
    // Serial.println(this->bias.gyr[2]);

    // Serial.println("BB");

    for(uint8_t i = 0; i < 3; i++)
    {
      this->data_POZYX.gyr[i] = LOWPASS_WEIGHT*this->data_POZYX.gyr[i] + (1-LOWPASS_WEIGHT)*temp_gyr[i]; // low-pass filter
    }

  }
  else // calibration step
  {
    this->data_POZYX.gyr[0] = sensor_raw.angular_vel[1] * POZYX_GYR_SCALE * -1.0; // convert to deg/s
    this->data_POZYX.gyr[1] = sensor_raw.angular_vel[0] * POZYX_GYR_SCALE * -1.0; // convert to deg/s
    this->data_POZYX.gyr[2] = sensor_raw.angular_vel[2] * POZYX_GYR_SCALE * -1.0; // convert to deg/s
    
  }

  // THIS WILL CORRECT FOR THE WEIRD ASS ORIENTATION OF THE POZYX

  this->data_POZYX.gyr[0] = -this->data_POZYX.gyr[0];
  this->data_POZYX.euler[0] = -this->data_POZYX.euler[0];
  this->data_POZYX.gyr[2] = -this->data_POZYX.gyr[2];
  this->data_POZYX.euler[2] = -this->data_POZYX.euler[2];

  // std::swap(this->data_POZYX.acc[1], this->data_POZYX.acc[2]);


  // this->data_POZYX.acc[2] = -this->data_POZYX.acc[2];

  for(uint8_t i = 0; i < 3; i++)
  {
    this->data_POZYX.gyr_rad[i] = this->data_POZYX.gyr[i]*PI/180.0;
    this->data_POZYX.euler_rad[i] = this->data_POZYX.euler[i]*PI/180.0;
  }
  this->data_POZYX.POZYX_Update_Counter ++;
}

void Sensors::print()
 {
  
   Serial.print(this->data_POZYX.gyr[0]);// roll rate
   Serial.print(",");
   Serial.print(this->data_POZYX.gyr[1]);// pitch rate
   Serial.print(",");
   Serial.print(this->data_POZYX.gyr[2]);// yaw rate
  
   Serial.print("     ");
  
   Serial.print(this->data_POZYX.acc[0]); // x acc 
   Serial.print(",");
   Serial.print(this->data_POZYX.acc[1]); // y acc
   Serial.print(",");
   Serial.print(this->data_POZYX.acc[2]); // z acc
   Serial.print(",");
  
 
   Serial.print("     ");


   Serial.print(this->data_POZYX.euler[0]); //roll angle
   Serial.print(",");
   Serial.print(this->data_POZYX.euler[1]); //pitch angle
   Serial.print(",");
   Serial.print(this->data_POZYX.euler[2]); //yaw angle
   Serial.println();
 }