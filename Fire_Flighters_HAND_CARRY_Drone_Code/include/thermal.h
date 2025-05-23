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
 * Dr. Vitor T. Valente (vitor.valente@psu.edu)
 *
 * EndCopyright
 ***/

#ifndef AERSP_THERMAL_H
#define AERSP_THERMAL_H

#include <Wire.h>
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

#define TA_SHIFT 8 //Default shift for MLX90640 in open air

class Thermal
{
public:
  Thermal();
  ~Thermal();

  void init();
  void update();
  void print();
  bool is_connected();
  bool Find_Fire(float Drone_Height,float Phi_FOV,float Theta_FOV, float* Fire_Position);

private:
};

float scaleRawToCelsius(uint16_t rawVal);

extern uint16_t mlx90640Frame[834];

#endif