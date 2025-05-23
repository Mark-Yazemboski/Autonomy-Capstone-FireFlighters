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

#ifndef AERSP_DATALINK_H
#define AERSP_DATALINK_H

#include <Arduino.h>
#include <Wire.h>
#include "../include/wifi.h"
#include "../include/rc_pilot.h"
#include "../include/datalink.h"
#include "../include/sensors.h"
#include "../include/motors.h"
#include "../include/utils.h"
//#include "../include/thermal.h"
#include "controller_ref.h"
#include "../include/controller.h"
#include "../include/navigation.h"
#include "myekf_ref.h"
Motors motors;
Sensors sens;
RC_PILOT rc;
wifi ether;
Dlink datalink;
//Thermal thermal;

float pwm[4];

void updateMixer( const char button0, char* LaunchState,
    float c_delf, float c_delm0, float c_delm1, float c_delm2,
    float* pwm);




#endif