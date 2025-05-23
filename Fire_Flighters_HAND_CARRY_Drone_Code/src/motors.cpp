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

#include "../include/motors.h"

Motors::Motors()
{

}

Motors::~Motors()
{

}

void Motors::init()
{
  for(uint8_t i=0; i<NUM_MOTORS; i++)
    motor[i].begin(20000ul, TIMER_SOURCE_DIV_1); // 20000 us is 50 Hz

  this->stop();

  // wait to make sure everything is set
  delay(100); // it's okay to use delay in setup()

  Servo.begin(20000ul, TIMER_SOURCE_DIV_1);
}

void Motors::calibrate()
{
  // ESC calibration: send max and min PWM to the ESC
  // Calibration must be done only once when you have a new ESC

  for(uint8_t i=0; i<NUM_MOTORS; i++) motor[i].pulseWidth_us(MAX_PWM_OUT);
  delay(5000);
  for(uint8_t i=0; i<NUM_MOTORS; i++) motor[i].pulseWidth_us(MIN_PWM_OUT);
  delay(5000);
}

void Motors::Servo_Closed(){
  Servo.pulseWidth_us(SERVO_CLOSED);
}

void Motors::Servo_Open(){
  Servo.pulseWidth_us(SERVO_OPEN);
}

void Motors::stop()
{
  for(uint8_t i=0; i<NUM_MOTORS; i++) motor[i].pulseWidth_us(MIN_PWM_OUT);
}

void Motors::update(float pwm[NUM_MOTORS])
{
  for(uint8_t i=0; i<NUM_MOTORS; i++)
  {
    pwm[i] = constrain(pwm[i], MIN_PWM_OUT, MAX_PWM_OUT);
    motor[i].pulseWidth_us(pwm[i]);
  }
}
