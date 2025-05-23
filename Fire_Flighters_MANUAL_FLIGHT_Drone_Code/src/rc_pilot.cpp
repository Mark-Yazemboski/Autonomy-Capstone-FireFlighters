#include "api/Common.h"
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
 * Dr. Thanakorn Khamvilai Email: thanakorn.khamvilai@ttu.edu
 * Dr. Vitor Valente       Email: vitor.valente@psu.edu
 *
 * EndCopyright
 ***/

 #include "rc_pilot.h"

 //////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                           Interrupt Functions                            //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#define RX_PIN_CHECK(chan)                             \
  if (mask & PCInt_RX_Pins[chan]) {                    \
    if (!(port & PCInt_RX_Pins[chan])) {               \
      dTime = cTime - edgeTime[chan];                  \
      if (MIN_PWM_IN < dTime && dTime < MAX_PWM_IN) {  \
        rcValue[chan] = dTime;                         \
      }                                                \
    }                                                  \
    else edgeTime[chan] = cTime;                       \
  }                                                  
  
void readRawRC(void *io_port){
  noInterrupts(); //disable other interrupts at this point
  uint16_t mask, port, cTime, dTime;
  bsp_io_port_t bsp_io_port = *(bsp_io_port_t*) io_port;
  static uint16_t PCintLast[3];
  
  R_IOPORT_PortRead(NULL, bsp_io_port, &port); //read port
   
  //write PPM to each PIN
  if(bsp_io_port == BSP_IO_PORT_00)
  {
    mask = port ^ PCintLast[0]; //indicate which bit change 
    cTime = micros(); //keep only 16 bits
    // interrupts(); //re-enable other interrupts at this point, the rest of this interrupt is not so time critical and can be interrupted safely
    PCintLast[0] = port; //memorize the state of all pins in the port
    RX_PIN_CHECK(ROLL);
    RX_PIN_CHECK(PITCH);
    RX_PIN_CHECK(THR);
  }
  else if(bsp_io_port == BSP_IO_PORT_01)
  {
    mask = port ^ PCintLast[1]; //indicate which bit change 
    cTime = micros(); //keep only 16 bits
    // interrupts(); //re-enable other interrupts at this point, the rest of this interrupt is not so time critical and can be interrupted safely
    PCintLast[1] = port; //memorize the state of all pins in the port
    RX_PIN_CHECK(YAW);
  }
  else if(bsp_io_port == BSP_IO_PORT_03)
  {
    mask = port ^ PCintLast[2]; //indicate which bit change 
    cTime = micros(); //keep only 16 bits
    // interrupts(); //re-enable other interrupts at this point, the rest of this interrupt is not so time critical and can be interrupted safely
    PCintLast[2] = port; //memorize the state of all pins in the port
    RX_PIN_CHECK(AUX);
    RX_PIN_CHECK(AUX2);
  }
  interrupts();
}

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                           Class Member Functions                         //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

float manualMap2(float x, float in_min, float in_mid, float in_max, float out_min, float out_mid, float out_max) {
  if (x < in_mid) {
      return (x - in_min) * (out_mid - out_min) / (in_mid - in_min) + out_min;
  } else {
      return (x - in_mid) * (out_max - out_mid) / (in_max - in_mid) + out_mid;
  }
}

RC_PILOT::RC_PILOT()
{
  for(uint8_t chan = 0; chan < RC_CHANS; chan++) {
    this->rcData[chan] = MIN_PWM_IN;
    this->rcData_previous[chan] = MIN_PWM_IN;
  }

  this->rc_in.ROLL  = MIN_PWM_IN;
  this->rc_in.PITCH = MIN_PWM_IN;
  this->rc_in.THR   = MIN_PWM_IN;
  this->rc_in.YAW   = MIN_PWM_IN;
  this->rc_in.AUX   = MIN_PWM_IN;
  this->rc_in.AUX2  = MIN_PWM_IN;

  // replace this with rc calibration data
  this->rc_in.ROLL_MIN  = 999;
  this->rc_in.PITCH_MIN = 1000;
  this->rc_in.THR_MIN   = 1000;
  this->rc_in.YAW_MIN   = 987;
  this->rc_in.AUX_MIN   = 1000;
  this->rc_in.AUX2_MIN  = 1000;

  

  this->rc_in.ROLL_MAX  = 1991;
  this->rc_in.PITCH_MAX = 1991;
  this->rc_in.THR_MAX   = 1984;
  this->rc_in.YAW_MAX   = 1972;
  this->rc_in.AUX_MAX   = 2000;
  this->rc_in.AUX2_MAX  = 2000;

  this->rc_in.ROLL_MID  = 1498;
  this->rc_in.PITCH_MID = 1498;
  this->rc_in.THR_MID   = (this->rc_in.THR_MIN + this->rc_in.THR_MAX)/2;
  this->rc_in.YAW_MID   = 1493;
  this->rc_in.AUX_MID   = 1500;
  this->rc_in.AUX2_MID  = 1500;
}

RC_PILOT::~RC_PILOT()
{

}

void RC_PILOT::init()
{
  for(uint8_t i = 0; i < RC_CHANS; i++){
    attachInterruptParam(digitalPinToInterrupt(RX_Pins[i]), readRawRC, CHANGE, &this->io_ports[i]);
  } 
  this->rc_in.Prev_ROLL = 0;
  this->rc_in.Prev_PITCH = 0;
  this->rc_in.Prev_THR = 0;
  this->rc_in.Prev_YAW = 0;
  this->rc_in.Prev_AUX = 0;
  this->rc_in.Prev_AUX2 = 0;

  this->rc_in.Good_Reciever = true;
  this->rc_in.Good_Kill_Wire = true;
  for(uint8_t i = 0; i < 4; i++){
    this->thrustavg[i] = 1000;
  }
  this->avgi = 0;
  this->avgg = 1000;
}

void RC_PILOT::update()
{ 
  for(uint8_t chan = 0; chan < RC_CHANS; chan++) {
    this->rcData[chan] = constrain(rcValue[chan], MIN_PWM_IN, MAX_PWM_IN);

    // previous jump errors, could use a filtering
    //this->rcData[chan] = abs(this->rcData[chan] - this->rcData_previous[chan]) > PWM_JUMP_LIMIT ? this->rcData_previous[chan] : this->rcData[chan];
    //this->rcData_previous[chan] = this->rcData[chan];
  }




  
  float Filtered_Roll =this->rcData[ROLL];
  float Filtered_Pitch =this->rcData[PITCH];
  float Filtered_Yaw =this->rcData[YAW];
  float Filtered_Thr = this->rcData[THR];


  // avg filter
  this->avgg= (this->thrustavg[0]+this->thrustavg[1]+this->thrustavg[2]+this->thrustavg[3])/4;

  this->thrustavg[this->avgi] = this->rcData[THR];
  this->avgi++;


  if(this->avgi>3){
    this->avgi=0;
  }

  Filtered_Thr =((this->avgg)*5+(this->rcData[THR]))/6;

  // if(this->rcData[THR] < (100+this->avgg) && this->rcData[THR] > (this->avgg-100)){
  //   Filtered_Thr =((this->avgg)*1+(this->rcData[THR]))/2;
  // }
  // else{
  //   Filtered_Thr =((this->avgg)*5+(this->rcData[THR]))/6;
  // }









  // this->rc_in.ROLL  = this->rcData[THR];
  // this->rc_in.PITCH =  this->avgg;
  // this->rc_in.THR   = Filtered_Thr;
  this->rc_in.ROLL   = manualMap2(Filtered_Roll, this->rc_in.ROLL_MIN, this->rc_in.ROLL_MID, this->rc_in.ROLL_MAX, MIN_PWM_IN,(MAX_PWM_IN+MIN_PWM_IN)/2 ,MAX_PWM_IN);
  this->rc_in.PITCH   = manualMap2(Filtered_Pitch, this->rc_in.PITCH_MIN, this->rc_in.PITCH_MID, this->rc_in.PITCH_MAX, MIN_PWM_IN,(MAX_PWM_IN+MIN_PWM_IN)/2 ,MAX_PWM_IN);
  this->rc_in.YAW   = manualMap2(Filtered_Yaw, this->rc_in.YAW_MIN, this->rc_in.YAW_MID, this->rc_in.YAW_MAX, MIN_PWM_IN,(MAX_PWM_IN+MIN_PWM_IN)/2 ,MAX_PWM_IN);
  this->rc_in.THR   = manualMap2(Filtered_Thr, this->rc_in.THR_MIN, this->rc_in.THR_MID, this->rc_in.THR_MAX, MIN_PWM_IN,(MAX_PWM_IN+MIN_PWM_IN)/2 ,MAX_PWM_IN);
  //this->rc_in.THR = this->rcData[THR];
  this->rc_in.AUX   = this->rcData[AUX];
  this->rc_in.AUX2  = this->rcData[AUX2];


  //This logic will detect if the reciever has been unplugged during the flight. It will see if the 
  //Reciever has sent the same exact message for the past 20 updates. The reciever normally has some noise,
  //And for it to send the same message 20 times in a row is basically impossible, and the drone should be terminated.

  if (this->rc_in.Prev_AUX == this->rcData[AUX]){
    this->rc_in.Same_Count_Kill_Switch += 1;
  }else{
    this->rc_in.Same_Count_Kill_Switch = 0;
  }

  if (this->rc_in.Same_Count_Kill_Switch >= 100){
    this->rc_in.Good_Kill_Wire = false;
  }

  if (this->rc_in.Prev_ROLL == this->rcData[ROLL] &&
    this->rc_in.Prev_PITCH == this->rcData[PITCH] &&
    this->rc_in.Prev_THR == this->rcData[THR] &&
    this->rc_in.Prev_YAW == this->rcData[YAW] &&
    this->rc_in.Prev_AUX == this->rcData[AUX] &&
    this->rc_in.Prev_AUX2 == this->rcData[AUX2])
  {
    this->rc_in.Same_Count += 1;
  }else{
    this->rc_in.Same_Count = 0;
    this->rc_in.Prev_ROLL = this->rcData[ROLL];
    this->rc_in.Prev_PITCH = this->rcData[PITCH];
    this->rc_in.Prev_THR = this->rcData[THR];
    this->rc_in.Prev_YAW = this->rcData[YAW];
    this->rc_in.Prev_AUX = this->rcData[AUX];
    this->rc_in.Prev_AUX2 = this->rcData[AUX2];
  }
  
  if (this->rc_in.Same_Count >= 100){
    this->rc_in.Good_Reciever = false;
  }
  
  

  

  
}

void RC_PILOT::print()
{
  Serial.print("RAW RC: "); 
  Serial.print(  this->rcData[ROLL]);  Serial.print(", "); 
  Serial.print(  this->rcData[PITCH]); Serial.print(", "); 
  Serial.print(  this->rcData[THR]);   Serial.print(", "); 
  Serial.print(  this->rcData[YAW]);   Serial.println(", ");
  Serial.print("RC: "); 
  Serial.print(  this->rc_in.ROLL);  Serial.print(", "); 
  Serial.print(  this->rc_in.PITCH); Serial.print(", "); 
  Serial.print(  this->rc_in.THR);   Serial.print(", "); 
  Serial.print(  this->rc_in.YAW);   Serial.print(", ");
  Serial.print(  this->rc_in.AUX);   Serial.print(", "); 
  Serial.println(this->rc_in.AUX2);
}
