//#include "motors.h"
#include <PS2X_lib.h>
int motor_right_speed = 0;
int motor_left_speed = 0;
PS2X ps2x; // create PS2 Controller Class
int error = 0; 
byte type = 0;
byte vibrate = 0;
#define PS2_DAT 12 //MISO  19
#define PS2_CMD 13 //MOSI  23
#define PS2_SEL 15 //SS     5
#define PS2_CLK 14 //SLK   18
bool kt;
//#define MIN_PWM 0
//#define MAX_PWM 4095
//bool RUN = 0;



void setupPS2controller()
{
  ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, true, true);
  ps2x.readType();
  //  ps2x.read_gamepad(false, 0); // disable vibration of the controller
}

bool PS2control()
{
  // Based on IgorF2's Arduino Bot:https://www.instructables.com/Arduino-Robot-With-PS2-Controller-PlayStation-2-Jo/

  int mJoyX = ps2x.Analog(PSS_RX); // read x-joystick
  int mJoyY = ps2x.Analog(PSS_RY); // read y-joystick

  int nJoyY = ps2x.Analog(PSS_LX); // read x-joystick
  int nJoyX = ps2x.Analog(PSS_LY); // read y-joystic

  nJoyX = map(nJoyX, 0, 255, -1023, 1023);
  nJoyY = map(nJoyY, 0, 255, 1023, -1023);
  
  mJoyX = map(mJoyX, 0, 255, -1023, 1023);
  mJoyY = map(mJoyY, 0, 255, 1023, -1023);

  // OUTPUTS
  int nMotMixL; // Motor (left) mixed output
  int nMotMixR; // Motor (right) mixed output

  int mMotMixL; // Motor (left) mixed output-------
  int mMotMixR; // Motor (right) mixed output---------
  

  // CONFIG
  // - fPivYLimt  : The threshold at which the pivot action starts
  //                This threshold is measured in units on the Y-axis
  //                away from the X-axis (Y=0). A greater value will assign
  //                more of the joystick's range to pivot actions.
  //                Allowable range: (0..+127)
  float fPivYLimit = 1023.0;

  // TEMP VARIABLES
  float nMotPremixL; // Motor (left) premixed output
  float nMotPremixR; // Motor (right) premixed output
  float mMotPremixL; // Motor (left) premixed output--------------------
  float mMotPremixR; // Motor (right) premixed output------------------------------
  int nPivSpeed;     // Pivot Speed
  int mPivSpeed; 
  float fPivScale;   // Balance scale between drive and pivot

  // Calculate Drive Turn output due to Joystick X input
  

  if(kt==0)
  {
        Serial.println(ps2x.NewButtonState(PSB_GREEN));
  Serial.println(ps2x.NewButtonState(PSB_RED));
      Serial.println(ps2x.NewButtonState(PSB_BLUE));
  Serial.println(ps2x.NewButtonState(PSB_PINK));
    Serial.println(ps2x.NewButtonState(PSB_PAD_LEFT));
  Serial.println(ps2x.NewButtonState(PSB_PAD_UP));
  Serial.println(ps2x.NewButtonState(PSB_PAD_RIGHT));
  if (nJoyY >= 0)
  {
    // Forward
    nMotPremixL = (nJoyX >= 0) ? 1023.0 : (1023.0 + nJoyX);
    nMotPremixR = (nJoyX >= 0) ? (1023.0 - nJoyX) : 1023.0;
  }
  else
  {
    // Reverse
    nMotPremixL = (nJoyX >= 0) ? (1023.0 - nJoyX) : 1023.0;
    nMotPremixR = (nJoyX >= 0) ? 1023.0 : (1023.0 + nJoyX);
  }


  if (mJoyY >= 0)
  {
    // Forward
    mMotPremixL = (mJoyX >= 0) ? 1023.0 : (1023.0 + mJoyX);
    mMotPremixR = (mJoyX >= 0) ? (1023.0 - mJoyX) : 1023.0;
  }
  else
  {
    // Reverse
    mMotPremixL = (mJoyX >= 0) ? (1023.0 - mJoyX) : 1023.0;
    mMotPremixR = (mJoyX >= 0) ? 1023.0 : (1023.0 + mJoyX);
  }

  // Scale Drive output due to Joystick Y input (throttle)
  nMotPremixL = nMotPremixL * nJoyY / 1023.0;
  nMotPremixR = nMotPremixR * nJoyY / 1023.0;
  mMotPremixL = mMotPremixL * mJoyY / 1023.0;
  mMotPremixR = mMotPremixR * mJoyY / 1023.0;

  // Now calculate pivot amount
  // - Strength of pivot (nPivSpeed) based on Joystick X input
  // - Blending of pivot vs drive (fPivScale) based on Joystick Y input
  nPivSpeed = nJoyX;
  fPivScale = (abs(nJoyY) > fPivYLimit) ? 0.0 : (1.0 - abs(nJoyY) / fPivYLimit);

  mPivSpeed = mJoyX;
  fPivScale = (abs(mJoyY) > fPivYLimit) ? 0.0 : (1.0 - abs(mJoyY) / fPivYLimit);


  // Calculate final mix of Drive and Pivot
  nMotMixL = (1.0 - fPivScale) * nMotPremixL + fPivScale * (nPivSpeed);
  nMotMixR = (1.0 - fPivScale) * nMotPremixR + fPivScale * (-nPivSpeed);

  mMotMixL = ((1.0 - fPivScale) * mMotPremixL + fPivScale * (mPivSpeed));
  mMotMixR = ((1.0 - fPivScale) * mMotPremixR + fPivScale * (-mPivSpeed));
//  Serial.print("trai-phai:");
//  Serial.println(nMotMixL);
//  Serial.print("len-xuong:");
//  Serial.println(mMotMixL);
//  Serial.print("\t");
//  Serial.println(nMotMixR);

//-----------------------------------

//  Serial.println(ps2x.NewButtonState(PSB_PAD_UP));
//  Serial.println(ps2x.NewButtonState(PSB_PAD_DOWN));
//  Serial.println(ps2x.Button(PSB_R1));
//  Serial.println(ps2x.Button(PSB_R2));
//  Serial.println(ps2x.Button(PSB_L1));
//  Serial.println(ps2x.Button(PSB_L2));

//-----------------------------------
  
  if(nMotMixL>50)
  {
       pwm.setPWM(PWM_CHANNEL6, 0, 4095);
    }
    else if(nMotMixL<-50)
  {
       pwm.setPWM(PWM_CHANNEL5, 0, 4095);
    } 
   else   
   {
    pwm.setPWM(PWM_CHANNEL5, 0, 0);
    pwm.setPWM(PWM_CHANNEL6, 0, 0);
    }
    if(mMotMixL>50)
  {
       pwm.setPWM(PWM_CHANNEL7, 0, 600);
    }
    else if(mMotMixL<-50)
  {
       pwm.setPWM(PWM_CHANNEL8, 0, 600);
    } 
   else   
   {
    pwm.setPWM(PWM_CHANNEL7, 0, 0);
    pwm.setPWM(PWM_CHANNEL8, 0, 0);
    }

//-------------------------------------------

  if (ps2x.NewButtonState(PSB_PAD_RIGHT)==1){
    pwm.setPWM(6, 0, 400);
    delay(200);
    pwm.setPWM(6, 0, 0);
  }

  else if (ps2x.NewButtonState(PSB_PAD_LEFT)==1){
    pwm.setPWM(6, 0, 200);
    delay(200);
    pwm.setPWM(6, 0, 0);
  }

//-------------------------------------------
// 
//  else if (ps2x.Button(PSB_PAD_DOWN)==1){
//    pwm.setPWM(5, 0, 0);
//  }

//-------------------------------------------

  if (ps2x.Button(PSB_R1)==0)
  {
    pwm.setPWM(PWM_CHANNEL1, 0, 4095);
  }

  else if (ps2x.Button(PSB_R1)==1){
    pwm.setPWM(PWM_CHANNEL1, 0, 0);
  }

//------------------------------------------

  if (ps2x.Button(PSB_R2)){
    pwm.setPWM(PWM_CHANNEL4, 0, 4095);
  }

  else {
    pwm.setPWM(PWM_CHANNEL4, 0, 0);
  }

//------------------------------------------

  if (ps2x.Button(PSB_L1)==0){
    pwm.setPWM(PWM_CHANNEL2, 0, 4095);
  }

  else if (ps2x.Button(PSB_L1)==1){
    pwm.setPWM(PWM_CHANNEL2, 0, 0);
  }

//------------------------------------------

  if (ps2x.Button(PSB_L2)){
    pwm.setPWM(PWM_CHANNEL3, 0, 4095);
  }

  else {
    pwm.setPWM(PWM_CHANNEL3, 0, 0);
  }

//-----------------------------------------

  if (ps2x.NewButtonState(PSB_PAD_UP)==1){
    pwm.setPWM(4, 0, 90);
  }
  if (ps2x.NewButtonState(PSB_PAD_DOWN)==1){
    pwm.setPWM(4, 0, 0);
  }
  }
  if(ps2x.NewButtonState(PSB_GREEN)==1){
    kt =1;
    dem=0;
  }
    if(ps2x.NewButtonState(PSB_BLUE)==1){
    kt =0;
    Serial.println("xoa");
    pwm.setPWM(PWM_CHANNEL5, 0, 0);
    pwm.setPWM(PWM_CHANNEL6, 0, 0);
    pwm.setPWM(PWM_CHANNEL5, 0, 0);
    pwm.setPWM(PWM_CHANNEL6, 0, 0);
  }
  if(kt==1)
  {
    LineFollowing();
    }
//  else if (ps2x.NewButtonState(PSB_PAD_LEFT)==0){
//    pwm.setPWM(4, 0, 0);
//  }
//  for (uint16_t pulselen = 150; pulselen < 600; pulselen++) {
//    pwm.setPWM(4, 0, pulselen);
//  }
//
//  delay(100);
//  for (uint16_t pulselen = 600; pulselen > 150; pulselen--) {
//    pwm.setPWM(4, 0, pulselen);
//  }
//
//  delay(100);
//----------------------------------------------

//  if (ps2x.NewButtonState(PSB_PAD_RIGHT)==0){
//    pwm.setPWM(4, 0, 0);
//  }
//
//  else if (ps2x.NewButtonState(PSB_PAD_RIGHT)==1){
//    pwm.setPWM(4, 0, 600);
//  }

  
// int c1 = 0, c2 = 0, c3 = 0, c4 = 0, c5 = 0, c6 = 0, c7 = 0, c8 = 0;

//  if (nMotMixR > 50)
//  {
////    c3 = nMotMixR;
//     pwm.setPWM(PWM_CHANNEL5, motor_right_speed, MAX_PWM - motor_right_speed );
//     //pwm.setPWM(PWM_CHANNEL2, 0, MAX_PWM );
//  }
//
//  else if (nMotMixR < -50)
//  {
//     pwm.setPWM(PWM_CHANNEL5, motor_right_speed, MAX_PWM - motor_right_speed );
//     //pwm.setPWM(PWM_CHANNEL1, 0, MAX_PWM );
////    c4 = abs(nMotMixR);
//  }
//
//  if (nMotMixL > 50)
//  {
////     pwm.setPWM(PWM_CHANNEL3, motor_left_speed, MAX_PWM - motor_left_speed );
////     pwm.setPWM(PWM_CHANNEL4, 0, MAX_PWM );
//    c1 = nMotMixL;
//  }
//  else if (nMotMixL < -50)
//  {
//    // pwm.setPWM(PWM_CHANNEL4, motor_left_speed, MAX_PWM - motor_left_speed );
//    // pwm.setPWM(PWM_CHANNEL3, 0, MAX_PWM );
//    c2 = abs(nMotMixL);
//  }
//
//if (mMotMixR > 50)
//  {
////    c7 = mMotMixR;
//     pwm.setPWM(PWM_CHANNEL5, motor_right_speed, MAX_PWM - motor_right_speed );
//     pwm.setPWM(PWM_CHANNEL6, 0, MAX_PWM );
//  }
//
//  else if (mMotMixR < -50)
//  {
//     pwm.setPWM(PWM_CHANNEL6, motor_right_speed, MAX_PWM - motor_right_speed );
//     pwm.setPWM(PWM_CHANNEL5, 0, MAX_PWM );
////    c8 = abs(mMotMixR);
//  }
//
//  if (mMotMixL > 50)
//  {
//     pwm.setPWM(PWM_CHANNEL7, motor_left_speed, MAX_PWM - motor_left_speed );
//     pwm.setPWM(PWM_CHANNEL8, 0, MAX_PWM );
////    c5 = mMotMixL;
//  }
//  else if (mMotMixL < -50)
//  {
//     pwm.setPWM(PWM_CHANNEL7, motor_left_speed, MAX_PWM - motor_left_speed );
//     pwm.setPWM(PWM_CHANNEL8, 0, MAX_PWM );
////    c6 = abs(mMotMixL);
//  }
//  
//  setPWMMotors(c1, c2, c3, c4, c5, c6, c7, c8);
//  delay(50);
//  return 1;


//void psbcontroll()
//(
//  if (ps2x.NewButtonState()) {  
//   if(ps2x.Button(PSB_L1))
//        Serial.println("L3 pressed");
//   if(ps2x.Button(PSB_R3))
//        Serial.println("R3 pressed");
//   if(ps2x.Button(PSB_L2))
//        Serial.println("L2 pressed");
//   if(ps2x.Button(PSB_R2))
//        Serial.println("R2 pressed");
}
