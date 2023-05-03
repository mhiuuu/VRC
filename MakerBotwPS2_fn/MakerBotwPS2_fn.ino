
#include "motors.h"
#include "PS2_controller.h"
#include "PS2X_lib.h"
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

int i;

// our servo # counter
uint8_t servonum = 0;
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void setup()
{
  Serial.begin(115200);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  //initMotors();
  setupPS2controller();
  pwm.setPWM(4, 0, 0);
  pwm.setPWM(6, 0, 0);
  Serial.println("Done setup!");
}


void loop()
{
  ps2x.read_gamepad(false, 0);
  PS2control();
 // LineFollowing();


//  if(ps2x.ButtonPressed(PSB_GREEN))
//  {
//    for (int i=0; i < 1000000000000000000; i++);
//    Serial.println("automatic");
//    if (i == 0){
//    LineFollowing();
//    }
//    else {
//      LineFollowing();
//      Serial.println("giá trị i hiện tại:");
//      Serial.println(i);
//    }
//  }
//
//  if(ps2x.ButtonPressed(PSB_RED))
//  {
//    for (int i = 50000000001; i < 100000000000000000000000; i++);
//    Serial.println("manual control");
//    if (i == 50000000001) {
//      PS2control();
//    }
//    else {
//      Serial.println("giá trị i hiện tại:");
//      Serial.println(i);
//      PS2control();
//    }
//  }



//---------------------------------------



//if(ps2x.Button(PSAB_PAD_RIGHT)==1)
//{
//  pwm.setPWM(7, 0, 500);
//  }
// else pwm.setPWM(7, 0, 100);
//    delay(10);
//      Serial.println(servonum);
//  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
//    pwm.setPWM(7, 0, pulselen);
//     delay(10);
//  }
//
  delay(100);
//  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
//    pwm.setPWM(7, 0, pulselen);
//       delay(10);
//  }
}
