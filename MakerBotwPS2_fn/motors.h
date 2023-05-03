#include <stdio.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define MIN_PWM 0
#define MAX_PWM 4095

//#define SDA 3
//#define SCL 0
// PWM channels of pca9685 0-16
#define PWM_CHANNEL5 8
#define PWM_CHANNEL6 9
#define PWM_CHANNEL7 10
#define PWM_CHANNEL8 11

#define PWM_CHANNEL1 12 
#define PWM_CHANNEL2 13
#define PWM_CHANNEL3 14 
#define PWM_CHANNEL4 15


int dem;
#define chan1 2
#define chan2 3
#define chan3 4
#define chan4 5//----------------------------------------------------------------------------------------------------
#define chan5 6
#define chan6 7



Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setPWMMotors(int c1, int c2, int c3, int c4, int c5, int c6, int c7, int c8)
{
  /*
  // setPWM(channel, on_duty_cycle, off_duty_cycle)
  Serial.print(c1);
  Serial.print("\t");
  Serial.print(c2);
  Serial.print("\t");
  Serial.print(c3);
  Serial.print("\t");
  Serial.print(c4);
  Serial.print("\t");
  Serial.print(c5);
  Serial.print("\t");
  Serial.print(c6);
  Serial.print("\t");
  Serial.print(c7);
  Serial.print("\t");
  Serial.print(c8);
  Serial.println();
*/
  pwm.setPWM(PWM_CHANNEL1, c1, MAX_PWM - c1);
  pwm.setPWM(PWM_CHANNEL2, c2, MAX_PWM - c2);
  pwm.setPWM(PWM_CHANNEL3, c3, MAX_PWM - c3);
  pwm.setPWM(PWM_CHANNEL4, c4, MAX_PWM - c4);
  pwm.setPWM(PWM_CHANNEL5, c5, MAX_PWM - c5);
  pwm.setPWM(PWM_CHANNEL6, c6, MAX_PWM - c6);
  pwm.setPWM(PWM_CHANNEL7, c7, MAX_PWM - c7);
  pwm.setPWM(PWM_CHANNEL8, c8, MAX_PWM - c8);
 
  
}


void initMotors()
{
  Wire.begin(); //SDA, SCL,400000);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(1600);
  Wire.setClock(400000);

  setPWMMotors(0, 0, 0, 0, 0, 0, 0, 0);
}

void setupPWMServo()
{
 pwm.writeMicroseconds(chan1, 2000);
 pwm.writeMicroseconds(chan2, 2000);
}

void LineFollowing()
{

   { pwm.setPWM(PWM_CHANNEL5, 0, 3500);
//    }
//  if((analogRead(2)>1500) && (analogRead(36)>1000) && (analogRead(32)>2000))
//  {
//    delay(500);//tien_thang
//    pwm.setPWM(PWM_CHANNEL8, 0, 0);
//    pwm.setPWM(PWM_CHANNEL6, 0, 800); // quay phiar hoawcj quay trais
//    delay(500);// 
//    while(analogRead(32)<2000){    Serial.println("ktra1");}
//    pwm.setPWM(PWM_CHANNEL6, 0, 0);
    dem=1;
    //}
//  if((dem==1) && analogRead(32)>2000)
//  {
//    pwm.setPWM(PWM_CHANNEL8, 0, 3500);
    while(analogRead(39)>2000){Serial.println("ktra2");}
    pwm.setPWM(PWM_CHANNEL5, 0, 0);
    dem=2;
    }
   if(dem==2)
   {
    pwm.setPWM(6, 0, 200);
    delay(400);
    pwm.setPWM(6, 0, 0);
    delay(100);
    pwm.setPWM(PWM_CHANNEL4, 0, 3500);
    delay(3700);// thoi_gian nang hop
    pwm.setPWM(PWM_CHANNEL4, 0, 0);
    delay(100);
    pwm.setPWM(PWM_CHANNEL1, 0, 0);
    delay(3000);// thoi_gian day hop
    pwm.setPWM(PWM_CHANNEL1, 0, 4095);
    delay(100);
//    pwm.setPWM(PWM_CHANNEL6, 0, 800);
//    delay(500);
//    while(analogRead(32)<2000){Serial.println("ktra3");}
//    pwm.setPWM(PWM_CHANNEL6, 0, 0);
    dem=3;
   }
    
    if (dem==3)
    {
      pwm.setPWM(PWM_CHANNEL6, 0, 3500);
      delay(5000);
      dem = 4;
    }

}
