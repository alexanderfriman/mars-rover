#include <esp_wifi.h>
#include <PS4Controller.h> // library for the playstation controller
#include <ESP32Servo.h> // library for controlling servos
#include <Arduino.h>
#include <TB6612_ESP32.h> // library for controlling the motordrivers
#include <ServoEasing.hpp> // library for making the servo movement smoother


#define AIN1 13 // ESP32 Pin D13 to TB6612FNG1 Pin AIN1
#define BIN1 12 // ESP32 Pin D12 to TB6612FNG1 Pin BIN1
#define AIN2 14 // ESP32 Pin D14 to TB6612FNG1 Pin AIN2
#define BIN2 27 // ESP32 Pin D27 to TB6612FNG1 Pin BIN2
#define PWMA1 26 // ESP32 Pin D26 to TB6612FNG1 Pin PWMA
#define PWMB1 25 // ESP32 Pin D25 to TB6612FNG1 Pin PWMB
#define STBY1 33 // ESP32 Pin D33 to TB6612FNG1 Pin STBY 

#define AIN3 19 // ESP32 Pin D19 to TB6612FNG2 Pin AIN1
#define BIN3 18 // ESP32 Pin D18 to TB6612FNG2 Pin BIN1
#define AIN4 5 // ESP32 Pin D5 to TB6612FNG2 Pin AIN2
#define BIN4 4 // ESP32 Pin D4 to TB6612FNG2 Pin BIN2
#define PWMA2 0 // ESP32 Pin D0 to TB6612FNG2 Pin PWMA
#define PWMB2 2 // ESP32 Pin D2 to TB6612FNG2 Pin PWMB
#define STBY2 15 // ESP32 Pin D15 to TB6612FNG2 Pin STBY

// used in the delay for the servos
#define SERVO_INTERVAL    7 // delay of 7 ticks
unsigned long servo_timestamp = 0;

// change the mac adress of the esp32 so that it can connect to the playstation controller
uint8_t newMACAddress[] = {0xE8, 0x9E, 0xB4, 0xAD, 0x8B, 0x4C}; 

// define the servomotors
ServoEasing servo_leftFront; 
ServoEasing servo_leftBack;
ServoEasing servo_rightFront;
ServoEasing servo_rightBack;

int servo_leftFront_angle = 90; 
int servo_leftBack_angle = 90;
int servo_rightFront_angle = 90;
int servo_rightBack_angle = 90;
int controller_turning = 0;


int controller_speed = 0; // controller value for speed
int speed = 0; // rover speed
int r = 0; // turning radius
float speed1, speed2, speed3, speed4 = 0; // the different wheel speeds
float thetaInnerFront, thetaInnerBack, thetaOuterFront, thetaOuterBack, thetaFront, thetaBack = 0; // different turning angles for the servomotors

// distance in mm
float a = 190;  // distance between front wheels
float b = 270;  // distance between middle wheels
float c = 240;  // distance between back wheels
float d = 250;  // distance between front and middle wheels
float e = 195;  // distance between back and middle wheels

// changes the default direction of the dc-motors
const int offsetA = -1; 
const int offsetB = 1;

Motor motor_ML = Motor(AIN1, AIN2, PWMA1, offsetA, STBY1,5000 ,8,1 ); // middle left motor
Motor motor_MR = Motor(BIN1, BIN2, PWMB1, offsetB, STBY1,5000 ,8,2 ); // middle right motor
Motor motor_OL = Motor(AIN3, AIN4, PWMA2, offsetA, STBY2,5000 ,8,1 ); // left front and back motors
Motor motor_OR = Motor(BIN3, BIN4, PWMB2, offsetB, STBY2,5000 ,8,2 ); // right front and back motors

void setup()
{
  Serial.begin(115200);
  PS4.begin("a1:b2:c3:d4:e5:f6"); //Enter your ps4 controllers mac-adress

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo_leftFront.setPeriodHertz(50);// Standard 50hz servo
  servo_leftBack.setPeriodHertz(50);
  servo_rightFront.setPeriodHertz(50); 
  servo_rightBack.setPeriodHertz(50);
  
  servo_leftFront.attach(21);  // connect left front Servo to pin 21
  servo_leftBack.attach(22);  // connect left back Servo to pin 22
  servo_rightFront.attach(23);  // connect right front Servo to pin 23
  servo_rightBack.attach(32); // connect right back Servo to pin 32

  // rotates all servos forward
  servo_leftFront.write(80);
  servo_leftBack.write(88);
  servo_rightFront.write(46);
  servo_rightBack.write(97);

  // sets the turning speed for the servos
  servo_leftFront.setSpeed(550);
  servo_leftBack.setSpeed(550);
  servo_rightFront.setSpeed(550);
  servo_rightBack.setSpeed(550);



}

void loop() {
  if (PS4.isConnected()) {
      // the delay for the servos
      if(millis()-servo_timestamp > SERVO_INTERVAL){
        servo_timestamp += SERVO_INTERVAL;
        // takes the right stick Y-value of the controller and changes it to a corresponding speed value for the dc-motors
        controller_speed = PS4.RStickY();
        speed=map(controller_speed,-128,128,-255,255);
        // takes the left stick X-value of the controller and changes it to a corresponding turning radius between 250mm and 800mm
        controller_turning = PS4.LStickX();
        r = map(abs(controller_turning),0,128,800,250);

        calculateMotorsSpeed(); // calculates the different dc motorspeeds
        calculateServoAngle(); // calculates the different servoangles

        if (PS4.Right()) { // turns the rover on the spot to the right
          // slowly changes the servos to the calculated angles
          servo_leftFront.startEaseTo(80+thetaFront);  // leftside turns to the right
          servo_leftBack.startEaseTo(88-thetaBack);
          servo_rightFront.startEaseTo(46-thetaFront); // rightside turns to the left
          servo_rightBack.startEaseTo(97+thetaBack);
          
          // speed control for the dc motors
          analogWrite(PWMA1,abs(speed1)); 
          analogWrite(PWMB1,abs(speed1));
          analogWrite(PWMA2,abs(speed2)); 
          analogWrite(PWMB2,abs(speed2));
          // left side goes forward and the right side backwards
          motor_ML.drive(speed1);
          motor_OL.drive(speed2);
          motor_MR.drive(-speed1);
          motor_OR.drive(-speed2);
        }
        else if (PS4.Left()) { //turns the rover on the spot to the left
          servo_leftFront.startEaseTo(80+thetaFront); // 
          servo_leftBack.startEaseTo(88-thetaBack);
          servo_rightFront.startEaseTo(46-thetaFront);
          servo_rightBack.startEaseTo(97+thetaBack);

          analogWrite(PWMA1,abs(speed1)); 
          analogWrite(PWMB1,abs(speed1));
          analogWrite(PWMA2,abs(speed2)); 
          analogWrite(PWMB2,abs(speed2));
          // right side forward and leftside backwards
          motor_ML.drive(-speed1);
          motor_OL.drive(-speed2);
          motor_MR.drive(speed1);
          motor_OR.drive(speed2);
        }
        else if (controller_turning>5){ //Steering right
          servo_leftFront.startEaseTo(80+thetaOuterFront);
          servo_leftBack.startEaseTo(88-thetaOuterBack);
          servo_rightFront.startEaseTo(46+thetaInnerFront);
          servo_rightBack.startEaseTo(97-thetaInnerBack);

          analogWrite(PWMA1,abs(speed1)); 
          analogWrite(PWMB1,abs(speed3));
          analogWrite(PWMA2,abs(speed2)); 
          analogWrite(PWMB2,abs(speed4));

          motor_ML.drive(speed1);
          motor_OL.drive(speed2);
          motor_MR.drive(speed3);
          motor_OR.drive(speed4);
        }
        else if (controller_turning<-5){ //Steering left
          servo_rightFront.startEaseTo(46-thetaOuterFront);
          servo_rightBack.startEaseTo(97+thetaOuterBack);
          servo_leftFront.startEaseTo(80-thetaInnerFront);
          servo_leftBack.startEaseTo(88+thetaInnerBack);

          analogWrite(PWMA1,abs(speed3)); 
          analogWrite(PWMB1,abs(speed1));
          analogWrite(PWMA2,abs(speed4)); 
          analogWrite(PWMB2,abs(speed2));

          motor_MR.drive(speed1);
          motor_OR.drive(speed2);
          motor_ML.drive(speed3);
          motor_OL.drive(speed4);
        }
        else{ //Not turning
          servo_leftFront.startEaseTo(80);
          servo_leftBack.startEaseTo(88);
          servo_rightFront.startEaseTo(46);
          servo_rightBack.startEaseTo(97);

          analogWrite(PWMA1,abs(speed1)); 
          analogWrite(PWMB1,abs(speed1));
          analogWrite(PWMA2,abs(speed1)); 
          analogWrite(PWMB2,abs(speed1));

          motor_MR.drive(speed1);
          motor_OR.drive(speed1);
          motor_ML.drive(speed1);
          motor_OL.drive(speed1);
        }
      }

  }
    
}


void calculateMotorsSpeed() {
  // if no steering, all wheels speed is the same - straight move
  if (controller_turning > -5 && controller_turning < 5) {
    speed1 = speed2 = speed3 = speed;
  }
  // when steering, wheels speed depends on the turning radius value
  else {
    // Outer wheels, furthest wheels from turning point, have max speed
    // Due to the rover geometry, the outer front and back wheels have the same speed.
    speed1 = speed; //Outer middle wheel

    speed2 = speed * sqrt(pow(r+(a+c)/4, 2) + pow(((d+e)/2), 2)) / (r + b/2); // Outer front and back wheels
    // Inner middle wheel is closest to the turning point, has the lowest speed
    speed3 = speed * (r - b/2) / (r + b/2); // inner middle
    
    speed4 = speed * sqrt(pow(r-(a+c)/4, 2) + pow(((d+e)/2), 2)) / (r + b/2); // inner front and back
  }
}


void calculateServoAngle() {
  // Calculate the angle for each servo for the input turning radius "r"
  // inner wheels
  thetaInnerFront = round((atan((d / (r - a/2)))) * 180 / PI); 
  thetaInnerBack = round((atan((e / (r - c/2)))) * 180 / PI);
  // outer wheels
  thetaOuterFront = round((atan((d / (r + a/2)))) * 180 / PI); 
  thetaOuterBack = round((atan((e / (r + c/2)))) * 180 / PI);
  // When turning on the spot, the turning radius is 0 and both sides behave like the outer side
  thetaFront = round((atan((d / (a/2)))) * 180 / PI);
  thetaBack = round((atan((e / (c/2)))) * 180 / PI);
}