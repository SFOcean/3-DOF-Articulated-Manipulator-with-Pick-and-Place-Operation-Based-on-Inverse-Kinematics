#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver myServo = Adafruit_PWMServoDriver();

#define SERVOMIN 150
#define SERVOMAX 600

uint8_t servonum = 0;
uint8_t numberOfServos = 4;

//Link Lengths
float a1 = 9.5;
float a2 = 10.5;
float a3 = 19.0;
float currentPos0 = 390;  //thet1 ++ 15
float currentPos1 = 380;  //thet2 ++ 5
float currentPos2 = 405;  //thet3 ++ 30

float theta1, theta2, theta3, minusx, r1, r2, r3, pi1, pi2, pi3, thet1, thet2, thet3;


void setup() {
  //Serial.begin(9600);
  myServo.begin();
  myServo.setPWMFreq(60);
  delay(10);
}

void loop() {
  
  //User Defined Coordinate Values for Inverse Kinematics
  
  //For PICKing Position
  float x1 = 16.0; // Coordinate of X axis
  float y1 = 10.0; // Coordinate of Y axis
  float z1 = 3.0;  // Coordinate of Z axis

  //For PLACEing Position
  float x2 = -20.0; // Coordinate of X axis
  float y2 = 15.0;  // Coordinate of Y axis
  float z2 = 4.0;   // Coordinate of Z axis

  float x3 = 0.0; 
  float y3 = 18.0;
  float z3 = 18.0;
  
  currentPos0 = 390;  //thet1 ++ 15
  currentPos1 = 380;  //thet2 ++ 5
  currentPos2 = 405;  //thet3 ++ 30
  
  myServo.setPWM(0, 0, currentPos0);
  myServo.setPWM(1, 0, currentPos1);
  myServo.setPWM(2, 0, currentPos2);
  myServo.setPWM(3, 0, 430);
  delay(3000);

  inverse_second(x1, y1, z1, currentPos0, currentPos1, currentPos2);
  delay(1000);

  inverse_first(x1, y1, z3, currentPos0, currentPos1, currentPos2);
  
  inverse_first(x3, y3, z3, currentPos0, currentPos1, currentPos2);
  inverse_first(x3, y3, z3, currentPos0, currentPos1, currentPos2);
  delay(500);
  inverse_first(x2, y2, z2, currentPos0, currentPos1, currentPos2);
  delay (3000);
  myServo.setPWM(3, 0, 250);
  delay(3000);
  inverse_first(x2, y2, z3, currentPos0, currentPos1, currentPos2);
  inverse_first(x3, y3, z3, currentPos0, currentPos1, currentPos2);
  myServo.setPWM(3, 0, 430);
  while(1){ 
  }
}


//Inverse Kinematics Function
float inverse_first(float x, float y, float z, float cPos1, float cPos2, float cPos3){
  //calculate theta1
   if (x>-1 ) {
   theta1 = atan(y/x);
   theta1 = theta1*180/PI;
   }
   else {
   minusx = -x;
   theta1 = atan(y/minusx);
   theta1 = theta1*180/PI; 
   theta1 = 180-theta1;
   }
   
  //calculate r1,r2,r3
   r1 = sqrt((sq(x))+(sq(y)));
   r2 = z-a1;
   r3 = sqrt((sq(r2))+(sq(r1)));
   
  //calculate pi1,pi2,pi3 
   pi1 = atan(r2/r1)*180/PI;
   pi2 = acos(((sq(a2))+(sq(r3))-(sq(a3))) 
   /(2*a2*r3))*180/PI;
   pi3 = acos(((sq(a2))+(sq(a3))-(sq(r3))) 
   / (2*a2*a3))*180/PI;
  //calculate theta2,theta3,theta4
   theta2 = pi1+pi2;
   theta3 = 180-pi3;

   thet1 = (2.5*theta1)+150+10;
   thet2 = (2.5*theta2)+150+5;
   thet3 = (2.5*theta3)+150+30;  //By adding offset

  // For Joint 1
   if (thet1 >= cPos1){
    for (int i = cPos1; i <= thet1; i++){
      myServo.setPWM(0, 0, i);
      delay(10);
    }
   }
    else {
      for (int i = cPos1; i >= thet1; i--){
      myServo.setPWM(0, 0, i);
      delay(10);
    }
   }
     // For joint 3
   if (thet3 >= cPos3){
    for (int i = cPos3; i <= thet3; i++){
      myServo.setPWM(2, 0, i);
      delay(10);
    }
   }
    else {
      for (int i = cPos3; i >= thet3; i--){
      myServo.setPWM(2, 0, i);
      delay(10);
    }
   }
   // For Joint 2
   if (thet2 >= cPos2){
    for (int i = cPos2; i <= thet2; i++){
      myServo.setPWM(1, 0, i);
      delay(10);
    }
   }
    else {
      for (int i = cPos2; i >= thet2; i--){
      myServo.setPWM(1, 0, i);
      delay(10);
    }
   }
   
 
   currentPos0 = thet1;
   currentPos1 = thet2;
   currentPos2 = thet3;
  }


  //////////
  float inverse_second(float x, float y, float z, float cPos1, float cPos2, float cPos3){
  //calculate theta1
   if (x>-1 ) {
   theta1 = atan(y/x);
   theta1 = theta1*180/PI;
   }
   else {
   minusx = -x;
   theta1 = atan(y/minusx);
   theta1 = theta1*180/PI; 
   theta1 = 180-theta1;
   }
   
  //calculate r1,r2,r3
   r1 = sqrt((sq(x))+(sq(y)));
   r2 = z-a1;
   r3 = sqrt((sq(r2))+(sq(r1)));
   
  //calculate pi1,pi2,pi3 
   pi1 = atan(r2/r1)*180/PI;
   pi2 = acos(((sq(a2))+(sq(r3))-(sq(a3))) 
   /(2*a2*r3))*180/PI;
   pi3 = acos(((sq(a2))+(sq(a3))-(sq(r3))) 
   / (2*a2*a3))*180/PI;
  //calculate theta2,theta3,theta4
   theta2 = pi1+pi2;
   theta3 = 180-pi3;

   thet1 = (2.5*theta1)+150+10;
   thet2 = (2.5*theta2)+150+5;
   thet3 = (2.5*theta3)+150+30;  //By adding offset

  // For Joint 1
   if (thet1 >= cPos1){
    for (int i = cPos1; i <= thet1; i++){
      myServo.setPWM(0, 0, i);
      delay(10);
    }
   }
    else {
      for (int i = cPos1; i >= thet1; i--){
      myServo.setPWM(0, 0, i);
      delay(10);
    }
   }
     // For joint 3
   if (thet3 >= cPos3){
    for (int i = cPos3; i <= thet3; i++){
      myServo.setPWM(2, 0, i);
      delay(10);
    }
   }
    else {
      for (int i = cPos3; i >= thet3; i--){
      myServo.setPWM(2, 0, i);
      delay(10);
    }
   }
   
  myServo.setPWM(3, 0, 250);
   // For Joint 2
   if (thet2 >= cPos2){
    for (int i = cPos2; i <= thet2; i++){
      myServo.setPWM(1, 0, i);
      delay(10);
    }
   }
    else {
      for (int i = cPos2; i >= thet2; i--){
      myServo.setPWM(1, 0, i);
      delay(10);
    }
   }
  
  delay (3000);
  myServo.setPWM(3, 0, 450);
 
   currentPos0 = thet1;
   currentPos1 = thet2;
   currentPos2 = thet3;
  }

  
