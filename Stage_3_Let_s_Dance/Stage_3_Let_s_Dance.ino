#include <Servo.h>

#define LeftButton 4            //Defining variables to matching pins
#define RighButton 2
#define leftservo  6
#define rightservo 5
#define GreenLED   7
#define YellowLED  12
#define RedLED     13
#define IROutput   3

#define LeftServoStop  102       //Defining constants to stop the servos
#define RightServoStop 89
Servo LeftServo;
Servo RightServo;



void AttachServo() {
  LeftServo.attach(leftservo);
  RightServo.attach(rightservo);
  setSpeed(0,0);  
}



void setSpeed(int LeftSpeed, int RightSpeed) {
  LeftServo.write(LeftServoStop + LeftSpeed);                        //Turn the servos so robot drives at named speed
  RightServo.write(RightServoStop - RightSpeed);             
}



float DriveDistance(float distance) {
  const float Pace = 0.00539665;                   //Pace is the meters travelled in 1 millisecond
  if (distance > 0) {                              
    setSpeed(10, 10); 
    float wait = (distance/Pace);                  //Works out delay time 
    delay(wait);                                   //Robot will drive forward for that time
  }
  else if (distance < 0) {
    setSpeed(-10, -10);
    distance = (distance - (2*distance));         //if distance is negative it will turn it into a positive number
    float wait = (distance/Pace);                      
    delay(wait);                                  //robot will drive backwards 
  }
  setSpeed(0,0);
}



void TurnAngle(float angle) {
  const float TimeForOneDegree = 17.90;               //Time to turn one degree
  if (angle > 0) {
    float DelayTime = (angle * TimeForOneDegree);      //Calculates time turn inputted angle
    setSpeed(20,-20);                                   //Turns right for calculated time
    delay(DelayTime);
  }
  else if (angle < 0) {                               
    float DelayTime = ((angle - (2*angle)) * TimeForOneDegree);     //If angle entered is negative the robot will turn left
    setSpeed(-20,20);
    delay(DelayTime);
    }
  setSpeed(0,0);
}



void LED(int x, int y, int z) {
  digitalWrite(GreenLED,  x);
  digitalWrite(YellowLED, y);
  digitalWrite(RedLED,    z);  
}



void LEDFlash() {
    delay(200);
    LED(1,0,0);
    delay(200);
    LED(1,1,0);
    delay(200);                             //Causes LEDs to blink constantly
    LED(1,1,1);  
    delay(200);
    LED(0,0,0);
}



void Boogie() {
  long first_stage = millis() + 3000;
  long second_stage= millis() + 10000;
  long third_stage = millis() + 16000;                   //Sets variables for each stage of the 'dance'
  long fourth_stage= millis() + 20000;
  int x = 1;
  
  while (x == 1) {
    setSpeed(25,-25);
    LEDFlash();                                        
    if (millis() > first_stage) {
      setSpeed(0,0);
      x = 2;
    }
  }
  
  while (x==2) {
    setSpeed(-25,25);
    LEDFlash();                                        //while the time is under a certain point the robot will perform different 'dance moves'
    if (millis() > second_stage) {
      setSpeed(0,0);
      x = 3;
    }
  }

  while (x==3) {
    setSpeed(25,7);
    delay(1000);
    setSpeed(7,25);
    delay(1000);
    if (millis() > third_stage) {
      setSpeed(0,0);
      x = 4;
    }
  }

  while (x==4) {                                      //Once final stage is complete (After 20 seconds) the dance is over
    setSpeed(25,-25);
    LEDFlash();
    if (millis() > fourth_stage) {
      setSpeed(0,0);
      x = 5;
    }
  }
}



void setup() {
  AttachServo();
  Boogie();
}

void loop() {
  // put your main code here, to run repeatedly:

}
