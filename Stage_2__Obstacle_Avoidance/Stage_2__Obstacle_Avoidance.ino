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



void setup() {
  Serial.begin(9600);
  AttachServo();
  pinMode(LeftButton,INPUT);
  pinMode(IROutput, OUTPUT);


  delay(3000);
  setSpeed(11,10);

}



void loop() {
  tone(IROutput,38000);                                  //Will turn on the IR scanner to check for object
  delay(125);
  int Object = (digitalRead(2));
  noTone(IROutput);
  delay(125);
  Serial.println(Object);
  if (Object == LOW) {                           //If an object is detected then the robot will carry out the task to avoid                                  
    LED(1,1,1);
    TurnAngle(90);
    DriveDistance(25);                                   //Will turn on all LEDs, turn right and drive 25cm before turning left and driving straight on
    TurnAngle(-90);
    LED(0,0,0);
    setSpeed(7,7);
  }
}
