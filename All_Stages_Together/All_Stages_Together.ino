#include <Servo.h>


#define LeftButton 4            //Defining variables to matching pins
#define RightButton 2
#define leftservo  6
#define rightservo 5
#define GreenLED   7
#define YellowLED  12
#define RedLED     13
#define IROutput   3
#define LeftLDR    A2
#define RightLDR   A0
#define CentreLDR  A1

#define LeftServoStop  102       //Defining constants to stop the servos
#define RightServoStop 89
Servo LeftServo;
Servo RightServo;

int DarkToLight;                  //Light to dark and drak to light threshholds used to defferentiate between light and dark spots
int LightToDark;



void AttachServo() {
  LeftServo.attach(leftservo);
  RightServo.attach(rightservo);
  setSpeed(0,0);  
}



void LDRsetup() {
  pinMode(RightLDR, INPUT);
  pinMode(CentreLDR, INPUT);
  pinMode(LeftLDR, INPUT); 
}



void setSpeed(int LeftSpeed, int RightSpeed) {
  LeftServo.write(LeftServoStop + LeftSpeed);                        //Turn the servos so robot drives at named speed
  RightServo.write(RightServoStop - RightSpeed);             
}



float DriveDistance(float distance) {
  const float Pace = 0.00539665;                   //Pace is the meters travelled in 1 millisecond
  if (distance > 0) {                              
    setSpeed(10, 10); 
    float wait = (distance/Pace);                  //Works out delay time using distance time calculation
    delay(wait);                                   //Robot will drive forward for that time which should equal the length entered by the user
  }
  else if (distance < 0) {
    setSpeed(-10, -10);
    distance = (distance - (2*distance));         //If distance is negative it will turn it into a positive number
    float wait = (distance/Pace);                      
    delay(wait);                                  //Robot will drive backwards for that time which will equal the distance entered
  }
  setSpeed(0,0);
}



void TurnAngle(float angle) {
  const float TimeForOneDegree = 17.90;               //Time to turn one degree
  if (angle > 0) {
    float DelayTime = (angle * TimeForOneDegree);      //Calculates time to turn inputted angle
    setSpeed(20,-20);                                   //Turns right for calculated time
    delay(DelayTime);
  }
  else if (angle < 0) {                               
    float DelayTime = ((angle - (2*angle)) * TimeForOneDegree);     //If angle entered is negative the robot will turn left
    setSpeed(-20,20);                                               //Will turn the negative number into a positive one in order to do the calculation
    delay(DelayTime);
    }
  setSpeed(0,0);
}



int WorkOutDarkLightLevel() {
  int DarkLightLevel = 0;
  for (int x=1;x<11;x++) {                                
    DarkLightLevel = (DarkLightLevel + LightLevel());                                               //Will work out a value for a dark stripe
  }                                                                                                 //Will get an average value from ten reading to get a more accurate result
  DarkLightLevel = (DarkLightLevel / 10);
  return DarkLightLevel;                                                                           //Will return this value to the CalibrateDarkValue function
}



int WorkOutWhiteLightLevel() {
  int WhiteLightLevel = 0;
  for (int x=1;x<11;x++) {                                                                         //Will work out an average value for a white stripe
    WhiteLightLevel = (WhiteLightLevel + LightLevel());
  }
  WhiteLightLevel = (WhiteLightLevel / 10);                                                        //Will return this value to the CalibrateWhiteValue function
  return WhiteLightLevel;
}



int CalibrateDarkValue() {
  Serial.println("Press the left button when the robot is on a dark surface.");                          //Will wait for the user to put robot on dark strip and press the left button
  while (digitalRead(LeftButton) == HIGH) {}
  delay(200);
  while (digitalRead(LeftButton) == LOW) {}
  delay(200);
  int DarkLightLevel = WorkOutDarkLightLevel();
  Serial.print("The light level for a black strip is " );                                            //Will record and store the light level for the dark strip by calling the function which reads the light levels
  Serial.println(DarkLightLevel);
  Serial.println();
  return DarkLightLevel;
}



int CalibrateWhiteValue() {
  Serial.println("Press the left button when the robot is on a white/light surface.");            //Will wait for the user to put the robot on a white patch/stripe
  while (digitalRead(LeftButton) == HIGH) {}
  delay(200);
  while (digitalRead(LeftButton) == LOW) {}
  delay(200);
  int WhiteLightLevel = WorkOutDarkLightLevel();
  Serial.print("The light level for a white part is " );
  Serial.println(WhiteLightLevel);                                                                //Will record and store the light level for the white stripes/areas
  Serial.println();
  return WhiteLightLevel;                                                                        //The values returned will go to the main code
}



void LED(int x, int y, int z) {
  digitalWrite(GreenLED, x);
  digitalWrite(YellowLED,y);                                //Turns LEDs on/off depending on parameters entered (1 for on, 0 for off)
  digitalWrite(RedLED, z);
  
}



void TurnLeft() {
  LED(1,0,0);
  TurnAngle(-90);                                          //Function to turn left if 'left' barcode is read
  setSpeed(6,6);
  BarcodeScan();                                           //After the robot has turned it is returned to the state ready to scan for another barcode
}



void TurnRight() {
  LED(0,1,0);
  TurnAngle(90);                                         //Function to turn right if 'right' barcode is read
  setSpeed(6,6);
  BarcodeScan();                                         //Robot ready to scan another barcode again
}



void UTurn() {
  LED(1,1,0);                                            //Function for a U-Turn 
  TurnAngle(180);
  setSpeed(6,6);
  BarcodeScan();                                         //Robot is ready to scan again
  }


  
void Stop() {
  LED(0,0,1);
  setSpeed(0,0);                                        //Stops the robot and ends the whole cycle (Robot will not be in a state ready to scan)
}



void RobotBoogie() {
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

  while (x==4) {
    setSpeed(25,-25);
    LEDFlash();
    if (millis() > fourth_stage) {                            //Once the final stage is complete (After 20 seconds) the robot will return to a state where it searches for a barcode
      setSpeed(6,6);
      x = 5;
      BarcodeScan();
    }
  }
}



void LEDFlash() {
    delay(200);
    LED(1,0,0);
    delay(200);
    LED(1,1,0);
    delay(200);                             //Causes LEDs to blink constantly
    LED(1,1,1);                             //This function is called while the boogie function is carried out
    delay(200);
    LED(0,0,0);
}



void ObstacleCheck() {
  tone(IROutput,38000);                                  //Will turn on IR scanner to search for obstacle
  delay(125);
  int ObjectFound = (digitalRead(2));                  //Assigns variable
  noTone(IROutput);
  delay(125);
  if (ObjectFound == LOW) {                           //If an obstacle is found it will call the avoid function
    ObstacleAvoid();
  }
}



void ObstacleAvoid() {                                    //Function is called if object is detected and will first turn off the scanner
    LED(1,1,1);
    TurnAngle(90);
    DriveDistance(25);                                   //The robot will turn left, drive 25cm and turn right before continuing straight in order to avoid the object
    TurnAngle(-90);
    LED(0,0,0);
    setSpeed(6,6);
}



int LightLevel() {
  int RightValue = analogRead(RightLDR);
  int CentreValue = analogRead(CentreLDR);                                                  //Reads the light values for each LDR and calculates an average for them
  int LeftValue = analogRead(LeftLDR);
  int AverageValue = ((RightValue + CentreValue + LeftValue) / 3);
  return AverageValue;
}



void BarcodeScan() {
  long BarWidths [4] = {};                                                           //Array that will store the values for the barcode that the robot will scan


  int LeftTurnBarcode  [4] = {1,1,0,0};                                                      //Arrays for barcodes. If bar is thick it is assigned a 0, if bar is thin it is assigned a 0
  int RightTurnBarcode [4] = {0,1,0,1};
  int UturnBarcode     [4] = {0,0,1,0};
  int StopBarcode      [4] = {0,1,1,0};
  int BoogieBarcode    [4] = {1,0,0,1};
  const int ThreshForBarWidth  = 1200 ;                                             //Threshold for for thin and thick bars (Less than value is a thin bar, more than for thick bar)
  


  int SearchForBlackBar = 1;
  int y = 0;

  
  while (SearchForBlackBar == 1) {
    int Brightness = LightLevel();                                                  //Drives forward and wais for a black bar to be read which indicates a barcode has been found
    setSpeed(6,6);
    if (Brightness < LightToDark) {
      SearchForBlackBar = 0;                                                        //If black strip is found it will begin the scanning phase
    }
    if (SearchForBlackBar == 1) {
    ObstacleCheck();                                                                //Checks for an obstacle while looking for a barcode
    }
  }

  while (y<4) {
    int AverageValue = LightLevel();
    long Start;
    long End;                                                                      
    long Width;

    if (AverageValue < LightToDark) {
     Start = millis();                                                                  //Starts the timer while driving over a black bar
     while (AverageValue < DarkToLight) {
       LED(1,1,1);                                                                     //LEDs on while driving over a black bar
       AverageValue = LightLevel(); 
     }
    }

    
    if (AverageValue > DarkToLight) {
      End = millis();                                                                    //When white is detected it will record the time and work out the time spent travelling over a black bar
      Width = End-Start;
      BarWidths[y] = Width;                                                              //Will store the time taken in the array (Will do this for each of the 4 black bars)
      y++;
      if (y > 3) {
        break;
      }
      else {
        while (AverageValue > LightToDark) {                                              //While travelling over a white space it will turn LEDs off 
          LED(0,0,0);
          AverageValue = LightLevel();
        }
     }
   } 
  

  }
  delay(8000);                                                                            //Delay to allow robot to drive away from the barcode 
  
  for (int index = 0; index < 4; index++) {                                              //Compares time over balck bars to the threshhold and converts it into binary
    if (BarWidths[index] < ThreshForBarWidth) {
      BarWidths[index] = 0;                                                              // 0 for thin strip (If the time is less than 1200 ms)
    }
    else {
      BarWidths[index] = 1;                                                              //1 for thick strip (If the time is greater than 1200 ms)
    }
  }


  int LeftTurn = 1;                                                                      
  for (int i = 0; i < 4; i++) {
    if (BarWidths[i] == LeftTurnBarcode[i]) {                                            //Will compare each element in the LeftTurn array to the recorded array and see if they match
      LeftTurn = 1;
    }
    else {
      LeftTurn = 0;                                                                      //If they all match the variable will return 1 (True)
      break;
    }
  }


  int RightTurn = 1;                                                                     //Will do this for all of the stored barcode arrays
  for (int i = 0; i < 4; i++) {
    if (BarWidths[i] == RightTurnBarcode[i]) {
      RightTurn = 1;
    }
    else {
      RightTurn = 0;
      break;
    }
  }


  int Uturn = 1;
  for (int i = 0; i < 4; i++) {
    if (BarWidths[i] == UturnBarcode[i]) {
      Uturn = 1;
    }
    else {
      Uturn = 0;
      break;
    }
  }



  int StopRobot = 1;
  for (int i = 0; i < 4; i++) {
    if (BarWidths[i] == StopBarcode[i]) {
      StopRobot = 1;
    }
    else {
      StopRobot = 0;
      break;
    }
  }



  int Boogie = 1;
  for (int i = 0; i < 4; i++) {
    if (BarWidths[i] == BoogieBarcode[i]) {
      Boogie = 1;
    }
    else {
      Boogie = 0;
      break;
    }
  }

  
  
  
  
  
  
  LED(0,0,0);
  if (LeftTurn == 1) {                                                                              //After all barcodes have been checked, the value which returned 1 will be the function called
    TurnLeft();
  }
  else if (RightTurn == 1) {
    TurnRight();
  }
  else if (Uturn == 1) {
    UTurn();
  }
  else if (StopRobot == 1) {
    Stop();
  }
  else if (Boogie == 1) {
    RobotBoogie();                                                                                  //If none of them are 1 (Invalid barcode) The robot will continue to drive on
  }
  else {
    BarcodeScan();                                                                                  //If no matching barcodes are found it will drive on and continue to look for another one
  }
}



void setup() {
  AttachServo();
  LDRsetup();
  
  
  Serial.begin(9600);


  int DarkValue =   CalibrateDarkValue();                                                //Calibrates the threshholds that the robot will use to defferentiate between light and dark stripes
  int LightValue  = CalibrateWhiteValue();
  int MiddleValue = ((LightValue + DarkValue) / 2);
      DarkToLight = (MiddleValue + 22);                                                  //The treshold allow for the robot to differentiate between light and dark strips more easily
      LightToDark = (MiddleValue - 22);
  
  
  while (digitalRead(LeftButton) == HIGH) {}
  delay(200);
  while (digitalRead(LeftButton) == LOW) {}
  delay(200);
  BarcodeScan();                                                                         //Once calibration is complete the scanning phase will begin after user presses left button


  
}

void loop() {
  
  
     
   

}
