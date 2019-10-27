#include <Servo.h>


#define LeftButton 4                                      //Defining variables to matching pins on robot shell
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

#define LeftServoStop  102                                 //Defining constants to stop the servos
#define RightServoStop 89
Servo LeftServo;
Servo RightServo;

int DarkToLight;                                           //Light to dark and drak to light threshholds used to defferentiate between light and dark spots
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
  RightServo.write(RightServoStop - RightSpeed);                     //Servos turn in opposite directions for the robot to move forward
}



float DriveDistance(float distance) {
  const float Pace = 0.00539665;                   //Pace is the meters travelled in 1 millisecond
  if (distance > 0) {                              
    setSpeed(10, 10); 
    float wait = (distance/Pace);                  //Works out delay time using distance time calculation
    delay(wait);                                   //Robot will drive forward for that time that has just been worked out. This should mena the robot will drive forward for the specified distance
  }
  else if (distance < 0) {
    setSpeed(-10, -10);
    distance = (distance - (2*distance));         //if distance is negative it will turn it into a positive number
    float wait = (distance/Pace);                      
    delay(wait);                                  //robot will drive backwards for that specified distance
  }
  setSpeed(0,0);
}



void TurnAngle(float angle) {
  const float TimeForOneDegree = 17.90;               //Time to turn one degree in milliseconds
  if (angle > 0) {
    float DelayTime = (angle * TimeForOneDegree);      //Calculates time turn inputted angle
    setSpeed(20,-20);                                   //Turns right for calculated time
    delay(DelayTime);
  }
  else if (angle < 0) {                               
    float DelayTime = ((angle - (2*angle)) * TimeForOneDegree);     //If angle entered is negative the robot will turn left
    setSpeed(-20,20);                                               //It converts the negative value entered into a positive in order to do the calculation
    delay(DelayTime);
    }
  setSpeed(0,0);
}



int WorkOutDarkLightLevel() {
  int DarkLightLevel = 0;
  for (int x=1;x<11;x++) {                                
    DarkLightLevel = (DarkLightLevel + LightLevel());                                               //Will work out a value for a dark strip by taking the average reading of the light level
  }
  DarkLightLevel = (DarkLightLevel / 10);
  return DarkLightLevel;
}



int WorkOutWhiteLightLevel() {
  int WhiteLightLevel = 0;
  for (int x=1;x<11;x++) {                                                                         //Will work out an average value for a white stripe
    WhiteLightLevel = (WhiteLightLevel + LightLevel());                                            //Will call the function that reads the light level 10 times, add them all together and divide by 10 to get the average
  }
  WhiteLightLevel = (WhiteLightLevel / 10);
  return WhiteLightLevel;                                                                          //Returns these values
}



int CalibrateDarkValue() {
  Serial.println("Press the left button when the robot is on a dark surface.");                          //Will wait for the user to put robot on dark strip and press the left button
  while (digitalRead(LeftButton) == HIGH) {}
  delay(200);
  while (digitalRead(LeftButton) == LOW) {}
  delay(200);
  int DarkLightLevel = WorkOutDarkLightLevel();
  Serial.print("The light level for a black strip is " );                                            //Will record and store the light level for the dark strip
  Serial.println(DarkLightLevel);
  Serial.println();
  return DarkLightLevel;
}



int CalibrateWhiteValue() {
  Serial.println("Press the left button when the robot is on a white/light surface.");            //Will wait for the user to put the robot on a white patch/strip and press the left button
  while (digitalRead(LeftButton) == HIGH) {}
  delay(200);
  while (digitalRead(LeftButton) == LOW) {}
  delay(200);
  int WhiteLightLevel = WorkOutDarkLightLevel();
  Serial.print("The light level for a white part is " );
  Serial.println(WhiteLightLevel);                                                                //Will record and store the light level for the white strips/areas
  Serial.println();
  return WhiteLightLevel;
}



void LED(int x, int y, int z) {
  digitalWrite(GreenLED, x);
  digitalWrite(YellowLED,y);                                //Turns LEDs on/off depending on parameters entered
  digitalWrite(RedLED, z);
  
}



void TurnLeft() {
  LED(1,0,0);
  TurnAngle(-90);                                        //Function to turn left if 'left' barcode is read
  setSpeed(7,7);
  BarcodeScan();                                           //After the robot has turned it is returned to the state ready to scan for another barcode
}



void TurnRight() {
  LED(0,1,0);
  TurnAngle(90);                                         //Function to turn right if 'right' barcode is read
  setSpeed(7,7);
  BarcodeScan();                                         //Robot ready to scan another barcode again
}



void UTurn() {
  LED(1,1,0);                                            //Function for a U-Turn 
  TurnAngle(180);
  setSpeed(7,7);
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
    LEDFlash();                                        //while the time is under a certain point the robot will perform different 'dance moves' for that period of time
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
    LEDFlash();                                                //Once the final stage has finsished (After 20 seconds) the robot will drive forward and be in a state of searching for a barcode
    if (millis() > fourth_stage) {
      setSpeed(7,7);
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
    LED(1,1,1);                             //Function is called during the robot boogie
    delay(200);
    LED(0,0,0);
}



int LightLevel() {
  int RightValue = analogRead(RightLDR);
  int CentreValue = analogRead(CentreLDR);                                                  //Reads the light values for each LDR and calculates an average for them
  int LeftValue = analogRead(LeftLDR);
  int AverageValue = ((RightValue + CentreValue + LeftValue) / 3);
  return AverageValue;
}



void BarcodeScan() {
  long BarWidths [4] = {};                                                         //Array that will store the values of the barcode that the robot will scan


  int LeftTurnBarcode  [4] = {1,1,0,0};                                                      //Arrays for barcodes. If bar is thick it is assigned a 1, if bar is thin it is assigned a 0
  int RightTurnBarcode [4] = {0,1,0,1};
  int UturnBarcode     [4] = {0,0,1,0};
  int StopBarcode      [4] = {0,1,1,0};
  int BoogieBarcode    [4] = {1,0,0,1};
  const int ThreshForBarWidth  = 900 ;                                             //Threshold for for thin and thick bars (Less than value is a thin bar, more than for thick bar)
  

  delay(2000);

  int SearchForBlackBar = 1;
  int y = 0;

  
  while (SearchForBlackBar == 1) {
    int Brightness = LightLevel();                                                  //Drives forward and waits for a black bar to be spotted
    setSpeed(7,7);
    if (Brightness < LightToDark) {
      SearchForBlackBar = 0;
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
      End = millis();                                                                    //When white is detected it will record the time spent over a black bar and work out the time spent travelling over a black bar
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
  delay(8000);
  
  for (int index = 0; index < 4; index++) {                                              //Compares time over balck bars to the threshhold and converts it into binary
    if (BarWidths[index] < ThreshForBarWidth) {
      BarWidths[index] = 0;                                                              // 0 for think strip (If the time is less than 900 ms)
    }
    else {
      BarWidths[index] = 1;                                                              //1 for thick strip (If the time is greater than 900ms)
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


  int RightTurn = 1;                                                                     //Will do this for all of the stored barcodes
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
    BarcodeScan();                                                                         //if no matching barcode is found it will continue to look for a new one
  }
}



void setup() {
  AttachServo();
  LDRsetup();
  
  
  Serial.begin(9600);


  int DarkValue =   CalibrateDarkValue();                                                //Calibrates the threshholds that the robot will use to defferentiate between light and dark stripes
  int LightValue  = CalibrateWhiteValue();
  int MiddleValue = ((LightValue + DarkValue) / 2);
      DarkToLight = (MiddleValue + 22);
      LightToDark = (MiddleValue - 22);
  
  
  delay(1000);
  BarcodeScan();                                                                              //Calls the main scanning function once callibration is complete


  
}

void loop() {
  
  
     
   

}
