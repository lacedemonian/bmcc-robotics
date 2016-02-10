#include <QTRSensors.h> // used to read the QTR Sensors
#include <LiquidCrystal.h> // used to operate the LCD on the controller
#include <SharpIR.h>

#define NUM_SENSORS   8     // number of sensors used to follow line
#define NUM_SENSORSA  2    // number of sensors used to find cross
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2

QTRSensorsRC qtrrc((unsigned char[]) { A8, A9, A10, A11, A12, A13, A14, A15 }, // eight sensors used to follow a line
  NUM_SENSORS, TIMEOUT, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS]; // array of the eight sensors

int ir_val[NB_SAMPLE];

QTRSensorsRC qtrrcA((unsigned char[]) { 11, A1 }, // two outer sensors used to detect cross-sections (not used in tuning)
  NUM_SENSORSA, TIMEOUT, EMITTER_PIN);

unsigned int sensorValuesA[NUM_SENSORSA]; // array of two outer sensors

int distanceSensor = A4;
int in1 = 4, in3 = 6, in2 = 5, in4 = 7;    // assign IN from H-Bridge to digital pin
int ENA = A5, ENB = A0;    // assign motors to a pmw pin
int spiralMotor = 8;
int LED = 42;  // assigns signal LED to pin 3
int deliverySensor = 35; // assigns digital IR sensor to pin 35 (used to exit phase two of delivery)
int LED2 = 52, LED3 = 47;
int KpIncrement = 34, KiIncrement = 36, KdIncrement = 38;   // assigns increment buttons to pins 34 36 38
int incrementChange = 40; // assigns increment changer to pin 40
int BUTTON = 12; // button used to exit phase one of delivery
int i = 0;

SharpIR sharp(A4, 20150);

int sum = 0;
int pos = 0;    // initialized position for PID 
int proportional = 0;    // initialized proportional for PID
int derivative = 0;    // initialized derivative for PID
int lastproportional = 0;    // initialized last proportional for PID
int integral = 0;    // initialized integral for PID
int const qtrsetpoint = 3500;    // constant setpoint for qtr array PID
int const sharpsetpoint = 1246; // constant setpoint for distance sensor PID

int speedRight;    // variable for the speed of right motor
int speedLeft;    // variable for the speed of left motor
int max_speed = 180;    // defined maximum speed
int error = 0;    // initialized error for PID
float Ki = 0, Kd = 0.2, Kp = 0.05;    // define values for calculatin ERROR on PID
float wKi = 0.0015, wKd = 0.22, wKp = 0.09 ; // define values for calculating error in wall follow

int speedLeftBase = 128;
int speedRightBase = 128;  // base speeds of motors during line follow
int speedLeftBaseWall = 130;
int speedRightBaseWall = 130;  //base speeds of motors wall follow

unsigned long previousMillis = 0;
unsigned long currentMillis = 0; // used for various time functions

const long decisionTimeOut = 1200; // used to time out decision mode (ignores decisions for this amount of milliseconds)

/*VARIOUS BOOLS USED AS FLAGS*/
bool initialDriveBool, lineFollowBool, decisionBool, deliveredStateBool; // used in line follow logic
bool phaseOneBool, phaseTwoBool, phaseThreeBool, phaseTwoTimeOutBool, phaseThreeTimeOutBool; // used for delivery
bool buttonStateBool; // button state in delivery
bool tripWireBool; // detects if ring has passed onto chute (if true, stops phase two)
bool forwardBool; // determines if motors should drive forward or in reverse

float increment[] = { 0.1, 0.01, 0.001, -0.1, -0.01, -0.001 }; // array housing increment values
char mapChar[] = { 'R', 'F', 'R', 'R', 'L', 'L', 'R', 'R', 'R', 'L', 'R', 'R', 'F', 'R', 'F', 'F', 'R' }; // map of decisive turns
byte mapCount = 0; // used to determine location
bool mapDelivery[] = { true, false, true, true, false, false, true, true, false, false, true, true, false, true, false, true }; // whether or not there is a delivery to make

void setup()
{
  decisionBool = true; // can make a decision
  initialDriveBool = false; // the initial drive will be the first function called
  lineFollowBool = true; // whether or not robot should line follow
  deliveredStateBool = false; // whether or not delivery was made in mapDelivery[mapCount]
  forwardBool = true;

  phaseOneBool = false; // used to determine phases of delivery (forward, drop, reverse)
  phaseTwoBool = false;
  phaseThreeBool = false;
  buttonStateBool = false;
  phaseTwoTimeOutBool = false;
  phaseThreeTimeOutBool = false;
  tripWireBool = false;


  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ENA, OUTPUT); // pwm or analog
  pinMode(ENB, OUTPUT); // pwm or analog
  pinMode(LED, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  
  pinMode(distanceSensor, INPUT);
  pinMode(KpIncrement, INPUT);
  pinMode(KiIncrement, INPUT); // Buttons on controller are INPUTS (only used in tuning)
  pinMode(KdIncrement, INPUT);
  pinMode(incrementChange, INPUT);
  //lcd.begin(16, 2);

  delay(500);


   //USED TO CALIBRATE QTR SENSORS. TEMPORARY UNTIL WE FIND A WAY TO EITHER CALIBRATE DURING initD OR CREATE CUSTOM VALUES 
  
  digitalWrite(LED, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 140; i++)  // make the calibration take about 4 seconds (change i for more or less)
  {
    qtrrc.calibrate(); //used to calibrate 8 sensors
    qtrrcA.calibrate(); // used to calibrate outer sensors (not used in pid tuning)
    delay(20);
  }
  digitalWrite(LED, LOW);     // turn off Arduino's LED to indicate we are through with calibration
  delay(2000); // wait 2 seconds
  
  Serial.begin(9600);
  previousMillis = millis();
}

void loop()
{
  while (lineFollowBool && !initialDriveBool)
    lineFollow();
  while (initialDriveBool && !lineFollowBool)
    initD();
}

void delivery()
{
  while (phaseOneBool)
  {
    digitalWrite(LED2,HIGH);
    calculateDistanceError();
    assignMotorSpeedWall(error, forwardBool);

    if (digitalRead(BUTTON) == LOW)
      buttonStateBool = true;

    if (buttonStateBool)
    {
      buttonStateBool = false;
      phaseTwoBool = true;
      phaseOneBool = false;
      delay(200);
      previousMillis = millis();
    }
  }
  delay(200);

  while (phaseTwoBool) // spins delivery motor until sensor picks up a ring has passed or time has passed
  {
    digitalWrite(LED2, LOW);
    digitalWrite(LED,HIGH);
    digitalWrite(spiralMotor, HIGH);;
    if (digitalRead(deliverySensor) == HIGH)
    {
      tripWireBool = true;
    }
    currentMillis = millis();
    if (currentMillis - previousMillis >= 3000)
    {
      phaseThreeTimeOutBool = true;
      phaseTwoBool = false;
      phaseThreeBool = true;
    }

    if (tripWireBool)
    {
      phaseThreeBool = true;
      phaseTwoBool = false;
      phaseThreeTimeOutBool = true;
      delay(200);
    }
  }
  delay(200);
  
  while (phaseThreeBool)
  {
    digitalWrite(spiralMotor, LOW);
    digitalWrite(LED, LOW);
    digitalWrite(LED2, HIGH);
    if (phaseThreeTimeOutBool)
    {
      previousMillis = millis();
      phaseThreeTimeOutBool = false;
    }
    currentMillis = millis();

    calculateDistanceError();
    assignMotorSpeedWall(error, !forwardBool);

    if (currentMillis - previousMillis >= 3000)
    {
      phaseThreeBool = false;
      deliveredStateBool = true;
    }
  }
  resetPID();
}

void lineFollow() //main line follow code
{
  //stats(); //stats will print out the Kp, Ki, Kd, and increment values
  digitalWrite(LED3, HIGH);
  calculateQTRError(); //stores error value in error
  assignMotorSpeedQTR(error);
  currentMillis = millis();

  qtrrcA.readCalibrated(sensorValuesA); //read input of outer sensors (0-1000) stores in sensorValuesA

  if (digitalRead(KpIncrement) == HIGH)
    Kp += increment[i];
  if (digitalRead(KiIncrement) == HIGH) //conditionals are used to increment Kp,Kd,Ki according to button pressed
    Ki += increment[i];                       // value changes according to 'i'
  if (digitalRead(KdIncrement) == HIGH)
    Kd += increment[i];

  if (digitalRead(incrementChange) == HIGH)                                   // used to change the incrementing value
  {
    if (i == 5) //once i becomes 5 (array location 6)
      i = 0; //return i to beginning of array (location 1)
    else
      i++; //otherwise increment i
  }
  if (currentMillis - previousMillis >= decisionTimeOut)
  {

    decisionBool = true;
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
  }
  /*
  if ((sensorValuesA[0] > 700 || sensorValuesA[1] > 700) && decisionBool)
  {
    resetPID();
    decision();
  }
  */
}

void stats() //used to display values on an LCD
{
  /*
  lcd.setCursor(0, 0); lcd.print("hello:");
  lcd.print(Kp, DEC);
  lcd.setCursor(7, 0); lcd.print("  d:"); lcd.print(Kd, DEC);
  lcd.setCursor(0, 1); lcd.print("i:"); lcd.print(Ki, DEC);
  lcd.setCursor(7, 1); lcd.print(" n:");
  switch (i)
  {
  case 0: lcd.print("0.1"); break;
  case 1: lcd.print("0.01"); break;
  case 2: lcd.print("0.001"); break;
  case 3: lcd.print("-0.1"); break;
  case 4: lcd.print("-0.01"); break;
  case 5: lcd.print("-0.001"); break;
  default:;
  }
  */
}

void sort(int a[], int arraySize)
{
  for(int i=0; i<(arraySize-1); i++) {
        bool flag = true;
        for(int o=0; o<(arraySize-(i+1)); o++) {
            if(a[o] > a[o+1]) {
                int t = a[o];
                a[o] = a[o+1];
                a[o+1] = t;
                flag = false;
            }
        }
        if (flag) break;
}
}

void initD()
{
  qtrrc.readCalibrated(sensorValues);
  calculateDistanceError();
  assignMotorSpeedWall(error, forwardBool);
  currentMillis = millis();
  
  Serial.print("dist: ");
  Serial.print(pos);
  Serial.print(" proportion: ");
  Serial.print(proportional);
  Serial.print(" error: ");
  Serial.println(error);
  
  
  
  //qtrrc.calibrate();
  //qtrrcA.calibrate();
  
  
  if (currentMillis - previousMillis >= 2000)
  {
    initialDriveBool = false;
    lineFollowBool = true;
  }
  
  
}

void calculateQTRError()
{
  pos = qtrrc.readLine(sensorValues); //readLine function returns the position of the robot on the line
  proportional = pos - qtrsetpoint; //proportional is the position with respect to the two most inner sensors (use values ~2000-3000)
  integral = integral + proportional; //integral adds the proportional to itself every time (area under the curve) as the position oscillates around zero, the integral will balance itself out
  derivative = proportional - lastproportional; //  derivative will take the slope of the oscillation per cycle
  error = (proportional*Kp + integral*Ki + derivative*Kd); //error is computed by the sum of the following pid values, each with their own constants
  lastproportional = proportional; //records the current positon for the next cycle
}

void calculateDistanceError()
{
  for (int i=0; i<NB_SAMPLE; i++){
        ir_val[i] = analogRead(A4);
    }
  sort(ir_val, NB_SAMPLE);
  pos = map(ir_val[NB_SAMPLE / 2], 0, 1023, 0, 5000);
  proportional = pos - sharpsetpoint; //proportional is the position with respect to the two most inner sensors (use values ~2000-3000)
  integral = integral + proportional; //integral adds the proportional to itself every time (area under the curve) as the position oscillates around zero, the integral will balance itself out
  derivative = proportional - lastproportional; //  derivative will take the slope of the oscillation per cycle
  error = (proportional*wKp + integral*wKi + derivative*wKd); //error is computed by the sum of the following pid values, each with their own constants
  lastproportional = proportional; //records the current positon for the next cycle
}

void resetPID()
{
  pos = 0;
  proportional = 0;
  derivative = 0;
  lastproportional = 0;
  integral = 0;
  speedLeft = speedLeftBase;
  speedRight = speedRightBase;
}

void assignMotorSpeedQTR(int e)
{
  speedLeft = speedLeftBase - e; //left motor speed relative to base speed minus error
  speedRight = speedRightBase + e; //right motor speed relative to base speed plus error


  if (speedRight>max_speed) //used to cap off motor at max speed
    speedRight = max_speed;
  if (speedLeft > max_speed)
    speedLeft = max_speed;

  if (speedRight< 0) speedRight = 0; //keep speed from going negative
  if (speedLeft < 0) speedLeft = 0;

  digitalWrite(in1, HIGH); //in1-in4 are used to determine direction of motor spin and depends on the wiring
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  analogWrite(ENA, speedRight); //finally the motor speeds are assigned to the motors through the H-Bridge
  analogWrite(ENB, speedLeft);
}

void assignMotorSpeedWall(int e, bool f)
{
  if (f) //forward
  {
    speedLeft = speedLeftBaseWall + e; //left motor speed relative to base speed minus error
    speedRight = speedRightBaseWall - e; //right motor speed relative to base speed plus error

    if (speedRight>max_speed) //used to cap off motor at max speed
      speedRight = max_speed;
    if (speedLeft > max_speed)
      speedLeft = max_speed;

    if (speedRight< 0) speedRight = 0; //keep speed from going negative
    if (speedLeft < 0) speedLeft = 0;

    digitalWrite(in1, HIGH); //in1-in4 are used to determine direction of motor spin and depends on the wiring
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    analogWrite(ENA, speedRight); //finally the motor speeds are assigned to the motors through the H-Bridge
    analogWrite(ENB, speedLeft);
  }
  else //reverse
  {
    speedLeft = speedLeftBaseWall + e; //left motor speed relative to base speed minus error
    speedRight = speedRightBaseWall - e; //right motor speed relative to base speed plus error

    if (speedRight>max_speed) //used to cap off motor at max speed
      speedRight = max_speed;
    if (speedLeft > max_speed)
      speedLeft = max_speed;

    if (speedRight< 0) speedRight = 0; //keep speed from going negative
    if (speedLeft < 0) speedLeft = 0;

    digitalWrite(in1, LOW); //in1-in4 are used to determine direction of motor spin and depends on the wiring
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    analogWrite(ENA, speedRight); //finally the motor speeds are assigned to the motors through the H-Bridge
    analogWrite(ENB, speedLeft);
  }
}

void decision()
{
  if (!mapDelivery[mapCount] || deliveredStateBool)
  {
    if (mapChar[mapCount] == 'R')
    {
      digitalWrite(LED, HIGH);
      analogWrite(ENA, 0); //finally the motor speeds are assigned to the motors through the H-Bridge
      analogWrite(ENB, speedLeftBase);
      delay(500);
      digitalWrite(LED, LOW);
      decisionBool = false;
      previousMillis = currentMillis;
      digitalWrite(LED2, HIGH);
      deliveredStateBool = false;
      if (mapCount == 16)
        mapCount = 0;
      else
        ++mapCount;
      deliveredStateBool = false;
    }
    else if (mapChar[mapCount] == 'L')
    {
      digitalWrite(LED, HIGH);
      analogWrite(ENA, speedRightBase + 60); //finally the motor speeds are assigned to the motors through the H-Bridge
      analogWrite(ENB, 0);
      delay(500);
      resetPID();
      digitalWrite(LED, LOW);
      decisionBool = false;
      previousMillis = currentMillis;
      digitalWrite(LED2, HIGH);
      deliveredStateBool = false;
      if (mapCount == 16)
        mapCount = 0;
      else
        ++mapCount;
      deliveredStateBool = false;
    }
    else if (mapChar[mapCount] == 'F')
    {
      digitalWrite(LED, HIGH);
      analogWrite(ENA, 0); //finally the motor speeds are assigned to the motors through the H-Bridge
      analogWrite(ENB, 0);
      delay(200);
      digitalWrite(LED, LOW);
      decisionBool = false;
      previousMillis = currentMillis;
      digitalWrite(LED3, HIGH);
      deliveredStateBool = false;
      if (mapCount == 16)
        mapCount = 0;
      else
        ++mapCount;
      deliveredStateBool = false;
    }
  }

  else
  {
    phaseOneBool = true;
    delivery();
  }
}
