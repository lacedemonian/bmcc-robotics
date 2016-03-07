#include <QTRSensors.h> // used to read the QTR Sensors
#include <PololuWheelEncoders.h>

#define NUM_SENSORS   8     // number of sensors used to follow line
#define NUM_SENSORSA  2    // number of sensors used to find cross
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2
#define NB_SAMPLE 25

/*OBJECTS AND ARRAYS*/
QTRSensorsRC qtrrc((unsigned char[]) { A8, A9, A10, A11, A12, A13, A14, A15 }, // eight sensors used to follow a line
  NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS]; // array of the eight sensors

QTRSensorsRC qtrrcA((unsigned char[]) { A3, A4 }, // two outer sensors used to detect cross-sections (not used in tuning)
  NUM_SENSORSA, TIMEOUT, EMITTER_PIN);
unsigned int sensorValuesA[NUM_SENSORSA]; // array of two outer sensors

PololuWheelEncoders encoders;
int ir_val[NB_SAMPLE]; // Array used by IR Distance Sensor

/*PINS AND COMPONENTS*/
int IN1 = 6, IN2 = 7, IN3 = 8, IN4 = 9, ENA = A1, ENB = A0; // H-Bride Pins
int LED = 43, LED2 = 44, LED3 = 45; // LED Pins
int SERVO = A5, BUTTON = 43, DELIVERY_SENSOR = 35; // Servo, Front Button, Delivery Sensor
int DISTANCE_SENSOR = A2;
int WHEEL_ENCODER_L1 = 10, WHEEL_ENCODER_L2 = 11; // Left Wheel Encoders
int WHEEL_ENCODER_R1 = 12, WHEEL_ENCODER_R2 = 13; // Right Wheel Encoders

/*PROPORTIONAL-INTEGRAL-DERIVATIVE INITIALIZATION*/
//CHANGE QTR SET POINT AND SHARP SET POINT AS REQUIRED
int sum = 0;
int pos = 0;    // initialized position for PID 
int proportional = 0;    // initialized proportional for PID
int derivative = 0;    // initialized derivative for PID
int lastproportional = 0;    // initialized last proportional for PID
int integral = 0;    // initialized integral for PID
int const qtrsetpoint = 3500;    // constant setpoint for qtr array PID
int const sharpsetpoint = 1148; // constant setpoint for distance sensor PID

/*PROPORTIONAL-INTEGRAL-DERIVATIVE SET UP*/
int max_speed = 200;    // defined maximum speed
int error = 0;    // initialized error for PID
float Ki = 0, Kd = 0.2, Kp = 0.05;    // Line Follow Tuning
float wKi = 0, wKd = 0, wKp = 0 ; // Wall Follow Tuning

/*DC MOTOR PROPERTIES*/
int speedRight;    
int speedLeft;
int speedLeftBase = 128; // Line Follow Base Speed
int speedRightBase = 128; 
int speedLeftBaseWall = 132; //Wall Follow Base Speed
int speedRightBaseWall = 132; 

/*PRECISE TURN VARIABLES*/
float wheelRadius = 0.83;
float wheelToWheel = 5;
float totalRotation;
float wantRotation;

/*TIME VARIABLES*/
unsigned long previousMillis = 0;
unsigned long currentMillis = 0; // Used for millis function
const long decisionTimeOut = 1000; // Amount of time to ignore decision-mode


/*VARIOUS BOOLS USED AS FLAGS*/
bool initialDriveBool, lineFollowBool, decisionBool, deliveredStateBool; // Line Follow Flags
bool tripWireBool, buttonStateBool, phaseOneBool, phaseTwoBool, phaseThreeBool, phaseTwoTimeOutBool, phaseThreeTimeOutBool; // Delivery Flags

bool forwardBool; // determines if motors should drive forward or in reverse



/*MAP OF COURSE*/
char mapChar[] = { 'R', 'F', 'R', 'R', 'L', 'L', 'R', 'R', 'F', 'R', 'S', 'R', 'F', 'R', 'F', 'F', 'R' }; // map of decisive turns
byte mapCount = 0; // used to determine location
bool mapDelivery[] = { true, false, true, true, false, false, true, true, false, false, true, true, false, true, false, true }; // whether or not there is a delivery to make

void linearSingle(float distance, bool right, bool forward)
// e.g (DISTANCE, TRUE, TRUE) will rotate right wheel forward for that distance
{
  if(right)
  {
    if(forward)
    {
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    }
    else
     {
       digitalWrite(IN3, LOW);
       digitalWrite(IN4, HIGH);
     }
    analogWrite(ENB, 200);
  }
  else
  {
    if(forward)
    {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    }
    else
    {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
    }
    analogWrite(ENA, 200);
  }
  totalRotation = 0;
  wantRotation = (distance/(6.28318*wheelRadius))*360;

  while(abs(totalRotation) < wantRotation)
  {
    if(right&&forward)
      totalRotation = 7.5*abs(encoders.getCountsM1());
    else if(!right&&forward)
      totalRotation = 7.5*abs(encoders.getCountsM2());
    else if(right&&!forward)
      totalRotation = 7.5*abs(encoders.getCountsM1());
    else
      totalRotation = 7.5*abs(encoders.getCountsM2());
  }
  analogWrite(ENA, 0);
totalRotation=encoders.getCountsAndResetM2();
totalRotation=encoders.getCountsAndResetM1();
}

void linearDouble(float distance, bool forward)
// e.g. (DISTANCE, TRUE) Drives it straight ahead
{
  totalRotation = encoders.getCountsAndResetM2();
  if(forward)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  analogWrite(ENA, 210);
  analogWrite(ENB, 210);
  totalRotation = 0;
  wantRotation = (distance/(6.28318*wheelRadius))*360;
  while(totalRotation < wantRotation)
  {
    //if(forward)
      totalRotation = 7.5*abs(encoders.getCountsM2());
    //else
     // totalRotation = 7.5*abs(encoders.getCountsM1());
  }
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  totalRotation=encoders.getCountsAndResetM2();
}

void pointTurn(float rotation, bool clockwise)
// e.g (360, TRUE) does one FULL rotation, clockwise)

{
  totalRotation = encoders.getCountsAndResetM2();
  if(clockwise)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  analogWrite(ENA, 180);
  analogWrite(ENB, 180);
  totalRotation = 0;
  wantRotation = ((wheelToWheel/1.66)*rotation);
  while(totalRotation < wantRotation)
  {
      //if(clockwise)
      totalRotation = 7.5*abs(encoders.getCountsM2());
      //else
      //totalRotation = 7.5*abs(encoders.getCountsM2());
  }
  totalRotation=encoders.getCountsAndResetM2();
  totalRotation=encoders.getCountsAndResetM1();
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void setup()
{
  /*CHANGE THESE FOR TESTING PURPOSES*/
  decisionBool = true; // Can we enter decision-mode?
  lineFollowBool = false; // whether or not robot should line follow
  deliveredStateBool = false; // whether or not delivery was made in mapDelivery[mapCount]

  /*INITIALIZE ALL DELIVERY BOOLS AS FALSE*/
  phaseOneBool = false;
  phaseOneBool=phaseTwoBool=phaseThreeBool=buttonStateBool=phaseTwoTimeOutBool=phaseThreeTimeOutBool=tripWireBool;

  /*INITIALIZE ALL PINS*/
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(WHEEL_ENCODER_L1, INPUT);
  pinMode(WHEEL_ENCODER_L2, INPUT);
  pinMode(WHEEL_ENCODER_R1, INPUT);
  pinMode(WHEEL_ENCODER_R2, INPUT);
  pinMode(DISTANCE_SENSOR, INPUT);

  encoders.init(WHEEL_ENCODER_L1,WHEEL_ENCODER_L2,WHEEL_ENCODER_R1,WHEEL_ENCODER_R2);
  
  delay(200);

  /*RUNS CALIBRATION ONCE, THEN USES CUSTOM VALUES FOR SENSORS*/
  digitalWrite(LED, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 140; i++)  // make the calibration take about 4 seconds (change i for more or less)
  {
    qtrrc.calibrate(); //used to calibrate 8 sensors
    qtrrcA.calibrate(); // used to calibrate outer sensors (not used in pid tuning)
    delay(20);
  }
  digitalWrite(LED, LOW);     
  delay(500); // wait 2 seconds
  
  //qtrrc.customCalibration();
  //qtrrcA.customCalibration();
  
  previousMillis = millis();
}

void loop()
{
  while (lineFollowBool)
    lineFollow();
  while(!lineFollowBool)
    initD();
}

void delivery()
{
  while (phaseOneBool)
  {
    digitalWrite(LED2,HIGH);
    calculateDistanceError();
    assignMotorSpeedWall(error, forwardBool);

    if (digitalRead(BUTTON) == HIGH)
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
    digitalWrite(SERVO, HIGH);;
    if (digitalRead(DELIVERY_SENSOR) == HIGH)
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
    digitalWrite(SERVO, LOW);
    digitalWrite(LED, LOW);
    digitalWrite(LED2, HIGH);
    if (phaseThreeTimeOutBool)
    {
      previousMillis = millis();
      phaseThreeTimeOutBool = false;
    }
    linearDouble(3, false);
    pointTurn(90, true);
    ++mapCount;
    phaseThreeBool = false;;
  }
  resetPID();
}

void lineFollow() //main line follow code
{
  digitalWrite(LED3, HIGH);
  calculateQTRError(); //stores error value in error
  assignMotorSpeedQTR(error);
  currentMillis = millis();

  qtrrcA.readCalibrated(sensorValuesA); //read input of outer sensors (0-1000) stores in sensorValuesA

  if (currentMillis - previousMillis >= decisionTimeOut)
  {
    decisionBool = true;
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
  }
  /*
  if ((sensorValuesA[1] > 700) && decisionBool)
  {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    delay(500);
    totalRotation = encoders.getCountsAndResetM2();
    resetPID();
    decision();
  }
  */
  
  
  if ((sensorValuesA[0] > 700 || sensorValuesA[1] > 700) && decisionBool)
  {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    delay(500);
    totalRotation = encoders.getCountsAndResetM2();
    resetPID();
    decision();
  }
  
  
  
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
  /*
  qtrrc.readCalibrated(sensorValues);
  calculateDistanceError();
  assignMotorSpeedWall(error, !forwardBool);
  currentMillis = millis();
  */
  /*
  if (currentMillis - previousMillis >= 2000)
  {
    initialDriveBool = false;
    lineFollowBool = true;
  }
  */
  
  totalRotation = encoders.getCountsAndResetM2();
  linearDouble(6, true);
  lineFollowBool = true;
  decisionBool = true;
  delay(10);
}

void calculateQTRError()
{
  pos = qtrrc.readLine(sensorValues); //readLine function returns the position of the robot on the line
  proportional = pos - qtrsetpoint; //proportional is the position with respect to the two most inner sensors (use values ~2000-3000)
  integral += integral + proportional; //integral adds the proportional to itself every time (area under the curve) as the position oscillates around zero, the integral will balance itself out
  derivative = proportional - lastproportional; //  derivative will take the slope of the oscillation per cycle
  error = (proportional*Kp + integral*Ki + derivative*Kd); //error is computed by the sum of the following pid values, each with their own constants
  lastproportional = proportional; //records the current positon for the next cycle
}

void calculateDistanceError()
{
  for (int i=0; i<NB_SAMPLE; i++)
  {
    ir_val[i] = analogRead(DISTANCE_SENSOR);
  }
  sort(ir_val, NB_SAMPLE);
  pos = map(ir_val[NB_SAMPLE / 2], 0, 1023, 0, 5000);
  proportional = pos - sharpsetpoint; //proportional is the position with respect to the two most inner sensors (use values ~2000-3000)
  integral += integral + proportional; //integral adds the proportional to itself every time (area under the curve) as the position oscillates around zero, the integral will balance itself out
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

  digitalWrite(IN1, HIGH); //in1-in4 are used to determine direction of motor spin and depends on the wiring
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

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

    digitalWrite(IN1, HIGH); //in1-in4 are used to determine direction of motor spin and depends on the wiring
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

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

    digitalWrite(IN1, LOW); //in1-in4 are used to determine direction of motor spin and depends on the wiring
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    analogWrite(ENA, speedRight); //finally the motor speeds are assigned to the motors through the H-Bridge
    analogWrite(ENB, speedLeft);
  }
}

void decision()
{
  //if (!mapDelivery[mapCount])
  //{
    if (mapChar[mapCount] == 'R')
    {
      digitalWrite(LED, HIGH);
      resetPID();
      decisionBool = false;
      linearDouble(2, true);
      pointTurn(110, true);
      delay(100);
      linearDouble(4, true);
      digitalWrite(LED, LOW);
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
      resetPID();
      linearDouble(3, true);
      decisionBool = false;
      pointTurn(90, false);
      delay(100);
      digitalWrite(LED, LOW);
      linearDouble(4, true);
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
    else
    {
      linearDouble(5, true);
      linearSingle(3, false, true);
      digitalWrite(LED, HIGH);
      digitalWrite(LED2, HIGH);
      digitalWrite(LED3, HIGH);
      pointTurn(780, true);
    }
  //}
/*
  else
  {
    phaseOneBool = true;
    delivery();
  } */
}
