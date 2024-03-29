#include <QTRSensors.h> // used to read the QTR Sensors
#include <LiquidCrystal.h> // used to operate the LCD on the controller

#define NUM_SENSORS   6     // number of sensors used to follow line
#define NUM_SENSORSA  2    // number of sensors used to find cross
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2

QTRSensorsRC qtrrc((unsigned char[]) {14, 15, 16, 17, 18, 19}, // six middle sensors used to follow a line
  NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
  
unsigned int sensorValues[NUM_SENSORS]; // array of the six middle sensors

LiquidCrystal lcd(,,,,,); // lcd object to display pid values

QTRSensorsRC qtrrcA((unsigned char[]) {5,6}, // two outer sensors used to detect cross-sections (not used in tuning)
  NUM_SENSORSA, TIMEOUT, EMITTER_PIN);
unsigned int sensorValuesA[NUM_SENSORSA];
 
int in1=4, in3=12, in2=7, in4=13;    // assign IN from H-Bridge to digital pin
int ENA=9, ENB=10;    // assign motors to a pmw pin
int LED=3;	// assigns signal LED to pin 3
int KpIncrement = 2, KiIncrement = 13; KdIncrement = 5;		// assigns increment buttons to pins 2,13,5 (5 is used here twice, use bigger board)
//int incrementChange = PLACEHOLDER // assigns increment changer to pin *

int pos = 0;    // initialized position for PID 
long proportional=0;    // initialized proportional for PID
long derivative=0;    // initialized derivative for PID
long lastproportional=0;    // initialized last proportional for PID
long integral=0;    // initialized integral for PID
int const setpoint= 2000;    // constant setpoint for PID
int speedRight;    // variable for the speed of right motor
int speedLeft;    // variable for the speed of left motor
int max_speed= 150;    // defined maximum speed
int error=0;    // initialized error for PID
float Ki=0, Kd=0, Kp=0;    // define values for calculatin ERROR on PID
int speedLeftBase = 50;  	//
int speedRightBase = 50;	// base speeds of left and right motors

float[] increment = {0.1, 0.01, 0.001, -0.1, -0.01, -0.001}; // array housing increment values

void setup()
{
  pidFollowTrue = true;
  
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ENA, OUTPUT); // pwm or analog
  pinMode(ENB, OUTPUT);	// pwm or analog
  pinMode(LED, OUTPUT);

  pinMode(KpIncrement, INPUT);
  pinMode(KiIncrement, INPUT);		// Buttons on controller are INPUTS
  pinMode(KdIncrement, INPUT);
	pinMode(incrementChange, INPUT);

  delay(500);
  
  digitalWrite(LED, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 140; i++)  // make the calibration take about 4 seconds (change i for more or less)
  {
    qtrrc.calibrate();       // used to calibrate inner 6 sensors
    //qtrrcA.calibrate();			// used to calibrate outer sensors (not used in pid tuning)
    delay(20);
  }
  digitalWrite(LED, LOW);     // turn off Arduino's LED to indicate we are through with calibration
  delay(2000); // wait 2 seconds
}

void loop()
{
  test();
}

void test()			//main line follow code
{ 											
  stats();                                          // prints values on  LCD
  unsigned int sensorValuesA[2]; 										// 	array of outer sensors
  qtrrcA.readCalibrated(sensorValuesA);							//	read input of outer sensors (0-1000)
  unsigned int sensorValues[6];												// 	array of inner sensors
  qtrrc.readCalibrated(sensorValues);                 //  read inputs of inner sensors (0-1000) [calibrated]
  pos = qtrrc.readLine(sensorValues);									//	readLine function returns the position of the robot on the line
  proportional= pos - setpoint;												//	proportional is the position with respect to the two most inner sensors (use values ~2000-3000)
  integral=integral+proportional;											//	integral adds the proportional to itself every time (area under the curve) as the position oscillates around zero, the integral will balance itself out
  derivative=proportional-lastproportional;						//	derivative will take the slope of the oscillation per cycle
  error=(proportional*Kp+integral*Ki+derivative*Kd);	//	error is computed by the sum of the following pid values, each with their own constants
  lastproportional=proportional;											//	records the current positon for the next cycle
  
  if(digitalRead(KpIncrement) == HIGH)
    Kp += increment[i];
  if(digitalRead(KiIncrement) == HIGH)				// conditionals are used to increment Kp,Kd,Ki according to button pressed
    Ki += increment[i];												// value changes according to 'i'
  if(digitalRead(KdIncrement) == HIGH)
    Kd += increment[i];
  
  if(input)																		// used to change the incrementing value
  {
    if(i==5)			// once i becomes 5 (array location 6)
      i = 0;			// return i to beginning of array (location 1)
    else 
      i++;				// otherwise increment i
  }
  
  
  
  Serial.print(qtrrcA.calibratedMinimumOn[0]);        // prints calibrated minimum ( close to 0)
  Serial.print(" ");
  for(int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(" ");
  }
  Serial.print(qtrrcA.calibratedMinimumOn[1]);
  Serial.print(" ");
  Serial.print("\t");
  
  Serial.print(qtrrcA.calibratedMaximumOn[0]);        //  prints calibrated maximum ( usually 1000)
  Serial.print(" ");
  for(int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(" ");
  }
  Serial.print(qtrrcA.calibratedMaximumOn[1]);
  Serial.print(" ");
  Serial.print("\t");

  Serial.print(sensorValuesA[0]);                     // prints current reading ( 0 - 1000 )
  Serial.print(" ");
  for(int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print(" ");
  }
  Serial.print(sensorValuesA[1]);
  
  Serial.print(" Error: ");                           // finally prints error value
  Serial.print(error);
  Serial.println();
  delay(300);                                         // waits 300 milliseconds
}

void stats()															// used to display values on an LCD
{
  lcd.setCursor(0,0);
  lcd.print("p:"); lcd.print(Kp); lcd.print(" ");
  lcd.print("d:"); lcd.print(Kd); lcd.setCursor(0,1);
  lcd.print("i:"); lcd.print(Ki); lcd.print(" ");
  lcd.print(increment[i]);
}
