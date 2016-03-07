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

int ir_val[NB_SAMPLE]; // Array used by IR Distance Sensor

void setup() {
  // put your setup code here, to run once:
  pinMode(A2, INPUT);
  digitalWrite(44, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 140; i++)  // make the calibration take about 4 seconds (change i for more or less)
  {
    qtrrc.calibrate(); //used to calibrate 8 sensors
    qtrrcA.calibrate(); // used to calibrate outer sensors (not used in pid tuning)
    delay(20);
  }
  digitalWrite(44, LOW);     
  delay(500); // wait 2 seconds
  
  qtrrc.customCalibration();
  qtrrcA.customCalibration();
  Serial.begin(9600);
}

void loop() {
  
  qtrrcA.readCalibrated(sensorValuesA);            
  qtrrc.readCalibrated(sensorValues);
  
  Serial.print(sensorValuesA[0]);
  Serial.print(" ");
  for(int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print(" ");
  }
  Serial.println(sensorValuesA[1]);
  delay(10);
  
  /*
  Serial.print("QTR_ERROR: ");
  Serial.print(qtrrc.readLine(sensorValues));
  Serial.print("DIST_RAW: ");
  for (int i=0; i<NB_SAMPLE; i++)
  {
    ir_val[i] = analogRead(A2);
  }
  sort(ir_val, NB_SAMPLE);
  int pos = map(ir_val[NB_SAMPLE / 2], 0, 1023, 0, 5000);
  Serial.print(pos);
  Serial.print(" ");
  Serial.print("DIST_PRO: ");
  Serial.println(pos - 0);
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
