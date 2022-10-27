#include <QTRSensors.h>

#define NUM_SENSORS 8 // number of sensors used
#define TIMEOUT 4 // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN 2 // emitter is controlled by digital pin 13

QTRSensors qtra;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

int IndicatorLED = 13;
//Motor 1
int pinAIN1 = 6; //Direction
int pinAIN2 = 7; //Direction
int pinPWMA = 5; //Speed

//Motor 2
int pinBIN1 = 8; //Direction
int pinBIN2 = 9; //Direction
int pinPWMB = 10; //Speed

//Standby
int pinSTBY = 12;

int position = 0;
int error = 0;
int m1Speed = 0;
int m2Speed = 0;
int motorSpeed = 0;
unsigned long previousMillis = 0; // time keeping function
int BlinkTime = 300; // blink time
int CycleTime = 1; // used to call for a blink toggle
int BlinkCycle = 1; // used to alternate the LED on and off
///////////////////////PID TUNING//////////////
int lastError = 0;
float KP = 0;
float KD = 0;
int M1 = 100; //base motor speeds
int M2 = 100; //base motor speeds
int M1max = 150; //max motor speeds
int M2max = 150; //max motor speeds
int M1min = 0; //min motor speeds
int M2min = 0; //min motor speeds

//Constants to help remember the parameters
static boolean turnCW = 0; //for motorDrive function
static boolean turnCCW = 1; //for motorDrive function
static boolean motor1 = 0; //for motorDrive, motorStop, motorBrake functions
static boolean motor2 = 1; //for motorDrive, motorStop, motorBrake functions

void setup()
{


  qtra.setTypeAnalog();
  qtra.setSensorPins((const uint8_t[]) {A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtra.setEmitterPin(2);

  //Set the PIN Modes
  pinMode(IndicatorLED, OUTPUT);
  digitalWrite(IndicatorLED, HIGH);
  pinMode(pinPWMA, OUTPUT);
  pinMode(pinAIN1, OUTPUT);
  pinMode(pinAIN2, OUTPUT);
  pinMode(pinPWMB, OUTPUT);
  pinMode(pinBIN1, OUTPUT);
  pinMode(pinBIN2, OUTPUT);
  pinMode(pinSTBY, OUTPUT);

  for (int i = 0; i < 400; i++) // make the calibration take about 10 seconds
  {
    TimeCheck();
    delay(5);
    qtra.calibrate(); // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }

}

void loop()
{
  TimeCheck(); //Check to see if it is time to blink the LED

  uint16_t position = qtra.readLineBlack(sensorValues);

  // compute our "error" from the line position
  // error is zero when the middle sensor is over the line,
  error = position - 3500; /// using an 8 sensor array targeting middle of position 4 to 5

  // set the motor speed based on proportional and derivative PID terms
  motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;
  /////////CHANGE THE - OR + BELOW DEPENDING ON HOW YOUR DRIVER WORKS
  m1Speed = M1 - motorSpeed; // M1 and M2 are base motor speeds.
  m2Speed = M2 + motorSpeed;

  if (m1Speed < M1min) //keep speeds to 0 or above
    m1Speed = M1min;
  if (m2Speed < M2min) m2Speed = M2min; if (m1Speed > M1max) //maximum allowed value
    m1Speed = M1max;
  if (m2Speed > M2max) //maximum allowed value
    m2Speed = M2max;
  // set motor speeds using the two motor speed variables above 255

  forward(); // run the motors based on error information
}

//////////////////////FORWARD////////
//Drive both motors CW, full speed
void forward()
{
  TimeCheck();
  motorDrive(motor1, turnCW, m1Speed);
  motorDrive(motor2, turnCW, m2Speed);
}

/* //Apply Brakes, then into Standby
  motorBrake(motor1);
  motorBrake(motor2);
  motorsStandby(); */

void TimeCheck() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > BlinkTime) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    BlinkIt();
  }
}

void motorDrive(boolean motorNumber, boolean motorDirection, int motorSpeed)
{
  /*
    This Drives a specified motor, in a specific direction, at a specified speed:
    - motorNumber: motor1 or motor2 ---> Motor 1 or Motor 2
    - motorDirection: turnCW or turnCCW ---> clockwise or counter-clockwise
    - motorSpeed: 0 to 255 ---> 0 = stop / 255 = fast
  */

  boolean pinIn1; //Relates to AIN1 or BIN1 (depending on the motor number specified)

  //Specify the Direction to turn the motor
  //Clockwise: AIN1/BIN1 = HIGH and AIN2/BIN2 = LOW
  //Counter-Clockwise: AIN1/BIN1 = LOW and AIN2/BIN2 = HIGH
  if (motorDirection == turnCW)
    pinIn1 = HIGH;
  else
    pinIn1 = LOW;

  //Select the motor to turn, and set the direction and the speed
  if (motorNumber == motor1)
  {
    digitalWrite(pinAIN1, pinIn1);
    digitalWrite(pinAIN2, !pinIn1); //This is the opposite of the AIN1
    analogWrite(pinPWMA, motorSpeed);
  }
  else
  {
    digitalWrite(pinBIN1, pinIn1);
    digitalWrite(pinBIN2, !pinIn1); //This is the opposite of the BIN1
    analogWrite(pinPWMB, motorSpeed);
  }

  //Finally , make sure STBY is disabled - pull it HIGH
  digitalWrite(pinSTBY, HIGH);
}

void motorBrake(boolean motorNumber)
{
  /*
    This "Short Brake"s the specified motor, by setting speed to zero
  */

  if (motorNumber == motor1)
    analogWrite(pinPWMA, 0);
  else
    analogWrite(pinPWMB, 0);
}

void motorStop(boolean motorNumber)
{
  /*
    This stops the specified motor by setting both IN pins to LOW
  */
  if (motorNumber == motor1) {
    digitalWrite(pinAIN1, LOW);
    digitalWrite(pinAIN2, LOW);
  }
  else
  {
    digitalWrite(pinBIN1, LOW);
    digitalWrite(pinBIN2, LOW);
  }
}

void BlinkIt() {
  if (BlinkCycle == 1) {
    //Led On
    digitalWrite(IndicatorLED, HIGH);
  }
  if (BlinkCycle == 2) {
    //Led Off
    digitalWrite(IndicatorLED, LOW);
    BlinkCycle = 0; // reset BlinkCycle
  }
  BlinkCycle++; // Set BlinkCycle to second mode
}
