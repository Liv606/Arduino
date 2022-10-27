//biblioteca
#include <QTRSensors.h>

//declarando sensor
QTRSensors qtr;

const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];

//Motor A
#define PWMA 3 
#define IN1  4 // Esquerda Frente
#define IN2  5 // Esquerda traz

//Motor B
#define PWMB 6
#define IN3  7 // direita frente
#define IN4  8 // direita traz

//Variáveis de velocidade
int Vmax = 120;
int Vmin = 0;  
int Vmedia = 96; 

//Variáveis do PID
float Kp = 0.089; //related to the proportional control term; 
              //change the value by trial-and-error (ex: 0.07).
float Ki = 0.0002; //related to the integral control term; 
              //change the value by trial-and-error (ex: 0.0008).
float Kd = 0.089; //related to the derivative control term; 
              //change the value by trial-and-error (ex: 0.6).
int P;
int I;
int D;
int lastError = 0;




void setup() {

  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
  
  // inicializando os pinos do motor A
  pinMode (PWMA, OUTPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);

   // inicializando os pinos do motor B
  pinMode (PWMB, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);

// motores direção
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);

}

void forward_brake(int posa, int posb) {
  //set the appropriate values for aphase and bphase so that the robot goes straight
  digitalWrite(IN2, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(PWMA, posa);
  analogWrite(PWMB, posb);
}

void loop() {

  uint16_t position = qtr.readLineWhite(sensorValues);
  Serial.print("Position");
  Serial.println(position);
  //delay(250);



  int error = 1500 - position; //3500 is the ideal position (the centre)

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P*Kp + I*Ki + D*Kd; //calculate the correction
                                       //needed to be applied to the speed
  
  int motorspeeda = Vmedia + motorspeed;
  int motorspeedb = Vmedia - motorspeed;
  
  if (motorspeeda > Vmax) {
    motorspeeda = Vmax;
  }
  if (motorspeedb > Vmax) {
    motorspeedb = Vmax;
  }
  if (motorspeeda < Vmin) {
    motorspeeda = Vmin;
  }
  if (motorspeedb < Vmin) {
    motorspeedb = Vmin;
  } 
    forward_brake(motorspeeda, motorspeedb);

    Serial.print("VOA");
    Serial.println(motorspeeda);
    Serial.print("VOB:");
    Serial.println(motorspeedb);
}

  
  

  
  /*//Motor A
  //combinação esquerda frente
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);

  //Combinação esquerda traz
   //digitalWrite(IN1,LOW;
   //digitalWrite(IN2,HIGH);

     //combinação direita frente
      //digitalWrite(IN3,HIGH);
      //digitalWrite(IN4,LOW);
    
         //combinação direita traz
      //digitalWrite(IN3,HIGH);
      //digitalWrite(IN4,LOW);
  

    //Definindo velocidade baixa
    analogWrite (PWMA,70);*/

 
