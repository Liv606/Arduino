//biblioteca
#include <QTRSensors.h>

//declarando sensor
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

//Motor A
int PWMA = 3; 
int IN1 = 4; // Esquerda Frente
int IN2 = 5; // Esquerda traz

//Motor B
int PWMB = 6;
int IN3 = 7; // direita frente
int IN4 = 8; // direita traz

//Variáveis de velocidade
#define Vmax 254
#define Vmin 0
#define Vmedia 254

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

}

void loop() {

  uint16_t position = qtr.readLineBlack(sensorValues);
  
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);
  //delay(250);

  //andar para frente
if(position > 2700 && position < 3700){
  
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  analogWrite (PWMA,Vmedia);

  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  analogWrite (PWMB,Vmedia);

  Serial.println("Indo reto");
  
} else if(position >= 3700){ //andar para a esquerda

  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  analogWrite (PWMA,Vmax);

  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  analogWrite (PWMB,Vmax);

  Serial.println("Virando esquerda");
   
} else if(position <= 2700){ //andar para a direita

  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  analogWrite (PWMA,Vmax);

  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  analogWrite (PWMB,Vmax);

  Serial.println("Virando direita");
  
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
}
 
