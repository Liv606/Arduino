#include <QTRSensors.h> //biblioteca do sensor

QTRSensors qtr;    //variável do sensor

const uint8_t SensorCount = 8; //Declarando o número de sensores
uint16_t sensorValues[SensorCount];

//Definindo motores
int IN1 = 5;
int IN2 = 6;
int IN3 = 9;
int IN4 = 10;

//inicializa pinos
void setup(){

  Serial.begin(9600);
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0,A1,A2,A3,A4,A5,A6,A7}, SensorCount);  // Declarando os sensores (onde estão ligados no arduino) 
  qtr.setEmitterPin(2);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); //O LED do arduino liga para mostrar que está calibrando

  Serial.println("calibrando..."); //se abrir o serial monitor, aparecerá "calibrando"
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // O LED do arduino desliga para mostrar que a calibração terminou

  // Vai printar o menor valor da calibração

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);   
    Serial.print(' ');
  }
  Serial.println();

   // Vai printar o maior valor da calibração
   
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  
  //Define os pinos como saida
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
}

void loop(){

  // Nessa parte será printado as posições dos 8 sensores em conjunto, variando de 0 a 5000 --> 2500 provavelmente significa o meio 
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);

  Serial.print("Possicao lida ");
  Serial.print(position);
  Serial.println();

  //Inicia motores
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);


}
