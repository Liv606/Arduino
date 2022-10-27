#include <QTRSensors.h> //biblioteca do sensor


const uint8_t SensorCount = 8; //Declarando o número de sensores
uint16_t sensorValues[SensorCount];

  #define pinMot1A 5   //ESQUERDA - TRAZ
  #define MotorEsquerdaFrente 6   //ESQUERDA - FRENTE
  #define MotorDireitaFrente 9   //DIREITA - FRENTE
  #define pinMot2B 10  //DIREITA - TRAZ

# define velocidade 80 // Define a velocidade padrão, pode variar de 0 (minimo) até 25

void setup() {
  // put your setup code here, to run once:
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

  // Declarando os motores 
  pinMode (pinMot1A, OUTPUT);
  pinMode (MotorEsquerdaFrente, OUTPUT);
  pinMode (MotorDireitaFrente, OUTPUT);
  pinMode (PinMot2B, OUTPUT);
  
  analogWrite (MotorEsquerdaFrente, LOW); //Low significa "desligado"
  analogWrite (MotorDireitaFrente, LOW);

}

void loop() {
  // put your main code here, to run repeatedly:
uint16_t position = qtr.readLineBlack(sensorValues);

// print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position

   for (uint8_t i = 0; i < SensorCount; i++)
  {
    //Serial.print(sensorValues[i]);
    //Serial.print('\t');
    //Primeiro Sensor
    if(i == 0) {
      
    }
  }

  Serial.print(sensorValues[3]);
  Serial.print(" - ");
  Serial.print(qtr.calibrationOn.minimum[3]+ 200);
  Serial.print('\t');
  Serial.print(sensorValues[5]);
  Serial.print(" - ");
  Serial.println(qtr.calibrationOn.minimum[5]+ 200);
  
  if(sensorValues[3] <= qtr.calibrationOn.minimum[3] + 200 && sensorValues[5] <= qtr.calibrationOn.minimum[5] + 200){
    digitalWrite (MotorEsquerdaFrente,velocidade) ; 
    digitalWrite (MotorDireitaFrente, velocidade) ; 
  }


  
  if(sensorValues[3] > qtr.calibrationOn.minimum[3] + 200) {
    digitalWrite (MotorEsquerdaFrente, LOW);
  } else {    
   analogWrite (MotorEsquerdaFrente, velocidade);
  }

  if(sensorValues[5] > qtr.calibrationOn.minimum[5] + 200) {
   digitalWrite (MotorDireitaFrente, LOW);
  } else {    
    analogWrite (MotorDireitaFrente, velocidade);
  }

 

  delay(50);

}
