#include <QTRSensors.h> //biblioteca do sensor

// This example is designed for use with six analog QTR sensors. These
// reflectance sensors should be connected to analog pins A0 to A5. The
// sensors' emitter control pin (CTRL or LEDON) can optionally be connected to
// digital pin 2, or you can leave it disconnected and remove the call to
// setEmitterPin().
//
// The setup phase of this example calibrates the sensors for ten seconds and
// turns on the Arduino's LED (usually on pin 13) while calibration is going
// on. During this phase, you should expose each reflectance sensor to the
// lightest and darkest readings they will encounter. For example, if you are
// making a line follower, you should slide the sensors across the line during
// the calibration phase so that each sensor can get a reading of how dark the
// line is and how light the ground is. Improper calibration will result in
// poor readings.
//
// The main loop of the example reads the calibrated sensor values and uses
// them to estimate the position of a line. You can test this by taping a piece
// of 3/4" black electrical tape to a piece of white paper and sliding the
// sensor across it. It prints the sensor values to the serial monitor as
// numbers from 0 (maximum reflectance) to 1000 (minimum reflectance) followed
// by the estimated location of the line as a number from 0 to 5000. 1000 means
// the line is directly under sensor 1, 2000 means directly under sensor 2,
// etc. 0 means the line is directly under sensor 0 or was last seen by sensor
// 0 before being lost. 5000 means the line is directly under sensor 5 or was
// last seen by sensor 5 before being lost.

QTRSensors qtr;

const uint8_t SensorCount = 8; //Declarando o número de sensores
uint16_t sensorValues[SensorCount];

  #define pinMot1A 5   //ESQUERDA - TRAZ
  #define MotorEsquerdaFrente 6   //ESQUERDA - FRENTE
  #define MotorDireitaFrente 9   //DIREITA - FRENTE
  #define pinMot2B 10  //DIREITA - TRAZ

  # define velocidade 150 // Define a velocidade padrão, pode variar de 0 (minimo) até 255 (máximo)
  int vVA;
  int vVB;

void setup()
{
  
Serial.begin(9600);
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0,A1,A2,A3,A4,A5,A6,A7}, SensorCount);  // Declarando os sensores (onde estão ligados no arduino) 
  qtr.setEmitterPin(2);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); //O LED do arduino liga para mostrar que está calibrando

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  
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
  pinMode (pinMot2B, OUTPUT);
  
  analogWrite (MotorEsquerdaFrente, LOW); //Low significa "desligado"
  analogWrite (MotorDireitaFrente, LOW);
  
}

void loop()
{
  // Nessa parte será printado as posições dos 8 sensores em conjunto, variando de 0 a 5000 --> 2500 provavelmente significa o meio 
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  /*for (uint8_t i = 0; i < SensorCount; i++)
  {
    //Serial.print(sensorValues[i]);
    //Serial.print('\t');
    //Primeiro Sensor
    if(i == 0) {
      
    }
  }
  Serial.print(sensorValues[0]);
  Serial.print(" - ");
  Serial.print(qtr.calibrationOn.minimum[0]+ 200);
  Serial.print('\t');
  Serial.print(sensorValues[1]);
  Serial.print(" - ");
  Serial.println(qtr.calibrationOn.minimum[1]+ 200);
  */

  Serial.print("Possicao lida ");
  Serial.print(position);
  Serial.println();
  //if(sensorValues[0] < qtr.calibrationOn.minimum[0] + 200) {
  //  digitalWrite (MotorEsquerdaFrente, LOW);
  //} else {    
  //  analogWrite (MotorEsquerdaFrente, velocidade);
  //}

  //if(sensorValues[1] < qtr.calibrationOn.minimum[1] + 200) {
  //  digitalWrite (MotorDireitaFrente, LOW);
  //} else {    
  //  analogWrite (MotorDireitaFrente, velocidade);
  //}

//delay(250);
    // Valores que foram calibrados na casa do rodrigo, provavelmente mudarão por conta de sombras e iluminação
    // esquerda
    if(position <= 3300) {
      vVB = velocidade - 50;  // Reduzimos a velocidade em x unidades para que o motor da esquerda fique lento e ele faça a curva para esquerda 
      } else // reto
        if(position > 3300 && position < 3800) {
          vVA = velocidade;
          vVB = velocidade;
        }else // direita
         if((position >= 3800)) {
            vVA = velocidade - 50 ; // Reduzimos a velocidade em x unidades para que o motor da direita fique lento e ele faça a curva para a direita
          }
    analogWrite (MotorEsquerdaFrente, vVA); //Aqui esta dizendo que o motor da esquerda terá a velocidade de vVA
    analogWrite (MotorDireitaFrente, vVB);  //Aqui esta dizendo que o motor da direita terá a velocidade de vVB
    Serial.print("MotorEsquerdaFrente ");
    Serial.print(vVA);
    Serial.print(" MotorDireitaFrente ");
    Serial.println(vVB);
    

  /*if(sensorValues[0] < qtr.calibrationOn.minimum[0] + 200) {
    digitalWrite (MotorEsquerdaFrente, LOW);
    
  } else if (sensorValues[1] < qtr.calibrationOn.minimum[1] + 200) {  
    digitalWrite (MotorDireitaFrente, LOW);
    
  } else if (sensorValues[0] > qtr.calibrationOn.minimum[0] + 200 && sensorValues[1] > qtr.calibrationOn.minimum[1] + 200) {
      analogWrite (MotorEsquerdaFrente, velocidade); 
      analogWrite (MotorDireitaFrente, velocidade);
      
  } else { 
    digitalWrite (MotorEsquerdaFrente, LOW);
  }*/

  //delay(50);
        }
