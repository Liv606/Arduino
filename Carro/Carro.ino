//Carrega biblioteca do sensor
#include <Ultrasonic.h>

//Definindo pinos do sensor
#define pino_trigger 13
#define pino_echo 11

//Definindo pinos do motor
int IN1 = 4;
int IN2 = 5;
int IN3 = 6;
int IN4 = 7;

//Inicializa o sensor nos pinos definidos acima
Ultrasonic ultrasonic(pino_trigger, pino_echo);

void setup() {
  
//Define os pinos como saida
 pinMode(IN1, OUTPUT);
 pinMode(IN2, OUTPUT);
 pinMode(IN3, OUTPUT);
 pinMode(IN4, OUTPUT);

//Iniciando serial
  Serial.begin(9600);
  Serial.println("Lendo dados do sensor...");
}

void loop() {

//Le as informações do sensor em cm
  float cmMsec;
  long microsec = ultrasonic.timing();
  cmMsec = ultrasonic.convert(microsec, Ultrasonic::CM);

//Exibe informações no serial monitor
  Serial.print("Distancia em cm: ");
  Serial.println(cmMsec);
  //delay(1000); --> tiramos para parar na hora

//Inicia motores
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  //delay(10000);
  
//Detecta objeto
if(cmMsec <= 20){
  
  //Inverte motores
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  //delay(10000);

}

}
