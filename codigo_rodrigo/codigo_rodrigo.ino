

  #define MET 4  //ESQUERDA - TRAZ
  #define MEF 5   //ESQUERDA - FRENTE
  #define MDF 6   //DIREITA - FRENTE
  #define MDT 7  //DIREITA - TRAZ

   # define velocidade 80 // Define a velocidade padrão, pode variar de 0 (minimo) até 255 (máximo)
    int vVA;
    int vVB;
void setup() {
  // put your setup code here, to run once:
   pinMode (MET, OUTPUT);
   pinMode (MEF, OUTPUT);
   pinMode (MDF, OUTPUT);
   pinMode (MEF, OUTPUT);
  
    analogWrite (MEF, LOW); //Low significa "desligado"
    analogWrite (MDF, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
    analogWrite (MEF, velocidade); //Aqui esta dizendo que o motor da esquerda terá a velocidade de vVA
    analogWrite (MDF, velocidade);  //Aqui esta dizendo que o motor da direita terá a velocidade de vVB
    Serial.print("MotorEsquerdaFrente ");
    Serial.print(vVA);
    Serial.print(" MotorDireitaFrente ");
    Serial.println(vVB);


  delay(50);
}
