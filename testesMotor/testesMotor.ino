
  #define pinMot1A 5
  #define pinMot1B 6
  #define pinMot2A 9
  #define pinMot2B 10

void setup() {
  // put your setup code here, to run once:
  pinMode (pinMot1A, OUTPUT);
  pinMode (pinMot1B, OUTPUT);
  pinMode (pinMot2A, OUTPUT);
  pinMode (pinMot1B, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite (pinMot1A, 20);
  delay (2000);
  digitalWrite (pinMot1A, LOW);

  digitalWrite (pinMot2A, 20);
  delay (2000); 
  digitalWrite (pinMot2A, LOW);

  digitalWrite (pinMot1B, 20);
  delay (2000);
  digitalWrite (pinMot1B, LOW);

  digitalWrite (pinMot2B, 20);
  delay (2000); 
  digitalWrite (pinMot2B, LOW);
  
  
  

}
