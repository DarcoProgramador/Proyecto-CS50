int led1 = 24;
int led2 = 26;
int option = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0){
    option = Serial.read();
    Serial.print(option);
    if(option == 'P'){
         digitalWrite(led1, HIGH);
         digitalWrite(led2, LOW);
     }
    if(option == 'N'){
        digitalWrite(led1, LOW);
        digitalWrite(led2, HIGH);
     }
    if(option == 'X'){
      digitalWrite(led1, LOW);
      digitalWrite(led2, LOW);
    }
  }
  
}
