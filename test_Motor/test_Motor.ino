double out = 20;
unsigned long motorPrevMillis;

void setup() {

   pinMode(4,OUTPUT);
   pinMode(5,OUTPUT);
   pinMode(9,OUTPUT);
   pinMode(10,OUTPUT);
   
   digitalWrite(4, LOW);// Step
   digitalWrite(5, LOW);// Direction tbd

   digitalWrite(9, LOW); // Step
   digitalWrite(10, HIGH); // Direction  tbd
}

void loop() {
  
  unsigned long curMillis = millis();

  if(curMillis - motorPrevMillis >= out && digitalRead(4) == LOW){
    motorPrevMillis = curMillis;
    digitalWrite(4,HIGH);
    digitalWrite(9,HIGH);

  }else if (curMillis - motorPrevMillis >= out*2 && digitalRead(4) == HIGH){
    motorPrevMillis = curMillis;

    digitalWrite(4,LOW);
    digitalWrite(9,LOW);
  }

}
