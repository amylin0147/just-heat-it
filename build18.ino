#include<Wire.h>
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //Serial.println("ready");
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  if (Serial.available() > 0) {
    // read the incoming byte:
    int incomingByte = Serial.read();
    if(incomingByte == 1){digitalWrite(4, HIGH);}
    if(incomingByte == 2){digitalWrite(4, LOW);}
  
    // say what you got:
    //Serial.print("I received: ");
    //Serial.println(incomingByte);
  }
}
