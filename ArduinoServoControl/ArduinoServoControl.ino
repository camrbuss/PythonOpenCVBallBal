#include <Servo.h>

uint8_t incomingByte = 0;   // for incoming serial data
uint8_t servoAngle = 0;
uint8_t plusMinusAngle = 40;
uint8_t zeroAngle = 80;

Servo myservo;  // create servo object to control a servo

void setup() {

  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  


  Serial.begin(9600);
  delay(1000);
  while (!Serial) 
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  myservo.write(zeroAngle);

}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) 
  {
    // read the incoming byte:
    incomingByte = Serial.read();
    servoAngle = map(incomingByte,32,126,zeroAngle-plusMinusAngle,zeroAngle+plusMinusAngle);
    myservo.write(servoAngle);
    
    // say what you got:
    //Serial.println(incomingByte);
  }

}
