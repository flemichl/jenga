#include <Servo.h> 

Servo myservo;

#define POT       A0
#define MAX_POT   1023

#define MIN_SERVO 60
#define MAX_SERVO 150

void setup() 
{ 
  pinMode(POT, INPUT);
  myservo.attach(9);
  Serial.begin(9600);
} 

void loop() 
{
    int potValue = analogRead(POT);
    //Serial.println(potValue);
    float angle = ((((float)potValue) / MAX_POT) * (MAX_SERVO - MIN_SERVO)) + MIN_SERVO;
    Serial.println(angle);
    myservo.write((int)angle);
} 
