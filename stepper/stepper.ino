#include <Stepper.h>

const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
// for your motor

#define ENABLE1   2
#define STEPPERA1 3
#define STEPPERA2 4
#define ENABLE2   9
#define STEPPERB1 7
#define STEPPERB2 8
#define LED    13

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, STEPPERA1, STEPPERA2, STEPPERB1, STEPPERB2);

int stepCount = 0;  // number of steps the motor has taken

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(ENABLE1, OUTPUT);
  pinMode(ENABLE2, OUTPUT);
  digitalWrite(ENABLE1, HIGH);
  digitalWrite(ENABLE2, HIGH);
  myStepper.setSpeed(60);
}

void loop() {
  // step one revolution  in one direction:
  digitalWrite(LED, HIGH);
  myStepper.step(stepsPerRevolution);
  delay(500);

  // step one revolution in the other direction:
  digitalWrite(LED, LOW);
  myStepper.step(-stepsPerRevolution);
  delay(500);
}
