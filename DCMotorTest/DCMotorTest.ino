#include <TimerOne.h>

// NOTE:
// Since we are registering a callback to Timer1, make sure that PWM pins are associated with Timer0/Timer2
// Avoid PWM pins 9, 10 since they are attached to Timer1
#define ENA 11 
#define IN1 10
#define IN2 9 
#define IN3 8
#define IN4 7
#define ENB 6

#define ENCODERA1   3
#define ENCODERA2   5
#define ENCODERB1   2
#define ENCODERB2   4
#define LEDpin 13

#define SPEED_MIN 0
#define SPEED_MAX 80
#define PWM_MIN   50
#define PWM_MAX   200

#define LoopTime 100 //milliseconds
#define dt       0.1 // seconds

#define PULSES_REVOLUTION   1920 // from http://www.robotshop.com/en/micro-6v-160rpm-1201-dc-geared-motor-encoder.html
#define WHEEL_CIRCUMFERENCE 20.42 // in cm, based on 65mm wheel diameter
#define TARGET_DISTANCE     60.96 //2 ft in cm  //100  // 
#define KP                  1
#define KI                  0.1
#define ETHRESHOLD          0.5
#define NUM_TARGETS         2

volatile long EncCountA = 0;
volatile int EncStateA1 = LOW;
volatile int EncStateA2 = LOW;
volatile int EncStateA1_last = LOW;
volatile int EncStateA2_last = LOW;

volatile long EncCountB = 0;
volatile int EncStateB1 = LOW;
volatile int EncStateB2 = LOW;
volatile int EncStateB1_last = LOW;
volatile int EncStateB2_last = LOW;

volatile long LastTime = 0;
volatile long NewTime = 0;
volatile long ErrorTotalA = 0;
volatile long ErrorTotalB = 0;

char byteIn = 'n';
int targetDistanceA[NUM_TARGETS], targetDistanceB[NUM_TARGETS];
int nextTargetIdx = -1, currentTargetIdx = 0;

void setup()
{
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(LEDpin, OUTPUT);
  digitalWrite(LEDpin, HIGH);  

  attachInterrupt(digitalPinToInterrupt(ENCODERA1), EncTrackerA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODERB1), EncTrackerB, CHANGE);
  
  Serial.begin(9600);

#ifdef DEBUG  
  Serial.print("Initialize Control Loop Frequency: ");
  Serial.println(LoopTime*1000.0);
#endif
  Timer1.initialize(LoopTime*1000.0);
  Timer1.attachInterrupt(ControlThread);

  /*targetDistanceA[0] = TARGET_DISTANCE;
  targetDistanceB[0] = TARGET_DISTANCE;

  targetDistanceA[1] = TARGET_DISTANCE;
  targetDistanceB[1] = -1*TARGET_DISTANCE;

  targetDistanceA[2] = TARGET_DISTANCE;
  targetDistanceB[2] = TARGET_DISTANCE;

  targetDistanceA[3] = -1*TARGET_DISTANCE;
  targetDistanceB[3] = TARGET_DISTANCE;*/
}

void loop()
{
  // nothing here, everything done by the Control Thread
  if (Serial.available())
  {
    byteIn = Serial.read();
    if (byteIn == 't' || byteIn == 'T') // t for target
    {
      nextTargetIdx++;
      targetDistanceA[nextTargetIdx % NUM_TARGETS] = Serial.parseInt();
      targetDistanceB[nextTargetIdx % NUM_TARGETS] = Serial.parseInt();
    }
  }
}

void ControlThread(){
  NewTime = micros();

  if (currentTargetIdx == nextTargetIdx)
  {
    float errorA = targetDistanceA[currentTargetIdx % NUM_TARGETS] - (((float)EncCountA / PULSES_REVOLUTION) * WHEEL_CIRCUMFERENCE); // cm
    float errorB = targetDistanceB[currentTargetIdx % NUM_TARGETS] - (((float)EncCountB / PULSES_REVOLUTION) * WHEEL_CIRCUMFERENCE); // cm
  
  #ifdef DEBUG
    Serial.print("Error A: ");
    Serial.println(errorA);
    Serial.print("Error B: ");
    Serial.println(errorB);
  #endif
    
    // integral error
    ErrorTotalA = ErrorTotalA + (dt * errorA);
    ErrorTotalB = ErrorTotalB + (dt * errorB);
    
    float EncDifference = ((float)(EncCountA - EncCountB) / PULSES_REVOLUTION) * WHEEL_CIRCUMFERENCE;
    float speed_percentA = KP * abs(errorA) + KI * abs(ErrorTotalA);
    float speed_percentB = KP * abs(errorB) + KI * abs(ErrorTotalB);
  
      // correct for difference between encoders if we are driving straight
    if (targetDistanceA[currentTargetIdx % NUM_TARGETS] == targetDistanceB[currentTargetIdx % NUM_TARGETS])
    {
      if (EncDifference > 0)
        speed_percentA -= KP * abs(EncDifference);
        speed_percentB += KP * abs(EncDifference);
      if (EncDifference < 0)
        speed_percentA += KP * abs(EncDifference);
        speed_percentB -= KP * abs(EncDifference);
    }
  
    float state = 0.5 * (((float)EncCountA / PULSES_REVOLUTION) * WHEEL_CIRCUMFERENCE) + 0.5 * (((float)EncCountB / PULSES_REVOLUTION) * WHEEL_CIRCUMFERENCE);
  
  #ifdef DEBUG
    Serial.print("Current Location: ");
    Serial.println(state);
  #endif
    
    if (speed_percentA > SPEED_MAX)
      speed_percentA = SPEED_MAX;
  
    if (speed_percentB > SPEED_MAX)
      speed_percentB = SPEED_MAX;
    
    // send motor command  
    if (errorA > 0) {
      forwardA((int)speed_percentA);
    }
    else {
      backwardA((int)speed_percentA);
    } 
  
    if (errorB > 0) {
      forwardB((int)speed_percentB);
    }
    else {
      backwardB((int)speed_percentB);
    }

    if (abs(errorA) <= ETHRESHOLD && abs(errorB) <= ETHRESHOLD)
    {
      EncCountA = 0;
      EncCountB = 0;
      halt();
      currentTargetIdx++;
      Serial.print('y');
    }
  }
}

void forwardA(int speed_percent) {
  int speed_pwm = map(speed_percent, SPEED_MIN, SPEED_MAX, PWM_MIN, PWM_MAX);

  analogWrite(ENA, speed_pwm);   
  digitalWrite(IN1, LOW);   
  digitalWrite(IN2, HIGH);         // left wheel goes forward
}

void backwardA(int speed_percent) {
  int speed_pwm = map(speed_percent, SPEED_MIN, SPEED_MAX, PWM_MIN, PWM_MAX);

  analogWrite(ENA, speed_pwm);   
  digitalWrite(IN1, HIGH);   
  digitalWrite(IN2, LOW);         // left wheel goes forward
}

void forwardB(int speed_percent) {
  int speed_pwm = map(speed_percent, SPEED_MIN, SPEED_MAX, PWM_MIN, PWM_MAX);
  
  analogWrite(ENB,speed_pwm);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);         // right wheel goes forward
}

void backwardB(int speed_percent) {
  int speed_pwm = map(speed_percent, SPEED_MIN, SPEED_MAX, PWM_MIN, PWM_MAX);
  
  analogWrite(ENB, speed_pwm);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);         // right wheel goes forward
}

void halt() {
  digitalWrite(ENA, LOW);
  digitalWrite(IN1,LOW);      
  digitalWrite(IN2,LOW);         //left wheel holds still
  digitalWrite(ENB, LOW);
  digitalWrite(IN3,LOW);      
  digitalWrite(IN4,LOW);         // right wheel holds still
}

void EncTrackerA(){
  EncStateA1 = digitalRead(ENCODERA1);
  EncStateA2 = digitalRead(ENCODERA2);

  if ( (EncStateA1_last == LOW) && (EncStateA1 == HIGH) ){
    // going from low to high
    if (EncStateA2 == HIGH) {
      // moving forward
      EncCountA++;
    }
    else {
      // moving backward
      EncCountA--;
    }
  }
  else if ( (EncStateA1_last == HIGH) && (EncStateA1 == LOW) ) {
    // going from high to low
    if (EncStateA2 == LOW) {
      // moving forward
      EncCountA++;
    }
    else if (EncStateA2 == HIGH) {
      //moving backward
      EncCountA--;
    }
  }
//  Serial.println(EncCount);

  EncStateA1_last=EncStateA1;
  EncStateA2_last=EncStateA2;
}

void EncTrackerB(){
  EncStateB1 = digitalRead(ENCODERB1);
  EncStateB2 = digitalRead(ENCODERB2);

  if ( (EncStateB1_last == LOW) && (EncStateB1 == HIGH) ){
    // going from low to high
    if (EncStateB2 == HIGH) {
      // moving forward
      EncCountB++;
    }
    else {
      // moving backward
      EncCountB--;
    }
  }
  else if ( (EncStateB1_last == HIGH) && (EncStateB1 == LOW) ) {
    // going from high to low
    if (EncStateB2 == LOW) {
      // moving forward
      EncCountB++;
    }
    else if (EncStateB2 == HIGH) {
      //moving backward
      EncCountB--;
    }
  }
//  Serial.println(EncCount);

  EncStateB1_last=EncStateB1;
  EncStateB2_last=EncStateB2;
}

