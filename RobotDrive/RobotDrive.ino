#include <TimerThree.h>
#include <TimerOne.h>
#include <Encoder.h>

// NOTE: Teensy has library for encoders.  Will just use that.
// NOTE: A is LEFT, B is RIGHT
// NOTE: TimerOne: 3, 4; TimerThree: 25, 32  ---- Don't use analogWrite on those pins
// Since we are registering a callback to Timer1, make sure that PWM pins are associated with Timer0/Timer2
// Avoid PWM pins 9, 10 since they are attached to Timer1

#define ENA 23
#define IN1 0
#define IN2 1
#define IN3 2
#define IN4 3
#define ENB 22

#define ENCODERL1   4
#define ENCODERL2   5
#define ENCODERR1   6
#define ENCODERR2   7
#define LED         13

#define SPEED_MIN 0
#define SPEED_MAX 95
// motors have different minimum speeds.
// may also want to do this for how it is mapped?
#define PWM_MIN_L   50
#define PWM_MAX_L   250
#define PWM_MIN_R   50
#define PWM_MAX_R   250

#define LoopTime 100 //milliseconds
#define dt       0.1 // seconds

#define WHEEL_BASE          19.4
#define PI_VALUE            3.14
#define ETHRESHOLD          0.5
#define NUM_TARGETS         2

#define TARGET_DISTANCE     60.96 //2 ft in cm
#define PLAYING_FIELD       67.5


//#define DEBUG
//#define PREDEF
//#define TEST_WHEELBASE

// Constants
float PULSES_REVOLUTION   = 1920.0 * 2; // from http://www.robotshop.com/en/micro-6v-160rpm-1201-dc-geared-motor-encoder.html
float WHEEL_CIRCUMFERENCE = 20.42; // in cm, based on 65mm wheel diameter
//float ADJUSTMENT_FACTOR   = 1.65; // adjustment factor to deal with squishy wheels.  Seems rather large.
//float PULSES_CM           = PULSES_REVOLUTION / WHEEL_CIRCUMFERENCE * ADJUSTMENT_FACTOR; // one cm of travel -- calibrated by driving a distance and seeing how close it gets
float PULSES_CM           = PULSES_REVOLUTION / WHEEL_CIRCUMFERENCE;
float KP                  = 1.0;
float KPW                 = 1.5;
float KI                  = 1.0;
float KD                  = 0.0;

volatile int gripper_pos = 0;

volatile float Theta = 0;
volatile float X_pos = 0;
volatile float Y_pos = 0;

volatile long LastTime = 0;
volatile long NewTime = 0;
volatile float ErrorTotalL = 0;
volatile float ErrorTotalR = 0;
volatile float DerrorDtL = 0;
volatile float DerrorDtR = 0;
volatile float prev_errorL = 0;
volatile float prev_errorR = 0;

char byteIn = 'n';
int targetDistanceL[NUM_TARGETS], targetDistanceR[NUM_TARGETS];
int nextTargetIdx = -1, currentTargetIdx = 0;

// Setup Encoders
Encoder LeftWheelEncoder(ENCODERL1, ENCODERL2);
Encoder RightWheelEncoder(ENCODERR1, ENCODERR2);

void setup()
{
  Serial.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

#ifdef DEBUG
  Serial.print("Initialize Control Loop Frequency: ");
  Serial.println(LoopTime * 1000.0);
  Serial.print("Encoder Counts to 1 cm: ");
  Serial.println(PULSES_CM);
#endif
  Timer3.initialize(LoopTime * 1000.0);
  Timer3.attachInterrupt(ControlThread);
#ifdef PREDEF
  demo1();
#endif

#ifdef TEST_WHEELBASE
  test_wheelbase();
#endif
  gyroSetup();
}


void loop()
{
  // for some reason, running this in the interrupt is affecting encoders.
  // maybe only update values in the interrupt?
  #ifndef PREDEF
    SerialRead();
  #endif
}

void ControlThread() {
  NewTime = micros(); 
}
