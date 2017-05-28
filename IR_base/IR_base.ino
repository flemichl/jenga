#include <IRLibDecodeBase.h>  //We need both the coding and
#include <IRLibSendBase.h>    // sending base classes
#include <IRLib_P01_NEC.h>    //Lowest numbered protocol 1st
#include <IRLib_HashRaw.h>    //We need this for IRsendRaw
#include <IRLibCombo.h>       // After all protocols, include this

// Include a receiver either this or IRLibRecvPCI or IRLibRecvLoop
#include <IRLibRecv.h>

// All of the above automatically creates a universal decoder
// class called "IRdecode" and a universal sender class "IRsend"
// containing only the protocols you want.
// Now declare instances of the decoder and the sender.
IRdecode myDecoder;
IRsend mySender; // connected to pin 3! can't be changed

#define BaseType true
#define RobotType false

int InSignal = 5;
int OutSignal = 6;

uint8_t codeProtocol =  1; // type of protocol: NEC is one
uint8_t codeBits = 32;  // length of code in bits

unsigned long repeatValue = 4294967295; //NEC repeat value
unsigned long startValue = 16738455; // NEC 1 value
unsigned long okValue = 16712445; // NEC OK value
unsigned long endValue = 16756815; // NEC 3 value
int currentState = 0;  //0 = Not Our Turn, 1 = Starting Turn, 2 = Waiting for Turn end;


char byteIn = 'n';
unsigned long value = 10;
IRrecv myReceiver(8); //pin number for the receiver - can be any digital pin
int receive_pin = 2;
volatile boolean receive_flag = true;
void setup() {
    myReceiver.enableIRIn(); // Start the receiver
    Serial.begin(9600);
    Serial.println(F("Starting IR Receiver and Sender"));

    pinMode(InSignal, INPUT); //Incoming Signal
    digitalWrite(InSignal, LOW); //Set to low initially
    pinMode(OutSignal, OUTPUT); //Outgoing Signal
    digitalWrite(OutSignal, HIGH); //Assume its not our turn first
    
}

void loop() {
  if (currentState == 0) { // waiting for our turn to start
    CheckIn();
    SendOut();
  }
  else if (currentState == 1) { // turn started, sending signals
    unsigned long time_start = millis();
    unsigned long wait_time = 100;
    while (millis() - time_start < wait_time) {
      BaseTransmit();
    }
    myReceiver.enableIRIn(); // Re-enable receiver
    Serial.println("Receive Mode");
    time_start = millis();
    while (millis() - time_start < wait_time) {
      int rec_resp = BaseReceive(okValue);
      if (rec_resp == 1) {
        currentState = 2;
        Serial.println("Waiting for Turn Over Signal");
      }
    }
  }
  else if (currentState == 2) { // robot received signal, waiting for end turn signal
    myReceiver.enableIRIn(); // Re-enable receiver
    int rec_resp2 = BaseReceive(endValue);
    if (rec_resp2 == 1) {
      currentState = 0; // back to sending not our turn signal
      Serial.println("Turn Over");
    }
  }  
}

void CheckIn() {
  if (digitalRead(InSignal) == HIGH) {
    Serial.println("In Signal Received");
    currentState = 1; // change state to our turn
  }
}

void SendOut() {
  digitalWrite(OutSignal, HIGH); // send output to high
}

int BaseReceive(unsigned long val) {
  if (myReceiver.getResults()) {
    Serial.print("IR Signal Received"); Serial.print("\t");
    receive_flag = true; 
    myDecoder.decode();
    Serial.println(myDecoder.value);
    if (myDecoder.value == val) {
      Serial.println("Received expected message");
      if (myDecoder.value == okValue) {
        Serial.println("OK Value Received");
      }
      else if (myDecoder.value == startValue) {
        Serial.println("Start Value Received");
      }
      else if (myDecoder.value == endValue) {
        Serial.println("End Value Received");
      }
      return 1;
    }
  }
  else {
    return 0;
  }
}

void BaseTransmit() {
  mySender.send(codeProtocol,codeValue,codeBits);
}
