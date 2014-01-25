// MultiChannels
//
// rcarduino.blogspot.com
//
// A simple approach for reading three RC Channels using pin change interrupts
//
// See related posts -
// http://rcarduino.blogspot.co.uk/2012/01/how-to-read-rc-receiver-with.html
// http://rcarduino.blogspot.co.uk/2012/03/need-more-interrupts-to-read-more.html
// http://rcarduino.blogspot.co.uk/2012/01/can-i-control-more-than-x-servos-with.html

#include <Servo.h>
#include <Serial.h>


//Enable print statements
//1 = PWMs received by arduino
//2 = analogWrite value sent by the arduino
#define DEBUG 2
//INPUT PINS
#define XMOTOR_IN_PIN 3
#define YMOTOR_IN_PIN 2

//OUTPUT PINS
//Digital direction pins
//RESERVED FOR INTERRUPTS: 2	3	21	20	19	18
#define XMOTOR_DOUT_PIN1 9
#define XMOTOR_DOUT_PIN2 10
#define YMOTOR_DOUT_PIN1 11
#define YMOTOR_DOUT_PIN2 12

//Analog speed pins
#define XMOTOR_AOUT_PIN1 4 // CH1
#define XMOTOR_AOUT_PIN2 5 // CH2
#define YMOTOR_AOUT_PIN1 6 // CH3
#define YMOTOR_AOUT_PIN2 7 // CH4 - ALL FOR ROVER5


#define XMOTOR_FLAG 1
#define YMOTOR_FLAG 2

#define MAX_XMOTORPW 1916 
#define MIN_XMOTORPW 1108 
#define MAX_YMOTORPW 1928 
#define MIN_YMOTORPW 1108 
const int MID_XMOTORPW =  (MAX_XMOTORPW + MIN_XMOTORPW)/2;
const int MID_YMOTORPW = (MAX_YMOTORPW + MIN_YMOTORPW)/2;
// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;


volatile uint16_t unXMOTORInShared;
volatile uint16_t unYMOTORInShared;


uint32_t ulXMOTORStart;
uint32_t ulYMOTORStart;


void updateXMOTORS(int switchPW){
  //determine direction of the motors
  if(switchPW < MID_XMOTORPW){
    digitalWrite(XMOTOR_DOUT_PIN1, LOW);
    digitalWrite(XMOTOR_DOUT_PIN2, LOW);  
  }
  else{
    digitalWrite(XMOTOR_DOUT_PIN1, HIGH);
    digitalWrite(XMOTOR_DOUT_PIN2, HIGH);  
  }
  
  //determine speed of the motors
  int analogWriteValue = ((double)(abs(switchPW - MID_XMOTORPW))/(double)(MID_XMOTORPW - MIN_XMOTORPW)) * 255;
  if(analogWriteValue < 20) analogWriteValue = 0;
  if(analogWriteValue > 255) analogWriteValue = 255;
  if(DEBUG == 2){
    static int count = 0;
    count++;
    if(count > 5){
      Serial.print("analogWriteValue for XMOTOR: ");
      Serial.println(analogWriteValue);
      count = 0;
    }
  } 
  analogWrite(XMOTOR_AOUT_PIN1, analogWriteValue);
  analogWrite(XMOTOR_AOUT_PIN2, analogWriteValue);  
}

void updateYMOTORS(int switchPW){
  //determine direction of the motors
  if(switchPW < MID_YMOTORPW){
    digitalWrite(YMOTOR_DOUT_PIN1, LOW);
    digitalWrite(YMOTOR_DOUT_PIN2, LOW);  
  }
  else{
    digitalWrite(YMOTOR_DOUT_PIN1, HIGH);
    digitalWrite(YMOTOR_DOUT_PIN2, HIGH);  
  }
  
  //determine speed of the motors
  int analogWriteValue = ((double)abs(switchPW - MID_YMOTORPW))/(double)(MID_YMOTORPW - MIN_YMOTORPW) * 255;
  if(analogWriteValue < 20) analogWriteValue = 0;
  if(analogWriteValue > 255) analogWriteValue = 255;
  if(DEBUG == 2){
    static int count = 0;
    count++;
    if(count > 5){
      Serial.print("analogWriteValue for YMOTOR: ");
      Serial.println(analogWriteValue);
      count = 0;
    }
  } 
  
  //sandwhich analogWrite value so it does not write unless it passes a threshold
  analogWrite(YMOTOR_AOUT_PIN1, analogWriteValue);
  analogWrite(YMOTOR_AOUT_PIN2, analogWriteValue);    
}



void setup()
{
  Serial.begin(14400);
 
  Serial.println("multiChannels");
  
  
  
  attachInterrupt(1, calcXMOTOR,CHANGE);
  attachInterrupt(0, calcYMOTOR,CHANGE);
  pinMode(XMOTOR_DOUT_PIN1, OUTPUT);
  pinMode(XMOTOR_DOUT_PIN2, OUTPUT);
  pinMode(YMOTOR_DOUT_PIN1, OUTPUT);
  pinMode(YMOTOR_DOUT_PIN2, OUTPUT);
  pinMode(XMOTOR_AOUT_PIN1, OUTPUT);
  pinMode(XMOTOR_AOUT_PIN2, OUTPUT);
  pinMode(YMOTOR_AOUT_PIN1, OUTPUT);
  pinMode(XMOTOR_AOUT_PIN2, OUTPUT);
}

void loop()
{
  static uint16_t unXMOTORIn;
  static uint16_t unYMOTORIn;
  // local copy of update flags
  static uint8_t bUpdateFlags;

  // check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;
   
    if(bUpdateFlags & XMOTOR_FLAG)
    {
      unXMOTORIn = unXMOTORInShared;
    }
   
    if(bUpdateFlags & YMOTOR_FLAG)
    {
      unYMOTORIn = unYMOTORInShared;
    }
   
    bUpdateFlagsShared = 0;
   
    interrupts(); 
  }
 

  if(bUpdateFlags & XMOTOR_FLAG)
  {
    updateXMOTORS(unXMOTORIn);
  }
 
  if(bUpdateFlags & YMOTOR_FLAG)
  {
    updateYMOTORS(unYMOTORIn);
  }
   bUpdateFlags = 0;
}


// simple interrupt service routine
void calcXMOTOR()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(XMOTOR_IN_PIN) == HIGH)
  {
    ulXMOTORStart = micros();
  }
  else
  {
    unXMOTORInShared = (uint16_t)(micros() - ulXMOTORStart);
    //Print every 5th PW received from receiver to not flood serial line
    if(DEBUG == 1){
      static int countX = 0;
      countX++;
      if(countX > 5){
        Serial.print("XMOTOR PW: ");
        Serial.println(unXMOTORInShared);
        countX = 0;
      }
    }
    bUpdateFlagsShared |= XMOTOR_FLAG;
  }
}

void calcYMOTOR()
{
  if(digitalRead(YMOTOR_IN_PIN) == HIGH)
  {
    ulYMOTORStart = micros();
  }
  else
  {
    unYMOTORInShared = (uint16_t)(micros() - ulYMOTORStart);
    //Print every 5th PW received from receiver to not flood serial line
    if(DEBUG == 1){
      static int countY = 0;
      countY++;
      if(countY > 5){
        Serial.print("YMOTOR PW: ");
        Serial.println(unYMOTORInShared);
        countY = 0;
      }
    }
    bUpdateFlagsShared |= YMOTOR_FLAG;
  }
}
