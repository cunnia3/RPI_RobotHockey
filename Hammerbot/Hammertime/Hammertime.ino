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
//
// rcarduino.blogspot.com
//

// include the pinchangeint library - see the links in the related topics section above for details



#include <PinChangeInt.h>

#include <Servo.h>
#include <Serial.h>


#define DEBUG 1
// Assign your channel in pins
#define THROTTLE_IN_PIN 3
#define STEERING_IN_PIN 2
#define AUX_IN_PIN 7

// Assign your channel out pins
#define gatelPin 6
#define gaterPin 9
#define wing1Pin 5
#define wing2Pin 4
Servo servoAux;

Servo gatel;
Servo gater;
//gate l good
int gatelClosed = 60;  
int gatelOpen = 180;

int gaterClosed = 160;
int gaterOpen = 60;

//Number corresponds to the wing that has to open first
Servo wing1;
Servo wing2;
int wing1Closed = 0;
int wing1Open = 175;
int wing2Closed = 160;
int wing2Open = 50;

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define THROTTLE_FLAG 1
#define STEERING_FLAG 2
#define AUX_FLAG 4
#define NEUTRAL_THROTTLE 1500 // this is the duration in microseconds of neutral throttle on an electric RC Car

// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;


volatile uint16_t unThrottleInShared;
volatile uint16_t unSteeringInShared;
volatile uint16_t unAuxInShared;


uint32_t ulThrottleStart;
uint32_t ulSteeringStart;
uint32_t ulAuxStart;


//Will check to see if the switch on the transmitter is on or off and handle the input accordingly
void updateGateservos(int switchPW){
  //if the switch is on
  if(switchPW > NEUTRAL_THROTTLE + 200)
  {
    //open the gates
    gatel.write(gatelOpen);
    gater.write(gaterOpen);
  }
  else
  {
    //close the gates
    gatel.write(gatelClosed);
    gater.write(gaterClosed);
  }
}

void updateWingservos(int switchPW){
  if(switchPW > NEUTRAL_THROTTLE + 300)
  {
    wing1.write(wing1Open);
    delay(900);
    wing2.write(wing2Open);    
  }
  else if(switchPW > NEUTRAL_THROTTLE - 300)
  {

    wing2.write(wing2Closed);
    delay(900);
    wing1.write(wing1Closed);
  }  
}

void setup()
{
  Serial.begin(9600);
 
  Serial.println("multiChannels");
  gatel.attach(gatelPin,500, 2200);
  gater.attach(gaterPin,500, 2500);
  wing1.attach(wing1Pin, 500, 2500);
  wing2.attach(wing2Pin);
  wing1.write(wing1Closed);
  wing2.write(wing2Closed);

  // using the PinChangeInt library, attach the interrupts
  // used to read the channels
  attachInterrupt(1, calcThrottle,CHANGE);
  attachInterrupt(0, calcSteering,CHANGE);
}

void loop()
{
  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that thier values will be retained
  // between calls to loop.
  static uint16_t unThrottleIn;
  static uint16_t unSteeringIn;
  static uint16_t unAuxIn;
  // local copy of update flags
  static uint8_t bUpdateFlags;

  // check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;
   
    // in the current code, the shared values are always populated
    // so we could copy them without testing the flags
    // however in the future this could change, so lets
    // only copy when the flags tell us we can.
   
    if(bUpdateFlags & THROTTLE_FLAG)
    {
      unThrottleIn = unThrottleInShared;
    }
   
    if(bUpdateFlags & STEERING_FLAG)
    {
      unSteeringIn = unSteeringInShared;
    }
   
    if(bUpdateFlags & AUX_FLAG)
    {
      unAuxIn = unAuxInShared;
    }
    
    bUpdateFlagsShared = 0;
   
    interrupts(); 
  }
 

  if(bUpdateFlags & THROTTLE_FLAG)
  {
    updateGateservos(unThrottleIn);
  }
 
  if(bUpdateFlags & STEERING_FLAG)
  {
    updateWingservos(unSteeringIn);
  }
   bUpdateFlags = 0;
}


// simple interrupt service routine
void calcThrottle()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(THROTTLE_IN_PIN) == HIGH)
  {
    ulThrottleStart = micros();
  }
  else
  {
    unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);
    //if(DEBUG)Serial.println(unThrottleInShared);
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}

void calcSteering()
{
  if(digitalRead(STEERING_IN_PIN) == HIGH)
  {
    ulSteeringStart = micros();
  }
  else
  {
    unSteeringInShared = (uint16_t)(micros() - ulSteeringStart);
    if(DEBUG)Serial.println(unSteeringInShared);
    bUpdateFlagsShared |= STEERING_FLAG;
  }
}

void calcAux()
{
  if(digitalRead(AUX_IN_PIN) == HIGH)
  {
    ulAuxStart = micros();
  }
  else
  {
    unAuxInShared = (uint16_t)(micros() - ulAuxStart);
    bUpdateFlagsShared |= AUX_FLAG;
  }
}
