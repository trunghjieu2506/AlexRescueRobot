#include <stdarg.h>
#include <math.h>
#include <serialize.h>
#include "packet.h"
#include "constants.h"

#define PI            3.141592654
#define ALEX_LENGTH   16
#define ALEX_BREADTH  6
//#define PACKET_SIZE 32

float alexDiagonal = 0.0;
float alexCirc = 0.0;

volatile TDirection dir;

/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the 
// wheel encoder.

#define COUNTS_PER_REV      4 

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          20.4

/*
 *    Alex's State Variables
 */

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks; 
volatile unsigned long rightReverseTicks;

//left and right encoder ticks for turning
volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns; 
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

unsigned long deltaDist;
unsigned long newDist;

unsigned long deltaTicks;
unsigned long targetTicks;

// TCS230 or TCS3200 pins wiring to Arduino
#define S0 47
#define S1 45
#define S2 43
#define S3 41
#define sensorOut 39
#define RED 0
#define GREEN 1
#define WHITE 2

// Stores frequency read by the photodiodes
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

// Stores the red. green and blue RGB values
int rgb[3] = {0};

// final colour.
static unsigned long colourDetected = 0;

const int TRIGFRONT = 25;
const int ECHOFRONT = 23;
const int TRIGBACK = 27;
const int ECHOBACK = 29;
#define SPEED_OF_SOUND 0.0345

double getUltra(const int TRIGPIN, const int ECHOPIN) {
  digitalWrite(TRIGPIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPIN, LOW);
  int microsecs = pulseIn(ECHOPIN,HIGH);
  double cms = microsecs*SPEED_OF_SOUND/2;

  return cms;
}

void colourDetect () {
    // Setting RED (R) filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  
  // Reading the output frequency
  redFrequency = pulseIn(sensorOut, LOW);
  // Remaping the value of the RED (R) frequency from 0 to 255
  rgb[0] = map(redFrequency, 3350, 2050, 0, 255);
  delay(100);
  
  // Setting GREEN (G) filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  
  // Reading the output frequency
  greenFrequency = pulseIn(sensorOut, LOW);

  // Remaping the value of the GREEN (G) frequency from 0 to 255. (low to high freq first)
  rgb[1] = map(greenFrequency, 4600, 3500, 0, 255);
  delay(100);
 
  // Setting BLUE (B) filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  
  // Reading the output frequency
  blueFrequency = pulseIn(sensorOut, LOW);

  // Remaping the value of the BLUE (B) frequency from 0 to 255
  rgb[2] = map(blueFrequency, 4300, 2300, 0, 255);
  delay(100);


  // Checks the current detected colour. red = 0, green = 1, white = 2.
  if(rgb[1] > rgb[0] && rgb[1] > rgb[2]) {
    colourDetected = GREEN;
  }
  else if(rgb[0] > rgb[1] && rgb[0] > rgb[2]){
    colourDetected = RED;
  }
  else {
    colourDetected =  WHITE;
  }
}

unsigned long computeDeltaTicks(float ang) {
  unsigned long ticks = (unsigned long)((ang * alexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
  return ticks;
}
void left(float ang, float speed) {

  if (ang == 0) 
    deltaTicks=99999999;

  else
    deltaTicks=computeDeltaTicks(ang);

  targetTicks = leftReverseTicksTurns + deltaTicks;
  //dbprintf("Initial Left: %d\n", leftReverseTicksTurns);
  //dbprintf("deltaTicks: %d \n", deltaTicks);
  cw(ang, speed);
}

void right(float ang, float speed) {
  if (ang == 0) 
    deltaTicks=99999999;

  else
    deltaTicks=computeDeltaTicks(ang);

  targetTicks = rightReverseTicksTurns + deltaTicks;
  //dbprintf("Initial Right: %d \n", rightReverseTicksTurns);
  //dbprintf("deltaTicks: %d \n", deltaTicks);
  ccw(ang, speed);
}


/*
 * 
 * Alex Communication Routines.
 * 
 */
 
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);  
}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
  TPacket statusPacket; 
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;

  statusPacket.params[0] = colourDetected;
  statusPacket.params[1] = rgb[0];
  statusPacket.params[2] = rgb[1];
  statusPacket.params[3] = rgb[2];
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;
  //statusPacket.params[10] = colourDetected;
  sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprintf(char *format, ...) {
     va_list args;
     char buffer[128];
     va_start(args, format);
     vsprintf(buffer, format, args);
     sendMessage(buffer);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket); 
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.
  
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);
}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 18 and 19
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 19 and 18. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 
  DDRD &= 0b11110011; 
  PORTD |= 0b00001100;
}



// Functions to be called by INT2 and INT3 ISRs.
void leftISR()
{
  if (dir == 1) {
    leftForwardTicks ++;
    forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  if (dir == 2) {
    leftReverseTicks ++;
    reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  if (dir == 3) {
    leftReverseTicksTurns ++;
    //dbprintf("LTick %d \n", leftReverseTicksTurns);
  }
  if (dir == 4) {
    leftForwardTicksTurns ++;
  }
}

void rightISR()
{
  if (dir == 1) {
    rightForwardTicks ++;
  }
  if (dir == 2) {
    rightReverseTicks ++;
  }
  if (dir == 3) {
    rightForwardTicksTurns ++;
  }
  if (dir == 4) {
    rightReverseTicksTurns ++;
    //dbprintf("RTicks %d \n", rightReverseTicksTurns);
  }
}

// Set up the external interrupt pins INT2 and INT3
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 18 and 19 to be
  // falling edge triggered. Remember to enable
  // the INT2 and INT3 interrupts.
  // Hint: Check pages 110 and 111 in the ATmega2560 Datasheet.
  EIMSK = 0b00001100;
  EICRA = 0b10100000;
}

// Implement the external interrupt ISRs below.
// INT3 ISR should call leftISR while INT2 ISR
// should call rightISR.
ISR(INT3_vect)
{
  leftISR();
}

ISR(INT2_vect)
{
  rightISR();
}


// Implement INT2 and INT3 ISRs above.

/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using 
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  // To replace later with bare-metal.
  Serial.begin(9600);
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using the other UARTs
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.
  
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

  int count=0;

  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs

  while(Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs
}

/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters()
{
  leftForwardTicks=0;
  rightForwardTicks=0;
  leftForwardTicksTurns=0;
  rightForwardTicksTurns=0;
  leftReverseTicks=0;
  rightReverseTicks=0;
  leftReverseTicksTurns=0;
  rightReverseTicksTurns=0;
  leftRevs=0;
  rightRevs=0;
  forwardDist=0;
  reverseDist=0; 
} 

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
}
// Intialize Alex's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK();
        forward((double) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_REVERSE:
        sendOK();
        backward((double) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_LEFT:
        sendOK();
        left((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_RIGHT:
        sendOK();
        right((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_GET_STATS:
    {
        sendOK();
        colourDetect();
        sendStatus();
      break;
    }
    case COMMAND_CLEAR_STATS:
        sendOK();
        clearOneCounter(command->params[0]); //clear all counters / params
      break;
    case COMMAND_STOP:
        sendOK();
        stop();
    /*
     * Implement code for other commands here.
     * 
     */
        
    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)   
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void setup() {
  // put your setup code here, to run once:
  alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  alexCirc = PI * alexDiagonal;
  cli();
  setupEINT();
  setupSerial();
  startSerial();
  enablePullups();
  initializeState();
  sei();
  //set up ultrasonic sensors 
  pinMode(TRIGFRONT, OUTPUT);
  pinMode(ECHOFRONT, INPUT);
  pinMode(TRIGBACK, OUTPUT);
  pinMode(ECHOBACK, INPUT);
  // set up colour sensor
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  
  // Setting colour sensor frequency scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}
void loop() {
  /* float ultra_front = getUltra(TRIGFRONT, ECHOFRONT);
  float ultra_back = getUltra(TRIGBACK, ECHOBACK);
  if (ultra_front < 4.0 || ultra_back < 4.0) {
    stop();
    //sendMessage("Too close");
  } */
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else
    if(result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else
      if(result == PACKET_CHECKSUM_BAD)
      {
        sendBadChecksum();
      } 
  
  if (deltaDist > 0) {
    if (dir == 1) {
      if (forwardDist > newDist || getUltra(TRIGFRONT, ECHOFRONT) < 9.0) {
        deltaDist = 0;
        newDist = 0;
        stop();
        clearCounters();
      }
    }
    else {
      if (dir == 2) {
	      //dbprintf("Backward Command Executing");
        if (reverseDist > newDist) {
          deltaDist = 0;
          newDist = 0;
          stop();
          clearCounters();
        }
      }
      else {}
    }
    if (dir == STOPPED) {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }
  if (deltaTicks > 0) {
    if (dir == 3) {
      //dbprintf("Left Ticks: %d \n", leftReverseTicksTurns);
      //dbprintf("Left,Target: %d \n", targetTicks);
      if (leftReverseTicksTurns >= targetTicks) {
        deltaTicks = 0; 
        targetTicks = 0;
        stop();
        clearCounters();
      }
    }
    else if (dir == 4) {
      //dbprintf("Right Ticks: %d \n", rightReverseTicksTurns);
      //dbprintf("Target: %d \n", targetTicks);
      //dbprintf("Target: %d \n", targetTicks);
      //dbprintf("Target: %d \n", targetTicks);

      if (rightReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
        clearCounters();
      }
    }
    else if (dir == 5) {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
    }
  }
 
}