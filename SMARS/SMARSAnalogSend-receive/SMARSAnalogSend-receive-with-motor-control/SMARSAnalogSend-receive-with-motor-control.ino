/*
  // March 2014 - TMRh20 - Updated along with High Speed RF24 Library fork
  // Parts derived from examples by J. Coliz <maniacbug@ymail.com>
*/
/**
 * Example for efficient call-response using ack-payloads 
 *
 * This example continues to make use of all the normal functionality of the radios including
 * the auto-ack and auto-retry features, but allows ack-payloads to be written optionally as well.
 * This allows very fast call-response communication, with the responding radio never having to 
 * switch out of Primary Receiver mode to send back a payload, but having the option to if wanting
 * to initiate communication instead of respond to a commmunication.
 */
 


#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

// Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 
RF24 radio(7,8);
int xAxisPin = A0;
int yAxisPin = A1;
int xAxisValue=0 ,yAxisValue=0 ;
int motor1_f=3,motor1_b=5,motor2_f=6,motor2_b=9;
struct Payload {
  int x=0;
  int y=0;
} payload;

// Topology
const uint64_t pipes[2] = { 0xABCDABCD71LL, 0x544d52687CLL };              // Radio pipe addresses for the 2 nodes to communicate.

// Role management: Set up role.  This sketch uses the same software for all the nodes
// in this system.  Doing so greatly simplifies testing.  

typedef enum { role_ping_out = 1, role_pong_back } role_e;                 // The various roles supported by this sketch
const char* role_friendly_name[] = { "invalid", "Ping out", "Pong back"};  // The debug-friendly names of those roles
role_e role = role_pong_back;                                              // The role of the current running sketch


void setup(){

  Serial.begin(9600);
  printf_begin();
  Serial.print(F("\n\rRF24/examples/pingpair_ack/\n\rROLE: "));
  Serial.println(role_friendly_name[role]);
  Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));
  
  
  
  // Setup and configure rf radio

  radio.begin();
  radio.setAutoAck(1);                    // Ensure autoACK is enabled
  radio.enableAckPayload();               // Allow optional ack payloads
  radio.setRetries(0,15);                 // Smallest time between retries, max no. of retries
  radio.setPayloadSize(sizeof(payload));
  if (role == role_pong_back)
  {
    radio.openWritingPipe(pipes[1]);
    radio.openReadingPipe(1,pipes[0]);
  }
  else
  {
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1,pipes[1]);
  }
  radio.startListening();                 // Start listening
  //radio.printDetails();                   // Dump the configuration of the rf unit for debugging
  pinMode(motor1_f,OUTPUT);
  pinMode(motor1_b,OUTPUT);
  pinMode(motor2_f,OUTPUT);
  pinMode(motor2_b,OUTPUT);
  analogWrite(motor1_f,0);
  analogWrite(motor1_b,0);
  analogWrite(motor2_f,0);
  analogWrite(motor2_b,0);
}

void loop(void) {

  if (role == role_ping_out){
    
    radio.stopListening();                                  // First, stop listening so we can talk.
    xAxisValue = analogRead(xAxisPin);
    yAxisValue = analogRead(yAxisPin);
    payload.x = map(xAxisValue,0,1023,0,254);
    payload.y = map(yAxisValue,0,1023,0,254);
    printf("Now sending %d and %d as payload. ",payload.x, payload.y);
    unsigned long time = micros();                          // Take the time, and send it.  This will block until complete   
                                                            //Called when STANDBY-I mode is engaged (User is finished sending)
    if (!radio.write( &payload, sizeof(payload) )){
      Serial.println(F("failed."));      
    }else{

      if(!radio.available()){ 
        Serial.println(F("Blank Payload Received.")); 
      }else{
        while(radio.available() ){
          unsigned long tim = micros();
          radio.read( &payload, sizeof(payload));
          printf("Got response %d and %d,\n\r",payload.x,payload.y,tim-time);
        }
      }

    }
    // Try again later
    delay(10);
  }

  // Pong back role.  Receive each packet, dump it out, and send it back

  if ( role == role_pong_back ) {
    byte pipeNo;
    while( radio.available(&pipeNo)){
      radio.read( &payload,sizeof(payload));
      radio.writeAckPayload(pipeNo,&payload, sizeof(payload) );  
      calculateMove();     
   }
 }

  // Change roles

  if ( Serial.available() )
  {
    char c = toupper(Serial.read());
    if ( c == 'T' && role == role_pong_back )
    {
      Serial.println(F("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK"));

      role = role_ping_out;                  // Become the primary transmitter (ping out)
      radio.openWritingPipe(pipes[0]);
      radio.openReadingPipe(1,pipes[1]);
    }
    else if ( c == 'R' && role == role_ping_out )
    {
      Serial.println(F("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK"));
      
       role = role_pong_back;                // Become the primary receiver (pong back)
       radio.openWritingPipe(pipes[1]);
       radio.openReadingPipe(1,pipes[0]);
       radio.startListening();
    }
  }
}

void calculateMove()
{
  int x,y,motor1,motor2;
  bool f=false,b=false,l=false,r=false;
  if (payload.x >135)
  {
    x=map(payload.x,136,254,100,254);
    f=true;
    b=false;
  }
  else
  if (payload.x <115)
  {
    x=map(payload.x,114,0,100,254);
    b=true;
    f=false;
  }
  else
  {
    x=0;
  }
  if (payload.y >135)
  {
    y=map(payload.y,136,254,100,200);
    l=true;
    r=false;
  }
  else
  if (payload.y <115)
  {
    y=map(payload.y,114,0,100,200);
    r=true;
    l=false;
  }
  else
  {
    y=0;
  }
  int diffCalc = x-y;
    if (diffCalc<0)
      diffCalc=0;
  if (f)
  {
    if (l)
    {
    analogWrite(motor1_f,diffCalc);
    analogWrite(motor1_b,0);
    analogWrite(motor2_f,x);
    analogWrite(motor2_b,0);
    }
    else if (r)
    {
      analogWrite(motor1_f,x);
      analogWrite(motor1_b,0);
      analogWrite(motor2_f,diffCalc);
      analogWrite(motor2_b,0);
    }
    else
    {
      analogWrite(motor1_f,x);
      analogWrite(motor1_b,0);
      analogWrite(motor2_f,x);
      analogWrite(motor2_b,0);
    }
  }
  else if (b)
  {
    if (l)
    {
    analogWrite(motor1_b,diffCalc);
    analogWrite(motor1_f,0);
    analogWrite(motor2_b,x);
    analogWrite(motor2_f,0);
    }
    else if (r)
    {
      analogWrite(motor1_b,x);
      analogWrite(motor1_f,0);
      analogWrite(motor2_b,diffCalc);
      analogWrite(motor2_f,0);
    }
    else
    {
      analogWrite(motor1_b,x);
      analogWrite(motor1_f,0);
      analogWrite(motor2_b,x);
      analogWrite(motor2_f,0);
    }
  }
  else if (!f && !b)
  {
    if (l)
    {
      analogWrite(motor1_f,0);
      analogWrite(motor1_b,y);
      analogWrite(motor2_f,y);
      analogWrite(motor2_b,0);
    }
    else if (r)
    {
      analogWrite(motor1_f,y);
      analogWrite(motor1_b,0);
      analogWrite(motor2_f,0);
      analogWrite(motor2_b,y);
    }
    else
    {
      analogWrite(motor1_f,0);
      analogWrite(motor1_b,0);
      analogWrite(motor2_f,0);
      analogWrite(motor2_b,0);
    }
}
}
