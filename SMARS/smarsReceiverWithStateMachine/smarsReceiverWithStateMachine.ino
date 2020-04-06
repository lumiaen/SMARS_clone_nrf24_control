#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include <NewPing.h>

// Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8
RF24 radio(7, 8);
#define xAxisValue 0
#define yAxisValue 0
#define motor1_b 3
#define motor1_f 5
#define motor2_b 6
#define motor2_f 9

#define TRIGGER_PIN  4  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     2  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 80 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

enum state_of_robot {AUTO,STOPPED,MANUAL};
enum move_state {FWD,BCK,NONE};

uint8_t move_position = STOPPED;
uint8_t state = AUTO;

int left_speed[2]  = {0,0};
int right_speed[2] = {0,0};

struct Payload
{
  int x = 130;
  int y = 130;
  bool autoMove = false;
} payload;

//struct TelemetryPayload
//{
//  int distance = 0;
//} telemetryPayload;

// Topology
const uint64_t pipes[2] = { 0xABCDABCD71LL, 0x544d52687CLL };              // Radio pipe addresses for the 2 nodes to communicate.

void setup() {
  Serial.begin(9600);
  printf_begin();
  // Setup and configure rf radio

  radio.begin();
  radio.setAutoAck(1);                    // Ensure autoACK is enabled
  radio.enableAckPayload();               // Allow optional ack payloads
  radio.setRetries(0, 15);                // Smallest time between retries, max no. of retries
  radio.setPayloadSize(sizeof(payload));
  radio.openWritingPipe(pipes[1]);
  radio.openReadingPipe(1, pipes[0]);
  radio.startListening();                 // Start listening
  pinMode(motor1_f, OUTPUT);
  pinMode(motor1_b, OUTPUT);
  pinMode(motor2_f, OUTPUT);
  pinMode(motor2_b, OUTPUT);
  analogWrite(motor1_f, left_speed[0]);
  analogWrite(motor1_b, left_speed[1]);
  analogWrite(motor2_f, right_speed[0]);
  analogWrite(motor2_b, right_speed[1]);
}

void loop(void)
{
  byte pipeNo;

  while ( radio.available(&pipeNo)) {
    radio.read( &payload, sizeof(payload));
    radio.writeAckPayload(pipeNo, &payload, sizeof(payload) );
  }
  calculateMove();
  executeMove();
}

int calcDistance()
{
  return sonar.ping_cm();
}

void calculateMove()
{
  int move_difference = 0;
  bool turn_left=false;
  
  //Calculate main speed
  if (payload.x > 135)
  {
    left_speed[0]= map(payload.x, 136, 255, 100, 255);
    left_speed[1]= 0;
    right_speed[0]= map(payload.x, 136, 255, 100, 255);
    right_speed[1]= 0;
    move_position = FWD;
  }
  else if (payload.x <115)
  {
    left_speed[1]= map(payload.x, 114, 0, 100, 255);
    left_speed[0]= 0;
    right_speed[1]= map(payload.x, 136, 255, 100, 255);
    right_speed[0]= 0;
    move_position=BCK;
  }
  else
  {
    left_speed[0]= 0;
    left_speed[1]= 0;
    right_speed[0]= 0;
    right_speed[1]= 0;
    move_position=NONE;
  }

  //Calculate difference
  if (payload.y > 135)
  {
    move_difference = map(payload.y, 136, 255, 100, 255);
  }
  else if (payload.y <115)
  {
    move_difference = map(payload.y, 114, 0, 100, 255);
    turn_left=true;
  }

  switch (move_position)
  {
    case FWD:
      if (turn_left)
      {
        if (left_speed[0] - move_difference < 0)
        {
          left_speed[1]= move_difference - left_speed[0];
          left_speed[0]= 0;
        }
        else
        {
          left_speed[0]= left_speed[0] - move_difference ;
        }
      }
      else
      {
        if (right_speed[0] - move_difference < 0)
        {
          right_speed[1]= move_difference - right_speed[0];
          right_speed[0]= 0;
        }
        else
        {
          right_speed[0]= right_speed[0] - move_difference ;
        }
      }
      break;
    case BCK:
      if (turn_left)
      {
        if (left_speed[1] - move_difference < 0)
        {
          left_speed[0]= move_difference - left_speed[1];
          left_speed[1]= 0;
        }
        else
        {
          left_speed[1]= left_speed[1] - move_difference ;
        }
      }
      else
      {
        if (right_speed[1] - move_difference < 0)
        {
          right_speed[0]= move_difference - right_speed[1];
          right_speed[1]= 0;
        }
        else
        {
          right_speed[1]= right_speed[1] - move_difference ;
        }
      }
      break;
    case NONE:
      if (turn_left)
      {
        left_speed[1] = move_difference;
        left_speed[0] = 0;
        right_speed[1] = 0;
        right_speed[0] = move_difference;
      }
      else
      {
        left_speed[0] = move_difference;
        left_speed[1] = 0;
        right_speed[0] = 0;
        right_speed[1] = move_difference;
      }
  }
}

void executeMove()
{
  if (state == AUTO)
  {
    if (payload.autoMove)
    {
      state = MANUAL;
    }
    else
    {
      adjustMove();
    }
  }
  if (state == MANUAL)
  {
    analogWrite(motor1_f, left_speed[0]);
    analogWrite(motor1_b, left_speed[1]);
    analogWrite(motor2_f, right_speed[0]);
    analogWrite(motor2_b, right_speed[1]);
  } 
}


void adjustMove()
{
  int dist = sonar.convert_cm(sonar.ping_median(5));
  int retries=0;
  while (dist < 10 && retries<10)
  {
    analogWrite(motor1_f, 0);
    analogWrite(motor1_b, 200);
    analogWrite(motor2_f, 200);
    analogWrite(motor2_b, 0);
    delay(200);
    dist = calcDistance();
    Serial.print("Distance : ");
    Serial.println(dist);
    Serial.print("Retries : ");
    Serial.println(retries);
    retries++;
  }
  if (retries == 10)
  {
    analogWrite(motor1_f, 0);
    analogWrite(motor1_b, 0);
    analogWrite(motor2_f, 0);
    analogWrite(motor2_b, 0);
    state=MANUAL;
    move_position=NONE;
  }
  else
  {
    analogWrite(motor1_f, 200);
    analogWrite(motor1_b, 0);
    analogWrite(motor2_f, 200);
    analogWrite(motor2_b, 0);
  }
}
