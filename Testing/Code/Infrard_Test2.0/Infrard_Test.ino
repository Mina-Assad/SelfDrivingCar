#include <Arduino.h>
#include <PWMServo.h>
#define IN1 7
#define IN2 8
#define ENA 5  //  ENA pin

#define FRONT 90  //degree when steering facing straight forward
int SHARP_RIGHT = FRONT + 33;
int SHARP_LEFT = FRONT - 40;
int RIGHT = FRONT + 17;
int LEFT = FRONT - 20;

#define SENSOR_FRONT 90
int SENSOR_LEFT = SENSOR_FRONT + 25;
int SENSOR_RIGHT = SENSOR_FRONT - 25;
int SENSOR_FAR_LEFT = SENSOR_FRONT + 60;
int SENSOR_FAR_RIGHT = SENSOR_FRONT - 60;

#define DELAY_TIME 1000

#define SPEED 170
#define FAST_SPEED 200
#define MID_SPEED 170
#define SERVO_STEER 9    //steer servo connect to D9
#define SERVO_SENSOR 10  //Ultrasonic sensor servo connect to D10

#define Echo_PIN 2  // Ultrasonic Echo pin connect to D11
#define Trig_PIN 3  // Ultrasonic Trig pin connect to D12

PWMServo head;
PWMServo head_steer;
int leftscanval, centerscanval, rightscanval, ldiagonalscanval, rdiagonalscanval;
const int distancelimit = 30;      //distance limit for obstacles in front
const int sidedistancelimit = 30;  //minimum distance in cm to obstacles at both sides (the car will allow a shorter distance sideways)
int distance;
int numcycles = 0;
const int forwardtime = 600;  //Time the robot spends turning (miliseconds)
const int turntime = 900;     //Time the robot spends turning (miliseconds)
const int backtime = 900;     //Time the robot spends turning (miliseconds)
#define LPT 2                 // scan loop coumter
int thereis;

void turn(int angle) {

  head_steer.write(angle);
}

void stop_Stop()  //Stop
{

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
}

int watch() {
  long echo_distance;
  digitalWrite(Trig_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(Trig_PIN, HIGH);
  delayMicroseconds(15);
  digitalWrite(Trig_PIN, LOW);
  echo_distance = pulseIn(Echo_PIN, HIGH);
  //echo_distance=echo_distance*0.01657; //original value
  //echo_distance=echo_distance*0.01715; //cm
  echo_distance = echo_distance * 0.006752;
  //Serial.println((int)echo_distance);
  return round(echo_distance);
}

void setup() {

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(Trig_PIN, OUTPUT);
  pinMode(Echo_PIN, INPUT);

  head_steer.attach(SERVO_STEER);
  head.attach(SERVO_SENSOR);
  head.write(90);
  turn(FRONT);
  stop_Stop();
  delay(2000);

  Serial.begin(9600);
}


void loop() {
  Serial.println(watch());
  delay(100);
}