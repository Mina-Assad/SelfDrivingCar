#include <PWMServo.h>
//#include <cstdlib>
#define IN1 7
#define IN2 8
#define ENA 5       //  ENA pin
#define INT1 2
#define INT2 3

#define FRONT 55        // steering to front 
int SHARP_RIGHT= 180;
int SHARP_LEFT=FRONT-180;
int  RIGHT=150;
int  LEFT=-150;
  
int DELAY_TIME = 1000;
int DELAY_TIME2 = 2*DELAY_TIME;

  
#define LFSensor_0 INT1  //OLD D3
#define LFSensor_1 A1
#define LFSensor_2 A2
#define LFSensor_3 A3
#define LFSensor_4 INT2  //OLD D10

#define SERVO_PIN    9  //servo connect to D3

#define SPEED 120
#define FAST SPEED + 30
#define MID SPEED + 10

#define trigger 0

bool FLG = 0;
int D = 0;

/*const int SPEED = 120;
const int FAST_SPEED = SPEED;
const int MID_SPEED = SPEED;
*/

PWMServo head;
/*motor control*/
void go_Back(int speed)  //Forward
{
  Serial.println("backward");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2,LOW);
   analogWrite(ENA,speed);
}
 
void go_Advance(int speed)  //Forward
{
  //Serial.println("forward");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2,HIGH);
 

   analogWrite(ENA,speed);
}
 
void turn(int angle)
{

  head.write(angle);

}
 
void stop_Stop()    //Stop
{

  Serial.println("stopward");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2,LOW);

  analogWrite(ENA,0);
}

void setup() {

 pinMode(ENA, OUTPUT); 
 pinMode(IN1, OUTPUT); 
 pinMode(IN2, OUTPUT); 
 pinMode(INT1, INPUT); 
 pinMode(INT2, INPUT); 
 attachInterrupt(digitalPinToInterrupt(INT1), Handler, FALLING);
 attachInterrupt(digitalPinToInterrupt(INT2), Handler, FALLING);
 head.attach(SERVO_PIN);
 turn(FRONT);
 stop_Stop();
 Serial.begin(9600);
 
}

void Handler(){
  char s[5]; // Sensor input array based on the pins;
  s[0] = digitalRead(LFSensor_0); // Left most sensor
  s[1] = digitalRead(LFSensor_1);
  s[2] = digitalRead(LFSensor_2);
  s[3] = digitalRead(LFSensor_3);
  s[4] = digitalRead(LFSensor_4); // Right most sensor
  if(s[0] != s[4]){
    if(s[0] == trigger){
      Serial.println("left");
      if(s[1] == trigger){
      }
      else{
      }
      D = 1;
    }
    else{
      Serial.println("right");
      if(s[3] == trigger){
      }
      else{
      }
      D = 2;
    }
  }
  else{
    Serial.println("both");
    D = 3;
  }
}
void loop() {
  switch(D){
    case 0:
      go_Advance(MID);
      break;
    case 1:
      go_Back(MID);
      delay(1000);
      stop_Stop();
      turn(180);
      go_Advance(FAST);
      delay(1000);
      turn(FRONT);
      D = 0;
      break;
    case 2:
      go_Back(MID);
      delay(1000);
      stop_Stop();
      turn(-180);
      go_Advance(FAST);
      delay(1000);
      turn(FRONT);
      D = 0;
      break;
  }
}

void stateReaction(int x){
  switch(x){
    case 0:
      go_Advance(MID);
      break;
    case 1:
      go_Back(MID);
      delay(1000);
      stop_Stop();
      turn(180);
      go_Advance(FAST);
      delay(1000);
      turn(-100);
      D = 0;
      break;
    case 2:
      go_Back(MID);
      delay(1000);
      stop_Stop();
      turn(-180);
      go_Advance(FAST);
      delay(1000);
      turn(100);
      D = 0;
      break;
  }
}

void straighteningUp(bool x){
  if(x == 0){
    return;
  }
  char s[5]; // Sensor input array based on the pins;
  s[0] = digitalRead(LFSensor_0);
  s[1] = digitalRead(LFSensor_1);
  s[2] = digitalRead(LFSensor_2);
  s[3] = digitalRead(LFSensor_3);
  s[4] = digitalRead(LFSensor_4);
  if(s[0] != trigger && s[1] != trigger && s[2] != trigger && s[3] != trigger && s[4] != trigger){
    x = 0;
  }
  return;
}