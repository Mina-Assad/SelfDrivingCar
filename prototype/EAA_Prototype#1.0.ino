#include <PWMServo.h>
#define IN1 7
#define IN2 8
#define ENA 5       //  ENA pin
#define INT1 2
#define INT2 3
 

#define FRONT 50        // steering to front 
int SHARP_RIGHT=FRONT+50;
int SHARP_LEFT=FRONT-50;
int  RIGHT=FRONT+20;
int  LEFT=FRONT-20;
  
#define DELAY_TIME 4000   
  
#define LFSensor_0 INT1  //OLD D3
#define LFSensor_1 A1
#define LFSensor_2 A2
#define LFSensor_3 A3
#define LFSensor_4 INT2  //OLD D10

#define SERVO_PIN    9  //servo connect to D3

#define SPEED 120
#define FAST_SPEED 120 
#define MID_SPEED 120

/*const int SPEED = 120;
const int FAST_SPEED = SPEED;
const int MID_SPEED = SPEED;
*/

PWMServo head;
/*motor control*/
void go_Back(int speed)  //Forward
{

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2,LOW);
   analogWrite(ENA,speed);
}
 
void go_Advance(int speed)  //Forward
{

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

  digitalWrite(IN1, LOW);
  digitalWrite(IN2,LOW);

  analogWrite(ENA,0);
}

bool FLG = 0;

void setup() {

 pinMode(ENA, OUTPUT); 
 pinMode(IN1, OUTPUT); 
 pinMode(IN2, OUTPUT); 
 pinMode(INT1, INPUT); 
 pinMode(INT2, INPUT); 
 if(FLG == 0){
  attachInterrupt(digitalPinToInterrupt(INT1), Handler, LOW);
  attachInterrupt(digitalPinToInterrupt(INT2), Handler, LOW);
 }
 else{
  attachInterrupt(digitalPinToInterrupt(INT1), Handler, LOW);
  attachInterrupt(digitalPinToInterrupt(INT2), Handler, LOW);
 }
 head.attach(SERVO_PIN);
 
     turn(FRONT);
     stop_Stop();
Serial.begin(9600);
 
}
 void Handler(){
  stop_Stop();
  delay(1000);
  char s[5]; // Sensor input array based on the pins;
  s[0] = digitalRead(LFSensor_0); // Left most sensor
  s[1] = digitalRead(LFSensor_1);
  s[2] = digitalRead(LFSensor_2);
  s[3] = digitalRead(LFSensor_3);
  s[4] = digitalRead(LFSensor_4); // Right most sensor
  if(s[0] != s[4]){
    if(s[0] == FLG){
      if(s[1] == FLG){
        turn(SHARP_RIGHT);
        go_Advance(FAST_SPEED);
        delay(DELAY_TIME);
        turn(SHARP_LEFT);
        return;
      }
      else{
        turn(RIGHT);
        go_Advance(MID_SPEED);
        delay(DELAY_TIME);
        turn(LEFT);
        turn(FRONT);
        return;
      }
    }
    else{
      if(s[3] == FLG){
        turn(SHARP_LEFT);
        go_Advance(FAST_SPEED);
        delay(DELAY_TIME);
        turn(SHARP_RIGHT);
        turn(FRONT);
        return;
      }
      else{
        turn(LEFT);
        go_Advance(MID_SPEED);
        delay(DELAY_TIME);
        turn(RIGHT);
        return;
      }
    }
  }
  else{
  stop_Stop();
  delay(200);
  return;
  }
 }
 
void loop() {
 go_Advance(MID_SPEED);
}
