#include <PWMServo.h>
#define IN1 7
#define IN2 8
#define ENA 5       //  ENA pin
#define INT1 2
#define INT2 3

#define FRONT 65        // steering to front 
int SRIGHT = 150;
int SLEFT = -150;
int  RIGHT = 100;
int  LEFT = -100;
  
int DELAY_TIME = 1000;
int DELAY_TIME2 = 2*DELAY_TIME;

  
#define LFSensor_0 INT1  //OLD D3
#define LFSensor_1 A1
#define LFSensor_2 A2
#define LFSensor_3 A3
#define LFSensor_4 INT2  //OLD D10

#define SERVO_PIN    9  //servo connect to D3

#define SPEED 120
#define FAST SPEED + 20
#define MID SPEED + 10

#define trigger 0

bool FLG = 0;     // flag for the inteerupt
int D = FRONT;    // direction so the car can turn to
int timeCNT = 200;// time counter variable
bool R = 1;       // the random variable for an obstacile in fornt of the car

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
        FLG = 11;
        return;
      }
      else{
        FLG = 1;
        return;
      }
    }
    else{
      Serial.println("right");
      if(s[3] == trigger){
        FLG = 22;
        return;
      }
      else{
        FLG = 2;
        return;
      }
    }
  }
  else{
    Serial.println("both");
    FLG = 3;
    return;
  }
}
void loop() {
  switch(FLG){
    case 0:
      go_Advance(SPEED);
      break;
    case 1:
      go_Back(MID);
      delay(1000);
      stop_Stop();
      delay(200);
      turn(RIGHT);
      go_Advance(FAST);
      delay(1000);
      stop_Stop();
      delay(200);
      turn(FRONT);
      FLG = 0;
      break;
    case 11:
      go_Back(MID);
      delay(1000);
      stop_Stop();
      delay(200);
      turn(RIGHT);
      go_Advance(FAST);
      delay(2000);
      stop_Stop();
      delay(200);
      turn(FRONT);
      FLG = 0;
      break;
    case 2:
      go_Back(MID);
      delay(1000);
      stop_Stop();
      delay(200);
      turn(LEFT);
      go_Advance(FAST);
      delay(1000);
      stop_Stop();
      delay(200);
      turn(FRONT);
      FLG = 0;
      break;
    case 22:
      go_Back(MID);
      delay(1000);
      stop_Stop();
      delay(200);
      turn(LEFT);
      go_Advance(FAST);
      delay(2000);
      stop_Stop();
      delay(200);
      turn(FRONT);
      FLG = 0;
      break;
    case 3:
      stop_Stop();
      break;
    default:
      FLG = 0;
      break;
  }
}