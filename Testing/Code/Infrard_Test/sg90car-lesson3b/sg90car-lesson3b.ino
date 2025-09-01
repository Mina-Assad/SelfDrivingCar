#include <PWMServo.h>

#define IN1 7
#define IN2 8
#define ENA 5       //  ENA pin
 

#define FRONT 50        // steering to front 
int SHARP_RIGHT=FRONT+50;
int SHARP_LEFT=FRONT-50;
int  RIGHT=FRONT+20;
int  LEFT=FRONT-20;
  
#define DELAY_TIME 1000   
  
#define LFSensor_0 A0  //OLD D3
#define LFSensor_1 A1
#define LFSensor_2 A2
#define LFSensor_3 A3
#define LFSensor_4 A4  //OLD D10

#define SERVO_PIN    9  //servo connect to D3

/*#define SPEED 150
#define FAST_SPEED 200 
#define MID_SPEED 180
#define SERVO_PIN    9  //servo connect to D3*/

const int SPEED = 110;
const int FAST_SPEED = SPEED;
const int MID_SPEED = SPEED;

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

  
void setup() {

 pinMode(ENA, OUTPUT); 
 pinMode(IN1, OUTPUT); 
 pinMode(IN2, OUTPUT); 


 head.attach(SERVO_PIN);
 
     turn(FRONT);
  
     stop_Stop();
Serial.begin(9600);
 
}

 
void loop() {
 auto_tracking();
 //turn(70);
}
char sensor[5];
 /*read sensor value string, 1 stands for black, 0 starnds for white, i.e 10000 means the first sensor(from left) detect black line, other 4 sensors detected white ground */
 
void auto_tracking(){
 int sensorvalue=32;
  sensor[0]= !digitalRead(LFSensor_0);
 
  sensor[1]=!digitalRead(LFSensor_1);
 
  sensor[2]=!digitalRead(LFSensor_2);
 
  sensor[3]=!digitalRead(LFSensor_3);
 
  sensor[4]=!digitalRead(LFSensor_4);
  sensorvalue +=sensor[0]*16+sensor[1]*8+sensor[2]*4+sensor[3]*2+sensor[4];
  
  String sensorval= String(sensorvalue,BIN).substring(1,6);
  
 Serial.print("VALUE=");
  Serial.println(sensorval);
  if (   sensorval=="01000" || sensorval=="10000"  || sensorval=="11000" )
  { 
    turn(SHARP_LEFT);
    go_Advance(FAST_SPEED);
  }
  if ( sensorval=="01100" || sensorval=="11100"  || sensorval=="11110" )
  {
    turn(LEFT);
    go_Advance(MID_SPEED);
  }
  if ( sensorval=="00010"  ||  sensorval=="00011" ||  sensorval=="00001"  ){ //The black line is  on the right of the car, need  right turn 
    turn(SHARP_RIGHT);
    go_Advance(FAST_SPEED);
  }
  if ( sensorval=="00110" || sensorval=="00111" || sensorval=="01111")
  {
    turn(RIGHT);
    go_Advance(MID_SPEED);
  }
   if (sensorval=="00100"    || sensorval=="01110")
  {
    turn(FRONT);
    go_Advance(SPEED);
  }
  if (sensorval=="11111"){
    stop_Stop();   //The car front touch stop line, need stop
    turn(FRONT);
  }

}
