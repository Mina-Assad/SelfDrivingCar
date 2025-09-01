/*  ___   ___  ___  _   _  ___   ___   ____ ___  ____  
 * / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \ 
 *| |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
 * \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
 *                  (____/ 
 * OSOYOO SG90 servo steering robot car Lesson 2
 * Tutorial URL https://osoyoo.com/?p=37134
 * CopyRight www.osoyoo.com

 *   
 * 
 */
#include <PWMServo.h>
#define IN1 7
#define IN2 8
#define ENA 5        //  ENA pin
#define INT1 2
#define INT2 3
 
#define FRONT 55        // steering to front 
int SHARP_RIGHT=FRONT+33;
int SHARP_LEFT=FRONT-40;

#define DELAY_TIME 1200     
#define BACK_TIME 1000  
#define SPEED 190
#define HI_SPEED 220
#define SERVO_PIN 9  //servo connect to D9
 
#define LFSensor_0 INT1  //OLD D3
#define LFSensor_1 24 //A1
#define LFSensor_2 25 //A2
#define LFSensor_3 26 //A3
#define LFSensor_4 INT2  //OLD D10

PWMServo head;
/*motor control*/
void go_Back(int speed)  //Forward
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2,LOW); 
  analogWrite(ENA,speed);
}
 
void  go_Advance(int speed)  //Forward
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
 go_Advance(SPEED);
 delay(DELAY_TIME); 
 
 go_Back(SPEED);
 delay(DELAY_TIME);

 go_Advance(HI_SPEED);
 turn(SHARP_LEFT);
 delay(DELAY_TIME);

 turn(SHARP_RIGHT);
 delay(DELAY_TIME);
stop_Stop();//stop move
delay(100);
 turn(FRONT);
 go_Back(HI_SPEED);
 delay(BACK_TIME);
 turn(SHARP_RIGHT);
  delay(BACK_TIME);
 turn(SHARP_LEFT);

delay(BACK_TIME);

turn(FRONT);
stop_Stop();//stop move
 // Serial.begin(9600);

}

 
void loop() {
  go_Advance(120);
}
