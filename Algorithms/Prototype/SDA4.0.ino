/*Note: changes/rewires:
speedPinR   3 -> 5
Echo_PIN    2 -> 4
Trig_PIN    10 -> 10*
LFSensor_0(IR1)   A0 -> 2(INT0)
LFSensor_4(IR5)   A4 -> 3(INT1)
*/
#include <Servo.h>
#define powerIndex 1.9
#define speedPinR 5   // RIGHT PWM pin connect MODEL-X ENA
#define RightDirectPin1  12    //  Right Motor direction pin 1 to MODEL-X IN1 
#define RightDirectPin2  11    // Right Motor direction pin 2 to MODEL-X IN2
#define speedPinL 6        //  Left PWM pin connect MODEL-X ENB
#define LeftDirectPin1  7    // Left Motor direction pin 1 to MODEL-X IN3
#define LeftDirectPin2  8   ///Left Motor direction pin 1 to MODEL-X IN4

#define SERVO_PIN     9  //servo connect to D9

#define Echo_PIN    4 // Ultrasonic Echo pin connect to D11
#define Trig_PIN    10  // Ultrasonic Trig pin connect to D12

#define BUZZ_PIN     13
#define FAST_SPEED  250/powerIndex     //both sides of the motor speed 250
#define SPEED  110/powerIndex     //both sides of the motor speed 100
#define TURN_SPEED  200/powerIndex     //both sides of the motor speed 200
#define BACK_SPEED  255/powerIndex     //back speed 255

#define INT0  2
#define INT1  3

#define LFSensor_0 INT0  //Left
#define LFSensor_1 A1
#define LFSensor_2 A2
#define LFSensor_3 A3
#define LFSensor_4 INT1  //Rignt

int leftscanval, centerscanval, rightscanval, ldiagonalscanval, rdiagonalscanval;
const int distancelimit = 20; //distance limit for obstacles in front           
const int sidedistancelimit = 10; //minimum distance in cm to obstacles at both sides (the car will allow a shorter distance sideways)
int distance;
int numcycles = 0;
const int turntime = 500;  // 45 degree turn
const int backtime = 300; 

Servo head;

int FLG = 0;
int trigger = 0;
int timerCNT = 0;
bool R = 0;
bool M = 0;
/*motor control*/
void go_Advance(void)  //Forward
{
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW);
}
void go_Left()  //Turn left
{
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,HIGH);
  M = 0;
}
void go_Right()  //Turn right
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,HIGH);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW);
  M = 1;
}
void go_Back()  //Reverse
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,HIGH);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,HIGH);
}
void stop_Stop()    //Stop
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,LOW);
  set_Motorspeed(0,0);
}

/*set motor speed */
void set_Motorspeed(int speed_L,int speed_R)
{
  analogWrite(speedPinL,speed_L); 
  analogWrite(speedPinR,speed_R);   
}

void Move(char action, int speed, int time){
  set_Motorspeed(speed,speed);
  switch (action){
    case 's':
      stop_Stop();
      break;
    case 'f':
      go_Advance();
      break;
    case 'b':
      go_Back();
      break;
    case 'l':
      go_Left();
      break;
    case 'r':
      go_Right();
      break;
    default:
      return;
  }
  delay(time);
  return;
}

void buzz_ON()   //open buzzer
{
  
  for(int i=0;i<100;i++)
  {
   digitalWrite(BUZZ_PIN,LOW);
   delay(2);//wait for 1ms
   digitalWrite(BUZZ_PIN,HIGH);
   delay(2);//wait for 1ms
  }
}
void buzz_OFF()  //close buzzer
{
  digitalWrite(BUZZ_PIN, HIGH);
  
}
void alarm(){
   buzz_ON();
 
   buzz_OFF();
}

/*detection of ultrasonic distance*/
int watch(){
  long echo_distance;
  digitalWrite(Trig_PIN,LOW);
  delayMicroseconds(5);                                                                              
  digitalWrite(Trig_PIN,HIGH);
  delayMicroseconds(15);
  digitalWrite(Trig_PIN,LOW);
  echo_distance=pulseIn(Echo_PIN,HIGH);
  echo_distance=echo_distance*0.01657; //how far away is the object in cm
  //Serial.println((int)echo_distance);
  return round(echo_distance);
}
//Meassures distances to the right, left, front, left diagonal, right diagonal and asign them in cm to the variables rightscanval, 
//leftscanval, centerscanval, ldiagonalscanval and rdiagonalscanval (there are 5 points for distance testing)
int watchsurrounding(){
/*  obstacle_status is a binary integer, its last 5 digits stands for if there is any obstacles in 5 directions,
 *   for example B101000 last 5 digits is 01000, which stands for Left front has obstacle, B100111 means front, right front and right ha
 */
 
int obstacle_status =B100000;
  centerscanval = watch();
  if(centerscanval<distancelimit){
    stop_Stop();
    alarm();
    obstacle_status  =obstacle_status | B100;
    }
  head.write(120);
  delay(100);
  ldiagonalscanval = watch();
  if(ldiagonalscanval<distancelimit){
    stop_Stop();
    alarm();
     obstacle_status  =obstacle_status | B1000;
    }
  head.write(170); //Didn't use 180 degrees because my servo is not able to take this angle
  delay(300);
  leftscanval = watch();
  if(leftscanval<sidedistancelimit){
    stop_Stop();
    alarm();
     obstacle_status  =obstacle_status | B10000;
    }

  head.write(90); //use 90 degrees if you are moving your servo through the whole 180 degrees
  delay(100);
  centerscanval = watch();
  if(centerscanval<distancelimit){
    stop_Stop();
    alarm();
    obstacle_status  =obstacle_status | B100;
    }
  head.write(40);
  delay(100);
  rdiagonalscanval = watch();
  if(rdiagonalscanval<distancelimit){
    stop_Stop();
    alarm();
    obstacle_status  =obstacle_status | B10;
    }
  head.write(0);
  delay(100);
  rightscanval = watch();
  if(rightscanval<sidedistancelimit){
    stop_Stop();
    alarm();
    obstacle_status  =obstacle_status | 1;
    }
  head.write(90); //Finish looking around (look forward again)
  delay(300);
   String obstacle_str= String(obstacle_status,BIN);
  obstacle_str= obstacle_str.substring(1,6);
  Serial.println(obstacle_str);
  
  if(obstacle_str == "00000" || obstacle_str == "10001" || obstacle_str == "11011" ){
    return 0;
  }
  else if(obstacle_str == "10000"){
    return 1;
  }
  else if(obstacle_str == "01000" || obstacle_str == "11000" || obstacle_str == "11100" || obstacle_str == "01100" || obstacle_str == "10100"){
    return 2;
  }
  else if(obstacle_str == "00100" || obstacle_str == "01110" ){
    return 3;
  }
  else if(obstacle_str == "00010" || obstacle_str == "00011" || obstacle_str == "00111" || obstacle_str == "00110" || obstacle_str == "00101"){
    return 4;
  }
  else if(obstacle_str == "00001"){
    return 5;
  }
  return 6;
}

void Push(){
  set_Motorspeed(FAST_SPEED,FAST_SPEED);
  go_Advance();
  delay(30);
  set_Motorspeed(SPEED,SPEED);
}

void surveilling(int x, bool& y){
  Serial.println(x);
  switch(x){
    case 0:
      Serial.println("go Froward");
      break;
    case 1:
      Move('r', TURN_SPEED, turntime);
      stop_Stop();
      Serial.println("go Right");
      break;
    case 2:
      Move('r', TURN_SPEED, 2*turntime);
      stop_Stop();
      Serial.println("go Sharp Right");
      break;
    case 3:
      if(y == 1){
        Move('r', TURN_SPEED, 2*turntime);
        stop_Stop();
        Serial.println("go Sharp Right");
      }
      else{
        Move('l', TURN_SPEED, 2*turntime);
        stop_Stop();
        Serial.println("go Sharp Left");
      }
      y = !y;
      Serial.println(y);
      break;
    case 4:
      Move('l', TURN_SPEED, 2*turntime);
      stop_Stop();
      Serial.println("go Sharp Left");
      break;
    case 5:
      Move('l', TURN_SPEED, turntime);
      stop_Stop();
      Serial.println("go Left");
      break;
    default:
      alarm();
      alarm();
      Serial.println("BLOCKED");
      break;
  }
  Push();
  return;
}

void selfDriving(int& x){ //Main Algorithm
  switch(x){
    case 0:
      if(timerCNT == 100){
        surveilling(watchsurrounding(), R);
        timerCNT = 0;
        return;
      }
      go_Advance();
      timerCNT++;
      return;
    case 1:
      Move('s', FAST_SPEED, 100);
      alarm();
      Serial.println("go left!!!");
      Move('b', BACK_SPEED, backtime);
      stop_Stop();
      Move('r', TURN_SPEED, turntime);
      stop_Stop();
      x = 0;
      surveilling(watchsurrounding(), R);
      return;
    case 2:
      Move('s', FAST_SPEED, 100);
      alarm();
      Serial.println("go left!!!");
      Move('b', BACK_SPEED, backtime);
      stop_Stop();
      Move('l', TURN_SPEED, turntime);
      stop_Stop();
      x = 0;
      surveilling(watchsurrounding(), R);
      return;
    case 3:
      if(M == 0){
      Move('s', FAST_SPEED, 100);
      alarm();
      Serial.println("go left!!!");
      Move('b', BACK_SPEED, backtime);
      stop_Stop();
      Move('r', TURN_SPEED, 2.5*turntime);
      stop_Stop();
      x = 0;
      surveilling(watchsurrounding(), R);
      return;
      }
      else{
        Move('s', FAST_SPEED, 100);
      alarm();
      Serial.println("go left!!!");
      Move('b', BACK_SPEED, backtime);
      stop_Stop();
      Move('l', TURN_SPEED, 2.5*turntime);
      stop_Stop();
      x = 0;
      surveilling(watchsurrounding(), R);
      return;
      }
    default:
      alarm();
      alarm();
      x = 0;
      return;
  }
}

void setup() {
  /*setup L298N pin mode*/
  pinMode(RightDirectPin1, OUTPUT); 
  pinMode(RightDirectPin2, OUTPUT); 
  pinMode(speedPinL, OUTPUT);  
  pinMode(LeftDirectPin1, OUTPUT);
  pinMode(LeftDirectPin2, OUTPUT); 
  pinMode(speedPinR, OUTPUT); 
  stop_Stop();//stop move
  /*init HC-SR04*/
  pinMode(Trig_PIN, OUTPUT); 
  pinMode(Echo_PIN,INPUT); 
  /*init buzzer*/
  pinMode(BUZZ_PIN, OUTPUT);
  digitalWrite(BUZZ_PIN, HIGH);  
  buzz_OFF();

  pinMode(INT0, INPUT); 
  pinMode(INT1, INPUT);
  attachInterrupt(digitalPinToInterrupt(INT0), Handler, FALLING);
  attachInterrupt(digitalPinToInterrupt(INT1), Handler, FALLING);

  digitalWrite(Trig_PIN,LOW);
  /*init servo*/
  head.attach(SERVO_PIN); 
  head.write(90);
   delay(2000);

  set_Motorspeed(SPEED,SPEED);

  Serial.begin(9600);

  surveilling(watchsurrounding(), R);

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
      FLG = 1;
      return;
    }
    else {
      Serial.println("right");
      FLG = 2;
      return;
    }
  }
  else{
    Serial.println("both");
    FLG = 3;
  }
  if(s[2] == trigger){
    FLG = 3;
  }
  Serial.println(FLG);
}

void loop() {
  selfDriving(FLG);
}