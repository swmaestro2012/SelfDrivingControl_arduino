#include <MsTimer2.h>
#include <Servo.h>
const int pingPin = 2;
const int sensorPin1=5;
const int sensorPin2=4;
const int sensorPin3=3;
const int sensorPin4=2;
const int sensorPin5=1;
const int sensorPin6=0;
const int pulseInterval = 50;
Servo servo;
Servo motor;
int movecount=0;
int currentSpeed = 99;
int cnt=0;
long mv = 5000;
int front_left;
int front_center;
int front_right;
double side_left;
double side_right;
double rear;
double old_side_left;
double old_side_right;
double old_rear;
int approach_left=0;
int approach_right=0;
int approach_rear=0;
int side = 250;
int forcestop = 80;
int front = 400;
int front_stop=100;
int absoluteTime=0;
int RUNMODE = 0;//0 = FRONT_FREE 1 = REAR 2 = FRONT_FORCE
int OBSTACLE = 0; //0 = LEFT, 1 = CENTER, 2 = RIGHT
int HANDLE = 2; 
int CURRENT_HANDLE=2;
int val[6][5] = {
  0,};
void sendPulse() {
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(20);
  digitalWrite(pingPin, LOW);
}
void countTime()
{
  absoluteTime++;
  absoluteTime%=65536;
  if(absoluteTime%pulseInterval==0)
    sendPulse();
  if(movecount>0)
    movecount--;
}
void analogInput()
{
  val[0][cnt%5]=(analogRead(sensorPin1)*mv)/1024;
  val[1][cnt%5]=(analogRead(sensorPin2)*mv)/1024;
  val[2][cnt%5]=(analogRead(sensorPin3)*mv)/1024;
  val[3][cnt%5]=((double)((double)1/analogRead(sensorPin5))*mv);
  val[4][cnt%5]=((double)((double)1/analogRead(sensorPin4))*mv);
  val[5][cnt%5]=((double)((double)1/analogRead(sensorPin6))*mv);
  cnt++;
}
void processing()
{
  for(int i =0 ; i<5 ; i++)
  {
    front_left+=val[0][i];
    front_center+=val[1][i];
    front_right+=val[2][i];
    side_left+=val[3][i];
    side_right+=val[4][i];
    rear+=val[5][i];
  }
  front_left/=5; 
  front_center/=5; 
  front_right/=5; 
  side_left/=5;
  side_right/=5;
  rear/=5;
  if(side_left<old_side_left)
  {
    approach_left++;
  }
  else
  {
    approach_left=0;
  }
  if(side_right<old_side_right)
  {
    approach_right++;
  }
  else
  {
    approach_right=0;
  }
  if(rear<old_rear)
  {
    approach_rear++;
  }
  else
  {
    approach_rear=0;
  }
  old_side_left = side_left;
  old_side_right = side_right;
  old_rear = rear;

}
int analyze()
{
  if(CURRENT_HANDLE==2)
  {
    HANDLE=2;
    if((front_left<=forcestop || front_right<=forcestop) && front_center<=front_stop)
    {
      movecount=1500;
      return 1;
    }
    if(front_center<=front)
    {
      if(front_left<=side && front_right>side)
      {
        HANDLE=4;
      }
      else if(front_left>side && front_right<=side)
      {
        HANDLE=0;
      }
      else if(front_left>side && front_right>side)
      {
        HANDLE = (front_left>front_right?0:4);
      }
    }
    if(front_center>front)
    {
      if(front_left<=side)
      {      
        HANDLE=3;
        CURRENT_HANDLE=3;
        return 0;
      }
      else if(front_right<=side)
      {
        HANDLE=1;
        CURRENT_HANDLE=1;
        return 0;
      }
    }
    if(approach_left>25 && side_left<35.0)
    {
      HANDLE=3;
    }
    else if(approach_right>25 && side_right<35.0)
    {
      HANDLE=1;
    }
  }
  else if(CURRENT_HANDLE<2)
  {
    if(front_left<side_stop)
    {
      movecount=1000;
      return 1;
    }
    else if(front_left<side)
    {
      if(front_right<side)
        HANDLE=2;
      else
        HANDLE=3;
    }
  }
  else if(CURRENT_HANDLE>2)
  {
    if(front_right<side_stop)
    {
      movecount=1000;
      return 1;
    }
    else if(front_right<side)
    {
      if(front_left<side)
        HANDLE=4;
      else
        HANDLE=3;   
    }
  }
  CURRENT_HANDLE=HANDLE;
  return 0;
}
void steering()
{
  switch(HANDLE)
  {
  case 0:
    servo.write(180);
    break;
  case 1:
    servo.write(135);
    break;
  case 2:
    servo.write(90);
    break;
  case 3:
    servo.write(45);
    break;
  case 4:
    servo.write(0);
    break;
  }
}
void motorcontrol()
{
  switch(RUNMODE)
  {
  case 0:
    motor.write(96);
    break;
  case 1:
    if(movecount>0)
    {
      servo.write(90);
      motor.write(84);
    }
    else
    {
      RUNMODE=2;
      movecount=1000;
    }
    break; 
  case 2:
    if(movecount>0)
    {
      servo.write(90);
      motor.write(79);
    }
    else
    {
      if(side_right>100.0 || side_left>100.0)
      {        
        RUNMODE=3;
        movecount=2500;
      }
      else
      {
        RUNMODE=2;
        movecount=3000;
      }
    }
    break;
  case 3:
    if(movecount>0)
    {
      if(side_right<side_left)
        servo.write(180);
      else
        servo.write(0);  
      motor.write(97);
    }
    else
    {
      RUNMODE=0;
    }
    break;
  }
}
void serialout()
{
  Serial.print(front_left);
  Serial.print(" ");
  Serial.print(front_center);
  Serial.print(" ");
  Serial.print(front_right);
  Serial.print(" ");
  Serial.print(side_left);
  Serial.print(" ");
  Serial.print(side_right);
  Serial.print(" ");
  Serial.print(rear);
  Serial.print(" ");
  Serial.print(approach_left);
  Serial.println(" ");
}
void setup() {
  Serial.begin(9600);
  pinMode(2,OUTPUT);
  MsTimer2::set(1,countTime);
  MsTimer2::start();
  servo.attach(3);
  motor.attach(5);
  for(int i=0 ; i<5 ; i++)
  {
    val[0][i] = (analogRead(sensorPin1)*mv)/1024;
    val[1][i] = (analogRead(sensorPin2)*mv)/1024;
    val[2][i] = (analogRead(sensorPin3)*mv)/1024;
    val[3][i] = ((double)((double)1/analogRead(sensorPin5))*mv);
    val[4][i] = ((double)((double)1/analogRead(sensorPin4))*mv);
    val[5][i] = ((double)((double)1/analogRead(sensorPin6))*mv);
  }
}

void loop() {
  analogInput();
  processing();
  if(RUNMODE==0)
  {
    RUNMODE = analyze();
    steering();
  }
  motorcontrol();
  serialout();
  front_left=0; 
  front_center=0; 
  front_right=0; 
  side_left=0;
  side_right=0;
  rear=0;
}
































