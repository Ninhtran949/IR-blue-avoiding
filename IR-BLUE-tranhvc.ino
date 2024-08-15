#include <AFMotor.h>
#include <NewPing.h>
#include <Servo.h>
#include <IRremote.hpp>


#define ECHO_PIN A0              //noi chan A0 (Uno)
#define TRIG_PIN A1              //Noi chan A1 (Uno)
#define MAX_DISTANCE 200 
#define MAX_SPEED 235 // sets speed of DC  motors
#define MAX_SPEED_OFFSET 20  

#define F 3877175040// Lệnh điều khiển tiến
#define B 2907897600// Lệnh điều khiển lùi
#define L 4144561920// Lệnh điều khiển rẽ trái
#define R 2774204160// Lệnh điều khiển rẽ phải
#define S 3810328320// Lệnh điều khiển dừng
#define X 3910598400// Lệnh tránh vật cản
#define x 4061003520// Lệnh Thoát tránh vật cản

uint32_t temp = 0;
IRrecv myIR(10);   // IR receiver object

AF_DCMotor motor2(4);  //motor-phai-noi pin 5 L293D
AF_DCMotor motor1(3); //motor- trai- noi pin 6 L293D

Servo myservo;

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); 
boolean goesForward=false;
int distance = 100;
int speedSet = 0;
char command; 

int dugme = 1;

void setup() 
{   
  
  
  myservo.attach(9);  
  myservo.write(90); 
  myservo.attach(9, 800, 2100); // -15  195 do
  
  delay(2000);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  
  myIR.begin(10, ENABLE_LED_FEEDBACK);   // Bắt đầu nhận tín hiệu từ remote hồng ngoại
  Serial.begin(9600); //Set the baud rate to your Bluetooth module
}

void loop(){


  if (Serial.available() > 0) {
    command = Serial.read();
    Serial.println(command);
    Stop();
    switch (command) {
      case 'F':
        forward();
        break;
      case 'B':
        back();
        break;
      case 'L':
        left();
        break;
      case 'R':
        right();
        break;
      case 'S':
        Stop();
        break;
      case 'X':  avoiding(); break;
      case 'x':  stopAvoiding(); break;
      default:
        // Xử lý các lệnh khác nếu cần thiết
        break;
    }

       // Tiếp tục nhận tín hiệu
  }
    if (myIR.decode()) {
    Serial.println("helloir");
    temp = myIR.decodedIRData.decodedRawData;

    switch (temp) {
      case F:
        forward();
        break;
      case B:
        back();
        break;
      case L:
        left();
        break;
      case R:
        right();
        break;
      case S:
        Stop();
        break;
      case X:  avoiding(); break;
      case x:  stopAvoiding(); break;
      default:
        // Xử lý các lệnh khác nếu cần thiết
        break;
    }

    myIR.resume();   // Tiếp tục nhận tín hiệu
  }
}


void forward()
{
  motor1.setSpeed(255); //Define maximum velocity
  motor1.run(FORWARD); //rotate the motor clockwise
  motor2.setSpeed(200); //Define maximum velocity
  motor2.run(FORWARD); //rotate the motor clockwise
}

void back()
{
  motor1.setSpeed(255); 
  motor1.run(BACKWARD); 
  motor2.setSpeed(200); 
  motor2.run(BACKWARD); 
}

void left()
{
  motor1.setSpeed(255); 
  motor1.run(FORWARD); 
  motor2.setSpeed(200);
  motor2.run(BACKWARD); 
}

void right()
{
  motor1.setSpeed(200);
  motor1.run(BACKWARD); 
  motor2.setSpeed(255);  
  motor2.run(FORWARD); 
}

void Stop()
{
  motor1.setSpeed(0);
  motor1.run(RELEASE); 
  motor2.setSpeed(0);
  motor2.run(RELEASE); 
}

void stopAvoiding()
{
  dugme = 0;
  moveStop();
}

void avoiding()
{
  long duration, distance;
  int distanceR = 0;
  int distanceL = 0;

  while(dugme == 1)
  { 
    distance = readPing();
    delay(40);
    if(distance <= 26)
    {
      moveStop();
      delay(100);
      moveBackward();
      delay(150);
      moveStop();
      delay(200);
      distanceR = lookRight();
      delay(200);
      distanceL = lookLeft();
      delay(200);
      if(distanceR >= distanceL)
      {
        turnRight();
        moveStop();
      }
      else
      {
        turnLeft();
        moveStop();
      }
    }
    else
    {
      moveForward();
    }
  }
}

// void onLED()
// {
//  digitalWrite(onfar, HIGH);
// }

// void offLED()
// {
//  digitalWrite(onfar, LOW);
// }

int lookRight()
{
  myservo.write(40); 
  delay(500);
  int distance = readPing();
  delay(100);
  myservo.write(90); 
  return distance;
}

int lookLeft()
{
  myservo.write(160); 
  delay(500);
  int distance = readPing();
  delay(100);
  myservo.write(90); 
  return distance;
}

int readPing() { 
  delay(70);
  int cm = sonar.ping_cm();
  if(cm == 0)
  {
    cm = 250;
  }
  return cm;
}

void moveStop() {
  motor1.run(RELEASE); 
  motor2.run(RELEASE);
} 

void moveForward() {
  if(!goesForward)
  {
    goesForward = true;
    motor1.run(FORWARD);      
    motor2.run(FORWARD);   
    for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2) // slowly bring the speed up to avoid loading down the batteries too quickly
    {
      motor1.setSpeed(speedSet);
      motor2.setSpeed(speedSet + MAX_SPEED_OFFSET);
      delay(5);
    }
  }
}

void moveBackward() {
  goesForward = false;
  motor1.run(BACKWARD);      
  motor2.run(BACKWARD);  
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2) // slowly bring the speed up to avoid loading down the batteries too quickly
  {
    motor1.setSpeed(speedSet);
    motor2.setSpeed(speedSet + MAX_SPEED_OFFSET);
    delay(5);
  }
}  

void turnRight() {
  motor2.run(FORWARD);
  motor1.run(BACKWARD);     
  delay(300);
  motor2.run(FORWARD);      
  motor1.run(FORWARD);      
} 

void turnLeft() {
  motor2.run(BACKWARD);     
  motor1.run(FORWARD);     
  delay(300);
  motor2.run(FORWARD);     
  motor1.run(FORWARD);
}

