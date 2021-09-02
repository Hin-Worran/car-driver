/**
 * version: 1.0.1
 * function: drive a robot with obstacle avoidance system 
 * author: hin
 * date: 2021/8/30 10:07
 */

#include <Servo.h>
#include "car.h"
// pin脚定义
const int pinLB = 11; //左退
const int pinLF = 4;  //左前
const int pinRB = 10; //右退
const int pinRF = 9;  //右前

int inputPin = A1;  //超声波Echo引脚
int outputPin = A0; //超声波Trig引脚
int servoPin = 12;  //舵机信息号引脚

const int sensorMid = 3;   //中感測器輸入腳  检测到黑色发送高电平
const int sensorLeft = 7;  //左感測器輸入腳
const int sensorRight = 8; //右感測器輸入腳
const int buttonPin = 2;   //绿色按钮

//马达脚
int MotorRPWM = 6;
int MotorLPWM = 5;

//变量定义
Servo myservo; //定义舵机
int basicPWM = 120;
int modifiedPWM = 0;        //用以解决左右轮速差问题
int turningTime = 350;      //转直角弯时间
int turningPoint = 10;      //转弯的临界距离，单位厘米
int turningPoint_side = 14; //侧面的转弯临界距离
bool isStart = false;       //启动状态

/**
 * 单击按钮
 */
void clickButton()
{
  static long clickTime = millis();
  if (millis() - clickTime > 100) //除颤
  {
    clickTime = millis();
    Serial.println("button clicked");
    isStart = !isStart;
  }
}

/*
  设置马达
*/
void setMotorPWM(int basicPWM, int modifiedPWM = 0)
{
  analogWrite(MotorRPWM, basicPWM);               //設置右馬達的PWM
  analogWrite(MotorLPWM, basicPWM + modifiedPWM); //设置左马达的PWM,并加上调整值
}

/*
   左右看，找路
*/
void findOtherWay()
{
  if (isStucked())
  {
    backward();
    delay(100);
    pause();
  }
  if (askDistance(90) > turningPoint)
  {
    forward();
    delay(100);
    pause();
  }
  else if (askDistance(0) > turningPoint_side)
  {
    turnRight(); //若右边有空间，右转
  }
  else if (askDistance(180) > turningPoint_side)
  {
    turnLeft(); //若左边有空间，左转
  }
  else
  {
    turnAround(); //其他情况，掉头
  }
}

/*
  前进
*/
void forward()
{
  digitalWrite(pinRB, LOW);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, LOW);
  digitalWrite(pinLF, HIGH);
  setMotorPWM(basicPWM, modifiedPWM);
}

/*
  后退
*/
void backward()
{
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, LOW);
  setMotorPWM(basicPWM, modifiedPWM);
}

/*
  左转
*/
void turnLeft()
{
  digitalWrite(pinRB, LOW);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, LOW);
  setMotorPWM(basicPWM, modifiedPWM);
  delay(turningTime);
  pause();
}

/*
  右转
*/
void turnRight()
{
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, LOW);
  digitalWrite(pinLF, HIGH);
  setMotorPWM(basicPWM, modifiedPWM);
  delay(turningTime);
  pause();
}

/**
 * 转动一定角度，正数向右，负数向左
 */
void turnAngle(int angle)
{
  double timePerAngle = turningTime / 90.0;
  if (angle < 0)
  {
    digitalWrite(pinRB, LOW);
    digitalWrite(pinRF, HIGH);
    digitalWrite(pinLB, HIGH);
    digitalWrite(pinLF, LOW);
    setMotorPWM(basicPWM, modifiedPWM);
    delay(timePerAngle * -angle);
    pause();
  }
  else if (angle > 0)
  {
    digitalWrite(pinRB, HIGH);
    digitalWrite(pinRF, LOW);
    digitalWrite(pinLB, LOW);
    digitalWrite(pinLF, HIGH);
    setMotorPWM(basicPWM, modifiedPWM);
    delay(timePerAngle * angle);
    pause();
  }
  else
  {
    pause();
  }
}

/*
  掉头
*/
void turnAround()
{
  digitalWrite(pinRB, LOW);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, LOW);
  setMotorPWM(basicPWM, modifiedPWM);
  delay(turningTime * 2);
  pause();
}

/*
  暂停
*/
void pause()
{
  setMotorPWM(0);
}

/*
  通过超声波测距，输入角度，返回距离（厘米）
*/
int askDistance(int angle)
{
  myservo.write(angle);
  delay(250);
  digitalWrite(outputPin, LOW); // 超声波发射低电平2μs
  delayMicroseconds(2);
  digitalWrite(outputPin, HIGH); // 超声波发射高电平10μs
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW);                     // 维持超声波发射低电平
  float distance = pulseIn(inputPin, HIGH) / 58.00; //读取相差时间,将时间转化为距离（单位：厘米）
  return (distance);
}

/**
 * 循迹
 */
void trackRoad()
{
  int SL = digitalRead(sensorLeft);
  int SR = digitalRead(sensorRight);
  int SM = digitalRead(sensorMid);
  int delayTime = 200;

  // Serial.print("L:");
  // Serial.println(SL);
  // Serial.print("R:");
  // Serial.println(SR);

  if (SR == HIGH)
  {
    turnAngle(30);
  }
  else if (SR == LOW && SM == HIGH) //左右白，中间黑，前进
  {
    forward();
    delay(delayTime);
    pause();
  }
  // else if (SL == LOW && SR == HIGH && SM == LOW) //左白右黑,快速右转
  // {
  //   turnAngle(30);
  // }
  else if (SL == HIGH && SR == LOW && SM == LOW) //左黑右白,快速左转
  {
    turnAngle(-30);
  }
  // else if (SL == HIGH && SR == HIGH && SM == HIGH) //都是黑，停止
  // {
  //   pause();        //停止
  //   findOtherWay(); //遇到十字路口，找路
  // }
  else if (SL == LOW && SR == LOW && SM == LOW) //全白，前进
  {
    // forward();
    // delay(delayTime);
    // pause();
    pause();
    findOtherWay();
  }
}

/**
 * 判断是否卡住了
 */
bool isStucked()
{
  static unsigned long lastMillis = millis(); //静态记录历史时间
  static int lastDistance = askDistance(90);  //历史距离
  unsigned long thisMillis = millis();        //当前时间
  int thisDistance = askDistance(90);         //当前距离
  if (thisMillis - lastMillis > 1500)         //每隔1.5秒进行判断
  {
    if (abs(thisDistance - lastDistance) < 1) //若1秒移动距离小于1cm，则认为stucked
    {
      lastDistance = thisDistance; //update status
      lastMillis = thisMillis;
      return true;
    }
    else
    {
      lastDistance = thisDistance;
      lastMillis = thisMillis;
      return false;
    }
  }
  else
  {
    return false;
  }
}

/**
 * observe and choose a way to go
 */
void observe()
{
  if (askDistance(0) > turningPoint_side) // if right side reachable, turn right
  {
    // noInterrupts();
    pause();
    delay(100);
    turnRight();
    // interrupts();
  }
  else if (askDistance(90) > turningPoint) // if front reachable, no need to do anything
  {
    forward();
  }
  else if (askDistance(180) > turningPoint_side) // if left side reachable, turn left
  {
    // noInterrupts();
    pause();
    delay(100);
    turnLeft();
    // interrupts();
  }
  else // if no direction is reachable turn around
  {
    // noInterrupts();
    pause();
    delay(100);
    backward();
    delay(200);
    pause();
    delay(100);
    turnAround();
    // interrupts();
  }
}

/**
 * drive the car back to road
 */
void backToRoad()
{
  pause();
  while (digitalRead(sensorMid) != HIGH)
  {
    if (digitalRead(sensorLeft) == HIGH)
    {
      turnAngle(-30);
    }
    else if (digitalRead(sensorRight) == HIGH)
    {
      turnAngle(30);
    }
  }
}

void setup()
{
  Serial.begin(9600); //波特率

  //设pinMode
  pinMode(pinLB, OUTPUT);
  pinMode(pinLF, OUTPUT);
  pinMode(pinRB, OUTPUT);
  pinMode(pinRF, OUTPUT);
  pinMode(inputPin, INPUT);
  pinMode(outputPin, OUTPUT);
  pinMode(sensorLeft, INPUT);
  pinMode(sensorRight, INPUT);
  pinMode(sensorMid, INPUT);
  pinMode(buttonPin, INPUT);
  pinMode(MotorLPWM, OUTPUT); //PWM
  pinMode(MotorRPWM, OUTPUT);

  myservo.attach(servoPin); //定义舵机信息号引脚

  //初始化
  isStart = false;                                                         //初始时启动状态为false
  myservo.write(90);                                                      //舵机摆正
  attachInterrupt(digitalPinToInterrupt(buttonPin), clickButton, RISING); //绑定按钮中断
  // attachInterrupt(digitalPinToInterrupt(sensorMid), backToRoad, FALLING); //红外传感器中断
  Serial.println("setup done");
}

void loop()
{
  //启动判断
  if (isStart)
  {
    observe();
    // if (askDistance(90) > turningPoint)
    // {
    //   trackRoad(); //循迹
    // forward(); //前方有空间，前进
    if (isStucked()) //卡车处理
    {
      pause();
      backward(); //后退
      delay(250);
      pause();        //停下来
      findOtherWay(); //重新找路
    }
  }
  // else
  // {
  //   pause();
  //   findOtherWay();
  // }
}
