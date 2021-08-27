/**
 * version: 1.0
 * function: drive a robot with obstacle avoidance system 
 * author: hin
 * date: 2021/8/14 17:34
 */

#include <Servo.h>
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

//函数声明

void forward();
void backward();
void turnLeft();
void turnRight();
void turnAround();
void setMotorPWM(int, int);
void pause();
void findOtherWay();
void getThisState();
void saveState();
void startIt();
int askDistance(int angle);
void trackRoad();

//变量定义
Servo myservo; //定义舵机
int basicPWM = 120;
int modifiedPWM = 0;                      //用以解决左右轮速差问题
int turningTime = 350;                    //转弯时间
int turningPoint = 30;                    //转弯的临界距离，单位厘米
unsigned long lastState[2], thisState[2]; //[0]:时间，[1]:距离
bool isStart = false;                     //启动状态

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
  getThisState();                                                     //获取当前状态
  saveState();                                                        //暂存当前状态
  isStart = false;                                                    //初始时启动状态为false
  myservo.write(90);                                                  //舵机摆正
  attachInterrupt(digitalPinToInterrupt(buttonPin), startIt, RISING); //绑定中断
  attachInterrupt(digitalPinToInterrupt(sensorMid), trackRoad, FALLING);

  Serial.println("setup done");
}

void loop()
{
  //启动判断
  if (isStart)
  {
    getThisState(); //获取当前状态

    //打印当前状态
    // Serial.print(thisState[0]);
    // Serial.print("\t");
    // Serial.print(thisState[1]);
    // Serial.print("\n");

    if (thisState[1] > turningPoint)
    {

      // void trackRoad();
      forward(); //前方有空间，前进
      getThisState();
      //每隔1.5秒判断
      if (thisState[0] - lastState[0] > 1500)
      {
        //1.5s内移动距离小于10cm
        if (abs(thisState[1] - lastState[1]) < 10)
        {
          pause();
          backward(); //后退
          delay(250);
          pause();        //停下来
          findOtherWay(); //重新找路
        }
        //更新状态
        for (int i = 0; i < 2; i++)
        {
          lastState[i] = thisState[i];
        }
      }
    }
    else
    {
      pause();
      findOtherWay();
    }
  }
}

/**
 * 单击按钮启动
 */
void startIt()
{
  isStart = !isStart;
}

/*
   获取当前状态
*/
void getThisState()
{
  thisState[0] = millis();
  thisState[1] = askDistance(90);
}

/*
暂存当前状态
*/
void saveState()
{
  for (int i = 0; i < 2; i++)
  {
    lastState[i] = thisState[i];
  }
}

/*
   左右看，找路
*/
void findOtherWay()
{
  if (askDistance(180) > turningPoint)
  {
    turnLeft(); //若左边有空间，左转
  }
  else if (askDistance(0) > turningPoint)
  {
    turnRight(); //若右边有空间，右转
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
  设置马达
*/
void setMotorPWM(int basicPWM, int modifiedPWM = 0)
{
  analogWrite(MotorRPWM, basicPWM - modifiedPWM / 2);
  analogWrite(MotorLPWM, basicPWM + modifiedPWM / 2);
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
  Serial.println("tracking");
  int infrared[3]; //记录红外传感器的状态
  infrared[0] = digitalRead(sensorLeft);
  infrared[1] = digitalRead(sensorMid);
  infrared[2] = digitalRead(sensorRight);
  if (!infrared[0] && !infrared[1] && !infrared[2])
  {
    findOtherWay();
  }
  else if (infrared[0] && !infrared[1] && !infrared[2])
  {
    turnLeft();
  }
  else if (infrared[1] && !infrared[2])
  {
    forward();
  }
  else
  {
    turnRight();
  }
}
