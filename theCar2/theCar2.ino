#include <Servo.h>
//pin脚定义
const int pinLB = 5; //左退
const int pinLF = 4; //左前
const int pinRB = 3; //右退
const int pinRF = 2; //右前

int inputPin = A1;//超声波Echo引脚
int outputPin = A0; //超声波Trig引脚
int servoPin = 12; //舵机信息号引脚

const int SensorMid = 10; //中感測器輸入腳
const int SensorLeft = 7; //左感測器輸入腳
const int SensorRight = 8; //右感測器輸入腳
const int moshi_S = 9; //模式切换感应（绿色按钮）

int MotorRPWM = 6;
int MotorLPWM = 11;

//变量定义
int SL;//左感測器狀態
int SR;//右感測器狀態
int SM;//中感測器狀態

int directionn = 0;

Servo myservo;//定义舵机

int basicPWM = 100;
int modifiedPWM = -14; //解决左右轮速差问题
int turningTime = 250; //转弯时间

int delay_time = 250;

int Fgo = 1;
int Rgo = 2;
int Lgo = 3;
int Bgo = 4;
int TAr = 5;


int F = 90;
int L1 = 170;
int L2 = 130;
int R1 = 10;
int R2 = 50;
int F_space = 0;
int Stime = 0;//转向用时

int L_space = 0;
int L1_space = 0;
int L2_space = 0;

int R_space = 0;
int R1_space = 0;
int R2_space = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(pinLB, OUTPUT);
  pinMode(pinLF, OUTPUT);
  pinMode(pinRB, OUTPUT);
  pinMode(pinRF, OUTPUT);
  pinMode(inputPin, INPUT);
  pinMode(outputPin, OUTPUT);
  myservo.attach(servoPin);//定义舵机信息号引脚
  pinMode(SensorLeft, INPUT);
  pinMode(SensorRight, INPUT);
  pinMode(SensorMid, INPUT);
  pinMode(moshi_S, INPUT);

  pinMode(MotorLPWM,  OUTPUT); //PWM
  pinMode(MotorRPWM,  OUTPUT);
}

//MotorRPWM，MotorLPWM后面的数字是PWM，可以调整速度控制，数值建议在100-200内，但循迹模式下，速度不能太快。
//调整左轮和右轮的速度，可以尽量解决两个轮转速不同的问题。
//根据实际情况调式。数值越大，速度越快。
void advance(float a) //前进
{
  digitalWrite(pinRB, LOW);
  digitalWrite(pinRF, HIGH);
  analogWrite(MotorRPWM, 108);
  digitalWrite(pinLB, LOW);
  digitalWrite(pinLF, HIGH);
  analogWrite(MotorLPWM, 94);


  //  if (moshi == 1)//如果是循迹模式
  //  {
  //    //------特别说明---------
  //    //步进步停，放慢速度，以适应红外检测。根据场地不同，可以适当调整时间
  //    //---------------
  //    delay(20);
  //    analogWrite(MotorRPWM, 0);
  //    analogWrite(MotorLPWM, 0);
  //    delay(20);
  //  }
  delay(a * 100);
}

void turnR(float d)//双轮逆向转动，转右
{
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, LOW);
  analogWrite(MotorRPWM, 100);
  digitalWrite(pinLB, LOW);
  digitalWrite(pinLF, HIGH);
  analogWrite(MotorLPWM, 100);
  delay(d * 100);
}

void turnL(float e)//双轮逆向转动，转左
{
  digitalWrite(pinRB, LOW);
  digitalWrite(pinRF, HIGH);
  analogWrite(MotorRPWM, 100);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, LOW);
  analogWrite(MotorLPWM, 100);
  delay(e * 100);
}

void turnAround(float e)//双轮逆向转动，转左
{
  digitalWrite(pinRB, LOW);
  digitalWrite(pinRF, HIGH);
  analogWrite(MotorRPWM, 100);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, LOW);
  analogWrite(MotorLPWM, 100);
  delay(e * 200);
}

void stopp(float f)//停止
{
  digitalWrite(pinRB, LOW);
  digitalWrite(pinRF, LOW);
  analogWrite(MotorRPWM, 0);
  digitalWrite(pinLB, LOW);
  digitalWrite(pinLF, LOW);
  analogWrite(MotorLPWM, 0);
  delay(f * 100);
}

void back(float g)//后退
{
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, LOW);
  analogWrite(MotorRPWM, 120);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, LOW);
  analogWrite(MotorLPWM, 100);
  delay(g * 100);
}


/*
   输入：角度
   返回：与该角度前方障碍的距离
*/
int askDistance(int angle)
{
  myservo.write(angle);
  delay(delay_time);
  digitalWrite(outputPin, LOW);   // 超声波发射低电平2μs
  delayMicroseconds(2);
  digitalWrite(outputPin, HIGH);  // 超声波发射高电平10μs
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW);    // 维持超声波发射低电平
  float distance = pulseIn(inputPin, HIGH) / 58.00; //读取相差时间,将时间转化为距离（单位：厘米）
  return (distance);
}

int L_max_space(int jiaodu1, int jiaodu2)//得到左边两个角度最大距离值
{
  L1_space = askDistance(jiaodu1);
  L2_space = askDistance(jiaodu2);
  if (L1_space > L2_space)
  {
    Stime = 4;
    return ( L1_space);
  } else
  {
    Stime = 1;
    return ( L2_space);
  }
}


int R_max_space(int jiaodu1, int jiaodu2)//得到右边两个角度最大距离值
{
  R1_space = askDistance(jiaodu1);
  R2_space = askDistance(jiaodu2);
  if (R1_space > R2_space)
  {
    Stime = 4;
    return ( R1_space);
  } else
  {
    Stime = 1;
    return ( R2_space);
  }
}

void detection()//根据多方距离，得到方向
{
  int delay_time = 250;
  F_space = askDistance(F);
  if (F_space < 10)//前方小于10公分，后退
  {
    stopp(1);
    back(2);
  }
  if (F_space < 25)//前方小于25公分
  {
    stopp(1);
    L_space = L_max_space(L1, L2);//得到左边距离值
    R_space = R_max_space(R1, R2);//得到右边距离值

    if (L_space > R_space)//如果左>右，左转
    {
      directionn = Lgo;
    }
    if (L_space <= R_space)//如果左<=右，右转
    {
      directionn = Rgo;
    }
    if (L_space < 10 && R_space < 10)//均小于10公分，后退
    {
      directionn = Bgo;
    }
  }
  else
  {
    directionn = Fgo;//否则前进
  }
}

//让车沿右边缘走
void R_edge()
{
  int delay_time = 250;
  R_space = askDistance(0);//得到右边距离值
  if (R_space > 25)
  {
    stopp(1);
    directionn = Rgo;
  }
  F_space = askDistance(F);
  if (F_space < 10)//前方小于10公分，后退
  {
    stopp(1);
    back(2);
  }
  if (F_space < 25)//前方小于25公分
  {
    stopp(1);
    L_space = L_max_space(L1, L2);//得到左边距离值
    R_space = R_max_space(R1, R2);//得到右边距离值

    if (L_space > 25)//如果左边可行
    {
      directionn = Lgo;
    }
    if (R_space > 25)//如果右可行，右转
    {
      directionn = Rgo;
    }
    if (L_space < 10 && R_space < 10)//均小于10公分，后退
    {
      directionn = TAr;
    }
  }
  else
  {
    directionn = Fgo;//否则前进
  }
}



//往返跑
int FB = 1;
void FandB()
{
  myservo.write(90);//摆正
  int distance_F = askDistance(90);
  if (distance_F < 25)
  {
    stopp(1);
    FB = 0; //往后跑
  }
  if (distance_F > 100)
  {
    stopp(1);
    FB = 1; //往前跑
  }
}

//------测试模式-------
int mode_test()
{
  FandB();
  if (FB == 1)
  {
    advance(1);
  }
  if (FB == 0)
  {
    back(1);
  }

}


int frontBlock()  //前方是墙
{
  int d = 30; //判断前方是墙的距离
  int distance = askDistance(90);
  if (distance < d)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}


int rightBlock()  //右方是墙
{
  int d = 25; //判断右方是墙的距离
  int distance = askDistance(5);
  if (distance < d)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}


void go(double x)
{
  /*
    digitalWrite(pinRB, LOW);
    digitalWrite(pinRF, HIGH);
    analogWrite(MotorRPWM, 120);
    digitalWrite(pinLB, LOW);
    digitalWrite(pinLF, HIGH);
    analogWrite(MotorLPWM, 100);
    delay(x * 100);
  */
  //  moshi_xj();
  delay(x * 100);
}




//------走迷宫模式----
int mode_maze()
{
  double x = 1; //右转后走的距离
  double t = 5; //左右转的时间
  double s = 1; //停下的时间

  if (frontBlock())
  {
    if (rightBlock())
    {
      go(x);
      turnL(t);
      stopp(s);

    }
    else
    {
      go(x);
      turnR(t);
      go(x);
      stopp(s);
    }
  }
  else
  {
    if (rightBlock())
    {
      go(x);
      stopp(s);
    }
    else
    {
      go(x);
      turnR(t);
      go(x);
      stopp(s);
    }
  }
  myservo.write(90);
}
/*
  void loop()
  {

  mode_maze();
  //myservo.write(90);  //重置舵机


  }
*/

void forward();
void backward();
void turnLeft();
void turnRight();
void turnAround();
void setMotorPWM();
void stop();
int turningPoint = 30;  //该转弯的距离，单位厘米

void loop() {
  myservo.write(90);  //摆正
  if ( askDistance(90) > turningPoint) {
    forward();  //前方有空间，前进
  }
  else {
    stop();
    if ( askDistance(180) > turningPoint) {
      turnLeft(); //若左边有空间，左转
    }
    else if ( askDistance(0) > turningPoint) {
      turnRight();  //若右边有空间，右转
    }
    else {
      turnAround(); //其他情况，掉头
    }
  }
}

/*
   前进
*/
void forward() {
  digitalWrite(pinRB, LOW);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, LOW);
  digitalWrite(pinLF, HIGH);
  setMotorPWM();
}

/*
   后退
*/
void backward() {
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, LOW);
  setMotorPWM();
}

/*
   左转
*/
void turnLeft() {
  digitalWrite(pinRB, LOW);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, LOW);
  setMotorPWM();
  delay(turningTime);
}

/*
   右转
*/
void turnRight() {
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, LOW);
  digitalWrite(pinLF, HIGH);
  setMotorPWM();
  delay(turningTime);
}

/*
   掉头
*/
void turnAround() {
  digitalWrite(pinRB, LOW);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, LOW);
  setMotorPWM();
  delay(turningTime * 2);
}

/*
   设置马达
*/
void setMotorPWM() {
  analogWrite(MotorRPWM, basicPWM - modifiedPWM / 2);
  analogWrite(MotorLPWM, basicPWM + modifiedPWM / 2);
}

void stop() {
  analogWrite(MotorRPWM, 0);
  analogWrite(MotorRPWM, 0);
}
