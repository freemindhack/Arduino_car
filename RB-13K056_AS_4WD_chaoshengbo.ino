#include <Servo.h> 
Servo myservo;
int duration; //定义变量duration用来存储脉冲 
int distance; //定义变量distance用来存储距离值 
int srfPin = 15; //定义srfPin为数字口15 
int z; //定义变量Z int val; 
int val;//定义变量val 
int val1; //定义变量val1 
int val2; //定义变量val2 
#define EA 5
#define I2 6
#define I1 7
#define EB 11
#define I4 10
#define I3 9 
void setup()
{
myservo.attach(8); //8号引脚输出舵机控制信号 
myservo.write(90); //使舵机转到90度
Serial.begin(9600);//打开串口，设置波特率为9600 
pinMode(I1,OUTPUT);//定义I1 接口
pinMode(I2,OUTPUT);//定义I2 接口
pinMode(EA,OUTPUT);//定义EA(PWM 调速)接口
pinMode(EB,OUTPUT);//定义EB(PWM 调速)接口
pinMode(I4,OUTPUT);//定义I4 接口
pinMode(I3,OUTPUT);//定义I3 接口
}
void loop()
{ 
  delay(200);
  qianjin(); //调用前进子程序 
  val=Ultrasonic(z); //将超声波读取的距离值赋值给val 
   if(val<35) //判断如果val小于25则继续执行 
{ 
  tingzhi(); //调用停止子程序 
  myservo.write(0); //让舵机转0度 
  delay(1000); //延时1秒等待舵机到达指定位置
  val1=Ultrasonic(z); //将超声波读取的距离值赋值给
  delay(1000); //延时1秒等待舵机到达指定位置 
  myservo.write(179); //让舵机转180度 
  delay(1000); //延时1秒等待舵机到达指定位置 
  val2=Ultrasonic(z); //将超声波读取的距离值赋值给val2 
  delay(1000); //延时1秒等待舵机到达指定位置 
  myservo.write(90); //让舵机转90度 
  delay(1000); //延时1秒 
  if(val1<val2) 
{ 
  zuozhuan(); //调用左转子程序 
  delay(100); //延时200毫秒
 } 
  else 
{ 
  youzhuan(); //调用右转子程序 
  delay(100); //延时200毫秒 
} 
  delay(500); //延时500毫秒
  }
}
void qianjin()//前进
{
analogWrite(EA,150);//输入模拟值进行设定速度
analogWrite(EB,150);
digitalWrite(I2,HIGH);//使直流电机运转
digitalWrite(I1,LOW);
digitalWrite(I3,HIGH);//使直流电机运转
digitalWrite(I4,LOW); 
}

void houtui()//后退
{
analogWrite(EA,120);//输入模拟值进行设定速度
analogWrite(EB,120);
digitalWrite(I2,LOW);//使直流电机运转
digitalWrite(I1,HIGH);
digitalWrite(I3,LOW);//使直流电机运转
digitalWrite(I4,HIGH);
}

void youzhuan()//右转
{
analogWrite(EA,120);//输入模拟值进行设定速度
analogWrite(EB,120);
digitalWrite(I2,LOW);//使直流电机运转
digitalWrite(I1,HIGH);
digitalWrite(I3,HIGH);//使直流电机运转
digitalWrite(I4,LOW); 
}

void zuozhuan()//左转
{
analogWrite(EA,120);//输入模拟值进行设定速度
analogWrite(EB,120);
digitalWrite(I2,HIGH);//使直流电机运转
digitalWrite(I1,LOW);
digitalWrite(I3,LOW);//使直流电机运转
digitalWrite(I4,HIGH);
}

void tingzhi()//停止
{
digitalWrite(I2,LOW);//使直流电机停转
digitalWrite(I1,LOW);
digitalWrite(I3,LOW);//使直流电机停转
digitalWrite(I4,LOW);
}

int Ultrasonic(int distance) 
{ 
  pinMode(srfPin,OUTPUT); //定义srfPin为输出接口 
  digitalWrite(srfPin, HIGH); //高电平触发前发送2微秒的低电平 
  delayMicroseconds(2); 
  digitalWrite(srfPin, LOW); //发送10微秒的高电平开始检测 
  delayMicroseconds(10); 
  digitalWrite(srfPin, HIGH); //等待脉冲返回前发送一个低电平 
  pinMode(srfPin, INPUT); //定义srfPin为输入接口 
  duration = pulseIn(srfPin, LOW); //从URF02读取脉冲 
  distance = duration/58; //除以58得到距离值 
  return distance; 
}


