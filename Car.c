#include<Servo.h>
Servo myservo;
int duration;

float dis;
float left_dis;
float right_dis;
#define TrigPin 2 //接超声波模块Trig
#define EchoPin 3  //接超声波模块Echo
#define EA 5
#define I2 6 
#define I1 7
#define EB 11
#define I4 10
#define I3 9 

void setup()
{
	myservo.attach(8); //使8号引脚输出舵机控制信号
	myservo.write(90);//使舵机面朝着很前方
	Serial.begin(9600);
	pinMode(I1,OUTPUT);//定义I1 接口
	pinMode(I2,OUTPUT);//定义I2 接口
	pinMode(EA,OUTPUT);//定义EA(PWM 调速)接口
	pinMode(EB,OUTPUT);//定义EB(PWM 调速)接口
	pinMode(I4,OUTPUT);//定义I4 接口
	pinMode(I3,OUTPUT);//定义I3 接口
	pinMode(TrigPin,OUTPUT);//超声波信号发出接口
	pinMode(EchoPin,INPUT); //超声波接受接口
}
void loop()
{
	
	forward();//调用前进子程序
	delay(1000);
	dis=Ultrasonic();
	Serial.print("Distance="); 
	Serial.print(dis); 
	Serial.println("cm"); 
	if(dis<35)
	{
		stop();
		Serial.println("stop"); 
		myservo.write(0);
		delay(1000);
		left_dis=Ultrasonic();
		Serial.print("Left_distance=");
		Serial.print(left_dis); 
		Serial.println("cm"); 
		myservo.write(179);
		delay(1000);
		right_dis=Ultrasonic();
		Serial.print("right_distance=");
		Serial.print(right_dis); 
		Serial.println("cm"); 
		myservo.write(90);
		delay(1000);
		if(left_dis<right_dis)
			{
				Serial.println("Turn right");
				right();
			}
		else
			{
				Serial.println("Turn left");
				left();
			}					
	}
}
void forward()
{
	analogWrite(EA,150);
	analogWrite(EB,150);
	digitalWrite(I2,HIGH);//使直流电机正转
	digitalWrite(I1,LOW);
	digitalWrite(I3,HIGH);//使直流电机正转
	digitalWrite(I4,LOW); 
}
void backward()
{
	analogWrite(EA,120);//输入模拟值进行设定速度
	analogWrite(EB,120);
	digitalWrite(I2,LOW);//使直流电机倒转
	digitalWrite(I1,HIGH);
	digitalWrite(I3,LOW);//使直流电机倒转
	digitalWrite(I4,HIGH);
}
void right()
{
	analogWrite(EA,255); //把占空比调高一点儿就能转动了
	analogWrite(EB,255);
	digitalWrite(I2,HIGH);//使直流电机正转
	digitalWrite(I1,LOW);
	digitalWrite(I3,LOW);//使直流电机正转
	digitalWrite(I4,HIGH); 
	delay(2500);
}
void left()
{
	analogWrite(EA,255);
	analogWrite(EB,255);
	digitalWrite(I2,LOW);//使直流电机正转
	digitalWrite(I1,HIGH);
	digitalWrite(I3,HIGH);//使直流电机正转
	digitalWrite(I4,LOW); 
	delay(2500);
}
void stop()
{
	digitalWrite(I2,LOW);
	digitalWrite(I1,LOW);
	digitalWrite(I3,LOW);
	digitalWrite(I4,LOW); 
}

int Ultrasonic()
{
	float distance;
	digitalWrite(TrigPin,LOW);//高电平出发前发送2ms低电平
	delayMicroseconds(2);
	digitalWrite(TrigPin,HIGH);//发送10微秒的高电平开始检测 
	delayMicroseconds(10);
	digitalWrite(TrigPin,LOW);//等待脉冲返回前发送一个低电平
	duration=pulseIn(EchoPin,HIGH); //读出脉冲时间
	distance=duration*0.017; //脉冲时间转化为距离值
	return distance;
}