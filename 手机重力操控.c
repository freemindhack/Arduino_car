#include<Servo.h>
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

}
void loop()
{

}
void forward()
{
	analogWrite(EA,150);
	analogWrite(EB,150);
	digitalWrite(I2,HIGH);//使直流电机正转
	digitalWrite(I1,LOW);
	digitalWrite(I3,HIGH);//使直流电机正转
	digitalWrite(I4,LOW); 
	return ;
}
void backward()
{
	analogWrite(EA,120);//输入模拟值进行设定速度
	analogWrite(EB,120);
	digitalWrite(I2,LOW);//使直流电机倒转
	digitalWrite(I1,HIGH);
	digitalWrite(I3,LOW);//使直流电机倒转
	digitalWrite(I4,HIGH);
	return ;
}
void right()
{
	analogWrite(EA,255); //把占空比调高一点儿就能转动了
	analogWrite(EB,255);
	digitalWrite(I2,HIGH);//使直流电机正转
	digitalWrite(I1,LOW);
	digitalWrite(I3,LOW);//使直流电机正转
	digitalWrite(I4,HIGH); 
	return ;
}
void left()
{
	analogWrite(EA,255);
	analogWrite(EB,255);
	digitalWrite(I2,LOW);//使直流电机正转
	digitalWrite(I1,HIGH);
	digitalWrite(I3,HIGH);//使直流电机正转
	digitalWrite(I4,LOW); 
	return ;
}
void stop()
{
	digitalWrite(I2,LOW);
	digitalWrite(I1,LOW);
	digitalWrite(I3,LOW);
	digitalWrite(I4,LOW); 
	return ;
}

