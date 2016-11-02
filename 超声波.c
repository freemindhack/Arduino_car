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
	myservo.write(90);
}
void loop()
{
		myservo.write(90);
		delay(1000);
		while((dis=(Ultrasonic()+Ultrasonic()+Ultrasonic())/3)>30){Serial.println("go");};  //前方距离超过40cm的时候一直往前走
		if( (dis=Ultrasonic())<35 )
		{
	        Serial.print("front_distance=");
			Serial.print(dis);
			Serial.println("cm"); 
		}
		// myservo.write(179);
		// delay(1000);
		// left_dis=Ultrasonic();
		// Serial.print("left_distance=");
		// Serial.print(left_dis);
		// Serial.println("cm"); 

		// myservo.write(0);
		// delay(1000);
		// right_dis=Ultrasonic();
		// Serial.print("right_dis=");
		// Serial.print(right_dis);
		// Serial.println("cm"); 
	
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