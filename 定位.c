//每次转弯的时候都旋转90度
#include<Servo.h>
Servo myservo;
int duration;
float dis;
float left_dis;
float right_dis;
int velocity=20;  //单位是 cm/s
int current_x=0;
int current_y=0;
int x;
int y;
int starttime=0;
int stoptime=0;
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
	// Serial.write("please input (X,Y):\n");
	// int mark=0;
	// while(!Serial.available());
	// while(Serial.available()>0){
	// 	char a=Serial.read();
	// 	if(isDigit(a)){
	// 		if(mark==1)
	// 			y=y*10+a-'0';
	// 		else
	// 			x=x*10+a-'0';
	// 	}
	// 	else{
	// 		if(mark==0){
	// 			Serial.print("x=");
	// 			Serial.println(x);
	// 			mark=mark+1;
	// 		}
	// 		else 
	// 		{
	// 			Serial.print("y=");
	// 			Serial.println(y);
	// 		}
	// 	}
	// }
	
	while(current_x!=x || current_y!=y)
	{
		current_y=runy();
		if( y==current_y ){  //返回值等于y说明 y方向已经到达了指定位置
			Serial.println("y arrived");
			myservo.write(0); //看看右边有没有障碍
			if( (right_dis=Ultrasonic() )<30 )  //如果y方向已经到了 但是x方向有障碍，就往后退一段
				{
					backward();
					delay(1000);
					current_y=current_y- 1000* velocity;
				}
			stop();
		}

		right();
		delay(1000); //右转
		stop();
		current_x=runx();
		if( x==current_x ){  //返回值等于x说明 x方向已经到达了指定位置
			Serial.println("x arrived");
			if( (left_dis=Ultrasonic() )<30 )  //如果x方向已经到了 但是y方向有障碍，就往后退一段
				{
					backward();
					delay(1000);
					current_x=current_x-1000* velocity;
				}
			stop();
		}
		left();
		delay(1000);
		stop();
	}
	right();  //到地方之后原地转圈
	delay(10000);
	
}
int runy()
{
	if(current_y==y)
		return y;
	forward();
	starttime = millis();
	while((dis=Ultrasonic())>40 && (current_y + velocity * ( millis()-starttime )< y) ) {};  //当前方障碍距离小于40且未到达指定y值的时候继续前进
	stoptime = millis();
	current_y=current_y+(stoptime-starttime)*velocity;
	return current_y;    //停下来就返回当前的y
}
int runx()
{
	if(current_x==x)
		return x;
	forward();
	starttime = millis();
	while((dis=Ultrasonic())>40 && (current_x + velocity * ( millis()-starttime )< x) ) {};  //当前方障碍距离小于40且未到达指定y值的时候继续前进
	stoptime = millis();
	current_x=current_x+(stoptime-starttime)*velocity;
	return current_x;    //停下来就返回当前的x;
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