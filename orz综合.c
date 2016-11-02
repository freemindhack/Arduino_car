#include<Servo.h>
Servo myservo;
int duration;
float dis;
float left_dis;
float right_dis;
int velocity=1; //单位是cm/s
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
	Serial.println("What do you want");
	Serial.println("w --move forward");
	Serial.println("s --move backward");
	Serial.println("a --Turn left");
	Serial.println("d --Turn right");
	Serial.println("m --dingwei");
	Serial.println("n --bizhang");
}
void loop()
{
	while(Serial.available())
   {
    char c=Serial.read();
     	switch(c)
     	{
     		case'w' : Serial.println("forward\n"); forward();delay(1000);stop();break;
     		case's' : Serial.println("backward\n"); backward();delay(1000);stop();break;
     		case'a' : Serial.println("left\n"); left();delay(1000);stop();break;
     		case'd' : Serial.println("right\n"); right();delay(1000);stop();break;
     		case'm' : Serial.println("dingwei_easy\n");dingwei_easy();break;
     		case'n' : Serial.println("bizhang\n");bizhang();break; 
     		default : stop();break;
     	}
    Serial.flush();
   }
}
void dingwei_easy()
{
	Serial.write("please input X:\n");
	while(!Serial.available());
	while(Serial.available()>0){
		x=x*10+Serial.read()-'0';
		Serial.print("x=");
		Serial.println(x);
	}
	x=x*1000;
	Serial.flush();
	Serial.write("please input Y:\n");
	while(!Serial.available());
	while(Serial.available()>0){
		y=y*10+Serial.read()-'0';
		Serial.print("y=");
		Serial.println(y);
	}
	y=y*1000;

	forward();
	delay(y);
	stop();

	right();
	delay(1000);
	
	forward();
	delay(x);

	stop();

}
// void read_xy()
// {
// 	Serial.write("please input X:\n");
// 	while(!Serial.available());
// 	while(Serial.available()>0){
// 		x=x*10+Serial.read()-'0';
// 		Serial.print("x=");
// 		Serial.println(x);
// 	}
// 	x=x*1000;
// 	Serial.flush();
// 	Serial.write("please input Y:\n");
// 	while(!Serial.available());
// 	while(Serial.available()>0){
// 		y=y*10+Serial.read()-'0';
// 		Serial.print("y=");
// 		Serial.println(y);
// 	}
// 	y=y*1000;
// }
void dingwei()
{	
	//read_xy();
	current_x=0;
	current_y=0;
	while(current_x!=x || current_y!=y)
	{
		current_y=runy();
		if( current_y==y ){  //返回值等于y说明 y方向已经到达了指定位置
			Serial.println("y arrived");
			myservo.write(0); //看看右边有没有障碍
			if( (right_dis=Ultrasonic() )<30 )  //如果y方向已经到了 但是x方向有障碍，就往后退一段
				{
					backward();
					delay(1000);
				}
			stop();
		}

		right();
		delay(500); //右转
		stop();
		current_x=runx();
		if( current_x==x ){  //返回值等于x说明 x方向已经到达了指定位置
			Serial.println("x arrived");
			if( (left_dis=Ultrasonic() )<30 )  //如果x方向已经到了 但是y方向有障碍，就往后退一段
				{
					backward();
				}
			stop();
		}
		left();
		delay(500);
		stop();
	}
	right();  //到地方之后原地转圈
	delay(10000);
	
}
int runy()
{
	if(current_y>=y)
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
	if(current_x>=x)
		return 1;
	forward();
	starttime = millis();
	while((dis=Ultrasonic())>40 && (current_x + velocity * ( millis()-starttime )< x) ) {};  //当前方障碍距离小于40且未到达指定y值的时候继续前进
	stoptime = millis();
	current_x=current_x+(stoptime-starttime)*velocity;
	return current_x;    //停下来就返回当前的x;
}


void bizhang()
{
		while(1){
			forward();
			while((dis=(Ultrasonic()+Ultrasonic()+Ultrasonic())/3)>30){Serial.println("go");};  //前方距离超过40cm的时候一直往前走
			if( (dis=Ultrasonic())<40 )
			{
		        Serial.print("front_distance=");
				Serial.print(dis);
				Serial.println("cm"); 
				backward();
				delay(200);  //检测到前方有障碍物的时候 先向后退一点
				stop();
				myservo.write(179);
				delay(1000);
				left_dis=Ultrasonic();
				Serial.print("left_distance=");
				Serial.print(left_dis);
				Serial.println("cm"); 
				myservo.write(1);
				delay(1000);
				right_dis=Ultrasonic();
				Serial.print("right_distance=");
				Serial.print(right_dis);
				myservo.write(90);
				delay(1000);
				if(30<left_dis<right_dis)
					{
						Serial.println("Turn right");
						right();
						delay(1000); //90度
						forward(); //保证转弯之后再往前走一点，防止原地打转
						delay(500);
						stop();
					}
				else if(30<right_dis<left_dis)
					{
						Serial.println("Turn left");
						left();
						delay(1000);
						forward(); //保证转弯之后再往前走一点，防止原地打转
						delay(500);
						stop();
					}
				else{
					backward();
					delay(1000);
					stop();
				}					
			}
			else;
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
	return ;
}
void backward()
{
	analogWrite(EA,150);//输入模拟值进行设定速度
	analogWrite(EB,150);
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