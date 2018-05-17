//Libraries
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Float64.h>


//Declare servos
Servo servoUp;  
Servo servoSide;                
 
//Variable to store possition of each servo
int maxim=180;
int posUp = maxim;    
int posSide = maxim/2;   

//ROS setup
ros::NodeHandle nh;

//MSG type
std_msgs::Int8MultiArray pos;
std_msgs::Float64 pos_servo;

// Callbacks for servos
void set_servos( const std_msgs::Int8MultiArray& pos){
  if (posUp<maxim && pos.data[1]==1)
    posUp++;
  else if (posUp>0 && pos.data[1]==-1)
    posUp--;
    
  if (posSide<maxim && pos.data[0]==1)
    posSide++;
  else if (posSide>0 && pos.data[0]==-1)
    posSide--;
  
}

//Subscriber
ros::Subscriber<std_msgs::Int8MultiArray> servo_pos("/servo_move", &set_servos);
ros::Publisher Pos_Servo("/servo_pos", &pos_servo);
 
void setup() 
{ 
  //Initialize servos
  servoUp.attach(9);  
  servoSide.attach(10);  
  
  //Subscribe
  nh.initNode();
  nh.advertise(Pos_Servo);
  nh.subscribe(servo_pos);
  
  pos.data_length = 2;
  
  
} 
 
 
void loop() 
{ 
    //Update servos
    servoUp.write(posUp);              
    servoSide.write(posSide);
    pos_servo.data=posSide;
    Pos_Servo.publish(&pos_servo);   
    nh.spinOnce();  
}
