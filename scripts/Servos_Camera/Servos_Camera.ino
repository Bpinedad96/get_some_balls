//Libraries
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Float32MultiArray.h>


//Declare servos
Servo servoUp;  
Servo servoSide;                
 
//Variable to store possition of each servo
int maxim=180;
int posUp = 0;    
int posSide = maxim/2; 
boolean turn=false;

//ROS setup
ros::NodeHandle nh;

//MSG type
std_msgs::Int8MultiArray pos;
std_msgs::Float32MultiArray ball;

// Callbacks for servos
void set_servos( const std_msgs::Int8MultiArray& pos){
  if (pos.data[0]==2) {
      if (posUp>0)
        posUp--;
      if (posSide<maxim && turn==false) {
        posSide++;
        if (posSide==maxim)
          turn=true;
      }
      else if (posSide>0) {
        posSide--;
        if (posSide==0)
          turn=false;
      }
      return;
  }
  
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
ros::Publisher pos_ball("/ball_pos", &ball);
 
void setup() 
{ 
  //Initialize servos
  servoUp.attach(9);  
  servoSide.attach(10);  
  
  //Subscribe
  nh.initNode();
  nh.advertise(pos_ball);
  nh.subscribe(servo_pos);
  
  pos.data_length = 2;
  ball.data_length = 2;
} 
 
 
void loop() 
{ 
    //Update servos
    servoUp.write(posUp);              
    servoSide.write(posSide);
    ball.data[1]=posSide;
    pos_ball.publish(&ball);   
    nh.spinOnce();  
}