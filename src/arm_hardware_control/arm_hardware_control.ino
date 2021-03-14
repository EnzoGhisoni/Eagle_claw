//rosrun rosserial_python serial_node.py /dev/ttyACM0
// On Ubuntu, if the port acces is denied: sudo chmod a+rw /dev/ttyACM0
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Float64.h>

#define rad2deg (180/3.14159265)

ros::NodeHandle  nh;

Servo joint1;
Servo joint2;
Servo joint3;
Servo joint4;
Servo joint5;


void joint1_cb( const std_msgs::Float64& msg){
  int joint_angle = int(rad2deg * msg.data);
  joint1.write(joint_angle); //set servo angle, should be from 0-180
}
void joint2_cb( const std_msgs::Float64& msg){
  int joint_angle = int(rad2deg * msg.data);
  joint2.write(int(rad2deg * msg.data)); //set servo angle, should be from 0-180
}

void joint3_cb( const std_msgs::Float64& msg){
  int joint_angle = int(rad2deg * msg.data);
  joint3.write(joint_angle); //set servo angle, should be from 0-180
}
void joint4_cb( const std_msgs::Float64& msg){
  int joint_angle = int(rad2deg * msg.data);
  joint4.write(int(rad2deg * msg.data)); //set servo angle, should be from 0-180
}

void joint5_cb( const std_msgs::Float64& msg){
  int joint_angle = int(rad2deg * msg.data);
  joint5.write(joint_angle); //set servo angle, should be from 0-180
}


ros::Subscriber<std_msgs::Float64> joint1_sub("/mrm/joint1_position_controller/command", joint1_cb);
ros::Subscriber<std_msgs::Float64> joint2_sub("/mrm/joint2_position_controller/command", joint2_cb);
ros::Subscriber<std_msgs::Float64> joint3_sub("/mrm/joint3_position_controller/command", joint3_cb);
ros::Subscriber<std_msgs::Float64> joint4_sub("/mrm/joint4_position_controller/command", joint4_cb);
ros::Subscriber<std_msgs::Float64> joint5_sub("/mrm/joint5_position_controller/command", joint5_cb);


void setup(){
  nh.initNode();
  nh.subscribe(joint1_sub);
  nh.subscribe(joint2_sub);
  nh.subscribe(joint3_sub);
  nh.subscribe(joint4_sub);
  nh.subscribe(joint5_sub);
  joint1.attach(9); //attach it to pin 9
  joint2.attach(10); //attach it to pin 10
  joint3.attach(11); //attach it to pin 9
  joint4.attach(12); //attach it to pin 10
  joint5.attach(13); //attach it to pin 9
}

void loop(){
  nh.spinOnce();
  delay(1);
}
