#include <ros.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Pose2D.h>



ros::NodeHandle  nh;
std_msgs::UInt16 str_msg; 
geometry_msgs::Pose2D traction_msg;

ros::Publisher steering_pub("follower_steering_2", &str_msg);
ros::Publisher traction_pub("coupling_2", &traction_msg);

// RC Command Input
byte PWM_STEER = 5;
byte PWM_TRAC = 3;

void setup() {
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(steering_pub);
  nh.advertise(traction_pub);
}


void loop() {
  // Read RC-Steering
  int str_read = pulseIn(PWM_STEER, HIGH);
  if (str_read > 1500){
    str_msg.data = map(str_read, 1500, 2000, 512, 0);
    }
  else{
    str_msg.data = map(str_read, 1500, 1050, 512, 1024);
    }
  
  // Read RC-Traction
  int trac_read = pulseIn(PWM_TRAC, HIGH);
  traction_msg.theta = trac_read;
  if (trac_read > 1500){
    traction_msg.y = map(trac_read, 1500, 1915, 0, 350);
    traction_msg.x = 0;
    }
  else{
    traction_msg.x = map(trac_read, 1500, 1070, 0, 730); 
    traction_msg.y = 0;
    }
    
  // Publish Topics
  steering_pub.publish(&str_msg);
  traction_pub.publish(&traction_msg);
  //delay(10);
  nh.spinOnce();
}
