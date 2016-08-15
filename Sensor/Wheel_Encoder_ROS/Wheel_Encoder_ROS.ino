/* Encoder Library - TwoKnobs Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>
// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder knobLeft(9, 8);
Encoder knobRight(12, 11);
/* 
 * rosserial Planar Odometry Example
 */

#include <ros.h>
#include <geometry_msgs/Pose2D.h>



ros::NodeHandle  nh;
geometry_msgs::Pose2D encoder_msg;
geometry_msgs::Pose2D driver_msg;
ros::Publisher encoder_pub("encoder_leader", &encoder_msg);
ros::Publisher driver_pub("driver", &driver_msg);

// Driver Command Input 
const int Steer = A0;
const int Brake = A1;
const int Throttle = A2;

long positionLeft = 0;
long positionRight = 0;

void setup() {
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(encoder_pub);
  nh.advertise(driver_pub);
}


void loop() {
  
  
  positionLeft = - knobLeft.read();
  positionRight = knobRight.read();
  
  encoder_msg.x = positionLeft;
  encoder_msg.y = positionRight;
  
  // Read the Driver Command
  driver_msg.x = analogRead(Throttle);
  driver_msg.y = analogRead(Brake);
  driver_msg.theta = analogRead(Steer);
  

   
  // Publish Topics
  encoder_pub.publish(&encoder_msg);
  driver_pub.publish(&driver_msg);
  nh.spinOnce();
}
