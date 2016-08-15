/*
This Function requests the /follower_steering_2 topic and actuator the Steering Motor.
And it publish its state(/steering_state_2)
*/
#include <ros.h>
// for message type
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>

// Define the pin
// Motor Controller
int EN = 9;
int IN1 = 11;
int IN2 = 10;

// PI controller - command index
unsigned long str_read;

// Control Variable
int rev;
int throttle;

// Read the angle
const int steering_Pin = A0;
int command = 485;
int sensorValue;
/* Calibration the neutral point.*/
int neutral_cal = 45;
int max_l = 924;
int max_r = 100;

// PI
int error = 0;
int IntThresh = 50;  // Anti-Windup
int integral = 0;


float Kp = 20.0;
float Ki = 0.01;


//Set up the node and publisher
std_msgs::UInt16 str_msg; 

ros::NodeHandle nh;
ros::Publisher pub("steering_state_2", &str_msg);

// Connection Test
void read_command( const std_msgs::UInt16& cmd_msg){
  
  command = cmd_msg.data - neutral_cal;
  command = constrain(command, max_r, max_l);
  str_msg.data = sensorValue + neutral_cal ;
  pub.publish(&str_msg);
}

ros::Subscriber<std_msgs::UInt16> sub("follower_steering_2", read_command);

void setup()
{
  nh.getHardware()->setBaud(115200);
  // Initiate the node
  nh.initNode();
  // Advertise the publisher
  nh.advertise(pub);
  // Subscribe the topic needed.
  nh.subscribe(sub);
  
  //PIN  Definition
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
}

void loop(){  
    
  sensorValue = analogRead(steering_Pin);
  error =  sensorValue - command;
  
  if (abs(error) < IntThresh){ // prevent integral 'windup'
     // accumulate the error integral
    integral = integral + error; // accumulate the error integral
    integral = constrain(integral, -20000, 20000);
  }
  else {
    integral = 0; // zero it if out of bounds
  }
  
  
  int P = (int)(Kp * error);
  P = constrain(P, -2000, 2000);
  int I = (int)(Ki * integral);
  I = constrain(I, -2000, 2000);
  
  str_read = 2000 + P + I;
  str_read = constrain(str_read, 500, 3500);  
  
  // Actuate the Command
  if (abs(error) < 10){
    // Reach Tolerable range: Brake
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  else
  {
    if (str_read >= 2000)
    {
      rev = map(str_read, 2000, 3500, 255, 30);
      rev = constrain(rev, 30, 255);
      drive(rev,1);
    }
    else 
    {
      throttle = map(str_read, 2000,500, 255, 30);
      throttle = constrain(throttle, 30, 255);
      drive(throttle, 0);
    }
  }
  nh.spinOnce();
}


void drive( int pwm, int reverse){
  //pwm - the speed (0 max, 255 stop)
  //dir - direction of motor (1 forward, 0 reverse)
  
  if (reverse == 0){  //reverse = 1 for reverse, reverse = 0 for forward
    analogWrite(IN2, pwm);
    analogWrite(IN1, 255);
  } 
  
  if (reverse == 1) {
    analogWrite(IN1, pwm);
    analogWrite(IN2, 255);
  }
}

