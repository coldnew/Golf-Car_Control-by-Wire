/*
This function requeested the virtual coupling command(ros topic, including throttle & brake)
and actuate it through digitPot and DC Motor.
*/

// ROS 
#include <ros.h>
// ROS message type
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
// Digital Potentialmeter
#include <SPI.h>

/* Define the pin */
// Feedback Control - Pin Definition
const int Brake = A1;
const int Throttle = A2;

// Init the command input
int driver_throttle = 0;
int driver_brake = 30;
int brake_min = 5;
int brake_max = 350; // Real_Max = 400

// Output
/*
Relay is a intruguing issue.
While it's open, throttle is functional.
But the oscillation make it very annoying. 
*/
int counter_throttle = 0;
int counter_brake = 0;
int check = 5;
const int throttle_relay = 8;


/* Actuator */
// Brake Motor Controller
int EN = 9;
int IN1 = 11;
int IN2 = 10;

// Digital Potentialmeter for Throttle
byte address = 0x00;
int CS= 53;

/* Brake Controller Parameters & Variables */
// Neutral Index
int brake_c = 2000;
// Control Variable
int rev;
int throttle;

/**** BRAKE PI Controller ****/
int error ;
int IntThresh = 40;  // Anti-Windup
int integral = 0;


//Set up the node and publisher
geometry_msgs::Pose2D follower_msg;

ros::NodeHandle nh;
ros::Publisher pub("state_2", &follower_msg);

void coupling_command(const geometry_msgs::Pose2D& cmd_msg)
{
  // Throttle Command
  driver_throttle = (int)cmd_msg.x;
  driver_throttle = constrain(driver_throttle, 0, 730);
  driver_throttle = map(driver_throttle, 0, 730, 0, 92);
  
  // Brake Command
  driver_brake = (int)cmd_msg.y;
  driver_brake = constrain(driver_brake, brake_min, brake_max);
  
  /* I set a counter to avoid relay turn on and off rapidly.*/
  if (driver_throttle > 0){
    counter_throttle += 1;
    counter_brake -= 1;
    }
  else if (driver_brake > 0){
    counter_throttle -= 1;
    counter_brake += 1;
    }

  if (counter_throttle > check){
    // Turn_Throttle_Relay_ON
    digitalWrite(throttle_relay, HIGH);
    counter_throttle = constrain(counter_throttle, 0, 2*check);
    counter_brake = 0;
    }
  else if (counter_brake > check){
    // Turn_Throttle_Relay_Off
    digitalWrite(throttle_relay, LOW);
    counter_throttle = 0;
    counter_brake = constrain(counter_brake, 0, 2*check);
    }
  else{
    // Turn_Throttle_Relay_Off
    digitalWrite(throttle_relay, LOW);
    }
    
  // Publish the vehicle state
  follower_msg.x = analogRead(Throttle);
  follower_msg.y = analogRead(Brake);
  // follower_msg.theta= driver_throttle;
  pub.publish(&follower_msg);
}

ros::Subscriber<geometry_msgs::Pose2D> sub_coupling("coupling_2", coupling_command);

void setup()
{
  // Set the serial baud
  nh.getHardware()->setBaud(115200);
  // Initiate the node
  nh.initNode();
  // Advertise the publisher
  nh.advertise(pub);
  // Advertise the topic needed.
  nh.subscribe(sub_coupling);
  
  /* PIN  Definition */
  // Motor Controler
  pinMode(EN, OUTPUT);
  // Turn On the Brake Motor Controller.
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  // Digital Potentimeter
  pinMode (CS, OUTPUT);
  // Throttle Relay
  pinMode (throttle_relay, OUTPUT);
  
  // Connect the Digital Potentialmeter
  SPI.begin();
}

void loop(){
  // RUN THE ROS NODE
  nh.spinOnce();
  // Control Loop
  digitalWrite(EN, HIGH);
  brake_controller(driver_brake);
  throttle_controller(driver_throttle);
}

/************************ Brake Control ************************/
void brake_controller(int driver_brake){
  // PID Controller Parameter
  float P = 10;
  float I = 0.01;

  int brake_state = analogRead(Brake); 
  error =  brake_state - driver_brake;
    
  if (abs(error) < IntThresh){ // prevent integral 'windup'
    integral = integral + error; // accumulate the error integral
    integral = constrain(integral, -20000, 20000);
    }
  else {
    integral = 0; // zero it if out of bounds
    }
  
  int P_control = P * error;
  P_control = constrain(P_control, -2000, 2000);
  int I_control = I * integral;
  I_control = constrain(I_control, -2000, 2000);

  // PI Controller command
  brake_c = 2000 + P_control + I_control;
  brake_c = constrain(brake_c, 500, 3500);
  
  // Do the Command
  if (abs(error) < 10)
  {
    // In the rolerable range : Brake the DC Motor
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  else
  {
    if (brake_c >= 2000)
    {
      rev = map(brake_c, 2000,3500, 255, 7);
      rev = constrain(rev, 7, 255);
      drive(rev,1); 
    }
    else 
    {
      throttle = map(brake_c, 2000,500, 255, 7);
      throttle = constrain(throttle, 7, 255);
      drive(throttle, 0);
    }
  } 
}

  /************************ Lower Level Control ******************/

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


int throttle_controller(int value)
{
  digitalWrite(CS, LOW);
  SPI.transfer(address);
  SPI.transfer(value);
  digitalWrite(CS, HIGH);
}
