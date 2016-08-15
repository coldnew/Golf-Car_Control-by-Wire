#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gyro;         // [x, y, z]            angular accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Unit Conversion
float ACCEL_RANGE = 19.62; // 2 * 9.81 m/s^2
float GYRO_RANGE = 4.3633; //250 (deg/sec) / 180 * pi => rad/sec
float SCALE = 32768;

// packet structure for InvenSense teapot demo
//uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

#include <Wire.h>
#include <ros.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Quaternion.h>

ros::NodeHandle  nh;

geometry_msgs::Accel accel_msg;
geometry_msgs::Quaternion orientation_msg;

ros::Publisher pub_accel("accel", &accel_msg);
ros::Publisher pub_orientation("orientation", &orientation_msg);



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)   
    Wire.begin();
    // configure LED for output
    // initialize serial communication
    //Serial.begin(115200);
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(pub_accel);
    nh.advertise(pub_orientation);
    // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    bool test =  mpu.testConnection();
      
    
    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(-3418);
    mpu.setYAccelOffset(-5146);
    mpu.setZAccelOffset(1543); 
    mpu.setXGyroOffset(83);
    mpu.setYGyroOffset(-28);
    mpu.setZGyroOffset(-1);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
    
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        
        
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
    
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGyro(&gyro, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);        
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        
        orientation_msg.x = q.x;
        orientation_msg.y = q.y;
        orientation_msg.z = q.z;
        orientation_msg.w = q.w;
        accel_msg.linear.x = float(aaReal.x) * 0.0006;
        accel_msg.linear.y = float(aaReal.y) * 0.0006;
        accel_msg.linear.z = float(aaReal.z) * 0.0006;
        accel_msg.angular.x = float(gyro.x) * 0.00013;
        accel_msg.angular.y = float(gyro.y) * 0.00013;
        accel_msg.angular.z = float(gyro.z) * 0.00013;
        pub_orientation.publish(&orientation_msg);
        pub_accel.publish(&accel_msg);
        

    }
    nh.spinOnce();
    //delay(5);
}
