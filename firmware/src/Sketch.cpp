/*
 * Created By: Logan Fillo
 * Created On: October 26 2019
 * Description: Main Arduino sketch
 */

#include <Wire.h>
#include <ros.h>
#include <Servo.h>
#include <MS5837.h>
#include <ArduinoHardware.h>
#include "../include/ThrusterArrayDriver.h"
#include "../include/DepthSensorDriver.h"

/* Messages */
drivers_msgs::ArduinoDrivers output_msg_;
drivers_msgs::ArduinoDrivers input_msg_;

/* ROS variables setup */
void driversCB(const drivers_msgs::ArduinoDrivers& msg);
ros::NodeHandle_<ArduinoHardware, 1, 1, 80, 105> nh_; // Custom nodehandle config   
ros::Subscriber<drivers_msgs::ArduinoDrivers> drivers_sub_("/arduino_node/input", &driversCB);
ros::Publisher drivers_pub_("/arduino_node/output", &output_msg_ );

/* Drivers */
arduino_drivers::ThrusterArrayDriver multi_thruster_driver_;
arduino_drivers::DepthSensorDriver depth_sensor_driver_;

/**
 * Callback function for driver subscriber
 * 
 * @param msg The message containg input for the drivers
 */
void driversCB(const drivers_msgs::ArduinoDrivers& msg)
{ 
    input_msg_ = msg;
}

/**
 * Setup method run once on Arduino
 * 
 * Starts I2C bus as master, initalizes ROS node handle and all drivers
 */
void setup() 
{
    Wire.begin();
    
    nh_.initNode();
    nh_.advertise( drivers_pub_ );
    nh_.subscribe( drivers_sub_ );

    multi_thruster_driver_.init();

    // NOTE: this loops forever when depth sensor is not conencted properly!
    // depth_sensor_driver_.init(); 
}

/**
 * Loop method run continously on Arduino
 * 
 * Updates all drivers, publishes output message and spins for callbacks, then sleeps
 */
void loop()
{
    multi_thruster_driver_.update( &output_msg_, &input_msg_ );
    // depth_sensor_driver_.update( &output_msg_, &input_msg_ );

    drivers_pub_.publish ( &output_msg_ );
    nh_.spinOnce();

    delay(200);
}
