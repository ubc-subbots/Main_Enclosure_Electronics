/*
 * Created By: Logan Fillo
 * Created On: October 25 2019
 * Description: Thrusters driver
 */

#include "../include/ThrusterArrayDriver.h"

namespace arduino_drivers
{
    ThrusterArrayDriver::ThrusterArrayDriver() {}
    ThrusterArrayDriver::~ThrusterArrayDriver() {}

    void ThrusterArrayDriver::init() 
    {
        // Attach servo objects to respective pins
        servo1.attach(servoPin1);
        servo2.attach(servoPin2);
        servo3.attach(servoPin3);
        servo4.attach(servoPin4);
        servo5.attach(servoPin5);
        servo6.attach(servoPin6);


        // Send "stop" signal to all ESCs.
        servo1.writeMicroseconds(1500+25, 1); 
        servo2.writeMicroseconds(1500+25, 1); 
        servo3.writeMicroseconds(1500+25, 1); 
        servo4.writeMicroseconds(1500+25, 1); 
        servo5.writeMicroseconds(1500+25, 1); 
        servo6.writeMicroseconds(1500+25, 1); 

        // Delay to allow the ESCs to recognize the stopped signal
        delay(7000); 

    }
    void ThrusterArrayDriver::update(drivers_msgs::ArduinoDrivers* output_msg, const drivers_msgs::ArduinoDrivers* input_msg) 
    {
        int fudge_factor = 25;
        bool invert = 1;

        // Send signal to ESC.
        servo1.writeMicroseconds(input_msg->thruster_array.thruster_one_command + fudge_factor, invert); 
        servo2.writeMicroseconds(input_msg->thruster_array.thruster_two_command + fudge_factor, invert); 
        servo3.writeMicroseconds(input_msg->thruster_array.thruster_three_command + fudge_factor, invert); 
        servo4.writeMicroseconds(input_msg->thruster_array.thruster_four_command + fudge_factor, invert); 
        servo5.writeMicroseconds(input_msg->thruster_array.thruster_five_command + fudge_factor, invert); 
        servo6.writeMicroseconds(input_msg->thruster_array.thruster_six_command + fudge_factor, invert); 

    }
    
} // namespace arduino_drivers