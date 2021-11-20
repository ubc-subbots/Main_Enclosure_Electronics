/*
 * Created By: Logan Fillo
 * Created On: October 25 2019
 * Description: Thrusters Driver interface
 */

#ifndef DRIVERS__THRSTERS_DRIVER_H
#define DRIVERS__THRSTERS_DRIVER_H

#include "Driver.h"
#include <Servo.h>

namespace arduino_drivers
{
    class ThrusterArrayDriver : public Driver
    {
    public:
        ThrusterArrayDriver (/* args */);
        ~ThrusterArrayDriver ();

        void init();
        void update(drivers_msgs::ArduinoDrivers* output_msg, const drivers_msgs::ArduinoDrivers* input_msg);

        // Servo objects
        Servo servo1;
        Servo servo2;
        Servo servo3;
        Servo servo4;
        Servo servo5;
        Servo servo6;

        // Servo pins
        static const char servoPin1 = 9;
        static const char servoPin2 = 8;
        static const char servoPin3 = 7;
        static const char servoPin4 = 6;
        static const char servoPin5 = 5;
        static const char servoPin6 = 4;
        
    };

} // namespace arduino_drivers

#endif //DRIVERS__THRSTERS_DRIVER_H