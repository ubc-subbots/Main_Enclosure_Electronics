/*
 * Created By: Logan Fillo
 * Created On: October 25 2019
 * Description: Abstract Driver interface
 */

#ifndef DRIVERS__DRIVER_H
#define DRIVERS__DRIVER_H

#include <drivers_msgs/ArduinoDrivers.h>
#include <Arduino.h>

namespace arduino_drivers
{
    class Driver
    {
    public:
    
        /**
         * Initializes the driver
         * 
         * Sets up any objects needed for the driver to
         * communcate with the associated hardware
         * 
         * To be implemented by subclasses
         */
        virtual void init() = 0;

        /**
         * Updates the driver during an iteration of the Arduino loop
         * 
         * As applciable, reads from the hardware associated with this driver 
         * and places the data inside the output message, then writes the 
         * associated section of the input message to the hardware.
         * 
         * To be implemented by subclasses
         * 
         * @param output_msg The message to be published as output
         * @param input_msgs_ The message recieved as input
         */
        virtual void update(drivers_msgs::ArduinoDrivers* output_msg, const drivers_msgs::ArduinoDrivers* input_msg) = 0;

    };
    
} // namespace arduino_drivers

#endif //DRIVERS__DRIVER_H