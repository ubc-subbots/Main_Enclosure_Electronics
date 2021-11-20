/*
 * Created By: Logan Fillo
 * Created On: October 25 2019
 * Description: Depth Sensor Driver interface
 */

#ifndef DRIVERS__DEPTH_SENSOR_DRIVER_H
#define DRIVERS__DEPTH_SENSOR_DRIVER_H

#include "Driver.h"
#include <MS5837.h>

namespace arduino_drivers
{
    class DepthSensorDriver : public Driver
    {
    public:
        DepthSensorDriver (/* args */);
        ~DepthSensorDriver ();

        void init();
        void update(drivers_msgs::ArduinoDrivers* output_msg, const drivers_msgs::ArduinoDrivers* input_msg);

        MS5837 sensor;
       
    };
    
} // namespace arduino_drivers

#endif //DRIVERS__DEPTH_SENSOR_DRIVER_H