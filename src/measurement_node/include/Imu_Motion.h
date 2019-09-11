#pragma once
#include <iostream>
#include "ros/ros.h"
#include <ros/console.h>
#include "sensor_msgs/Imu.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

#define SAMPLE_INTERVAL (0.010)

class Imu_Motion
{
private:
    const float     dt = SAMPLE_INTERVAL; // [s]
    const float     dt2 = SAMPLE_INTERVAL * SAMPLE_INTERVAL; // [s]
    tf2::Vector3    velocity_o; // [m/s]
    tf2::Vector3    acceleration_o; // [m/s^2]
    tf2::Vector3    position_o;

public:
    Imu_Motion();
    void process(const sensor_msgs::Imu::ConstPtr &imuData);
    inline tf2::Vector3 getPosition()
    {
        return position_o;
    }

    static inline void removeGravity(const tf2::Quaternion &orientation, tf2::Vector3 &accelerationVector);
};