#pragma once
#include <iostream>
#include <queue>

#include "ros/ros.h"
#include <ros/console.h>
#include "sensor_msgs/Imu.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

constexpr double SAMPLE_INTERVAL() { return 0.005; }; // [s]
constexpr double SAMPLE_INTERVAL_2() { return SAMPLE_INTERVAL() * SAMPLE_INTERVAL(); }; // [s]
constexpr size_t QUEUE_SIZE() { return 5; };
constexpr size_t VELOCITY_QUEUE_SIZE() { return QUEUE_SIZE() * 5; };
constexpr double VELOCITY_ATTTENUATOR_GAIN_CONTINUOUS() { return 0.02; };  // [m/s]
constexpr double VELOCITY_ATTENUATOR_TRESH_CONTINUOUS() { return 0.015; }; // [m/s]
constexpr double VELOCITY_ATTENUATOR_TRESH_LEVEL() { return 0.015; }; // [m/s]
constexpr double VELOCITY_ATTENUATOR_GAIN_LEVEL() { return 0.005; }; // [m/s]

class Imu_Motion
{
private:
    const double                dt = SAMPLE_INTERVAL(); // [s]
    const double                dt2 = SAMPLE_INTERVAL_2(); // [s]
    tf2::Vector3                velocity_o;     // [m/s]
    std::vector<tf2::Vector3>   velocity_queue;  
    tf2::Vector3                acceleration_o; // [m/s^2]
    tf2::Vector3                position_o;     // [m]

public:
    Imu_Motion(tf2::Vector3 initialPosition = tf2::Vector3(0.0, 0.0, 0.0));
    void process(const sensor_msgs::Imu::ConstPtr &imuData);
    inline tf2::Vector3 getPosition()
    {
        return position_o;
    }

    static inline void removeGravity(const tf2::Quaternion &orientation, tf2::Vector3 &accelerationVector);
};