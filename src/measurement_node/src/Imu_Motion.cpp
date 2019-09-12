#include "Imu_Motion.h"

#define G_ACCEL_VALUE   9.8105 //[m/s^2 / g]

Imu_Motion::Imu_Motion()
:
    velocity_o(tf2::Vector3(0.0, 0.0, 0.0)),
    acceleration_o(tf2::Vector3(0.0, 0.0, 0.0)),
    position_o(tf2::Vector3(0.0, 0.0, 0.0))
{ }

void Imu_Motion::removeGravity(const tf2::Quaternion &orientation, tf2::Vector3 &accelerationVector)
{
    tf2::Vector3    gravityAccel(0.0, 0.0, G_ACCEL_VALUE);
    gravityAccel = tf2::quatRotate(orientation.inverse(), gravityAccel);
    accelerationVector -= gravityAccel;
}

void Imu_Motion::process(const sensor_msgs::Imu::ConstPtr &imuData)
{
    tf2::Vector3 acceleration;
    tf2::convert(imuData->linear_acceleration, acceleration);
    tf2::Quaternion orientation;
    tf2::convert(imuData->orientation, orientation);

    // Removing gravity
    removeGravity(orientation, acceleration);

    if(acceleration.length() < 0.06)
        acceleration_o = tf2::Vector3(0.0, 0.0, 0.0);
    else
        // Rotating accel vector
        acceleration = tf2::quatRotate(orientation, acceleration);

    ROS_INFO("Acc: %f, %f, %f", acceleration.m_floats[0], acceleration.m_floats[1], acceleration.m_floats[2]);

    // Computing displacement and velocity for every axis
    // New displacement calculation
    position_o += (acceleration + acceleration_o) / 4.0 * dt2 + velocity_o * dt;
    ROS_INFO("Pos: %f, %f, %f", position_o.m_floats[0], position_o.m_floats[1], position_o.m_floats[2]);

    // New speed calculation
    velocity_o += (acceleration + acceleration_o) / 2.0 * dt;
    if(velocity_o.length() < 0.03)
        velocity_o = tf2::Vector3(0.0, 0.0, 0.0);
    ROS_INFO("Vel: %f, %f, %f", velocity_o.m_floats[0], velocity_o.m_floats[1], velocity_o.m_floats[2]);
    printf("\n");

    // New previous acceleration value
    acceleration_o = acceleration;

}

