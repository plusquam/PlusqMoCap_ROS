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
    tf2::quatRotate(orientation, gravityAccel);
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

    if(acceleration.length() > 0.15)
    {
        // Rotating accel vector
        tf2::quatRotate(orientation, acceleration);

        // Computing displacement and velocity for every exis
        // New displacement calculation
        position_o += (acceleration + acceleration_o) / 4.0 * dt2 + velocity_o;

        // New speed calculation
        velocity_o += (acceleration + acceleration_o) / 2.0 * dt;

        // New previous acceleration value
        acceleration_o = acceleration;
    }
}

