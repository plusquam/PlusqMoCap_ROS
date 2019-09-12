#include "Imu_Motion.h"

#define G_ACCEL_VALUE   9.8105 //[m/s^2 / g]
constexpr double VELOCITY_ATTENUATOR_GAIN_INV() { return 1.0 - VELOCITY_ATTTENUATOR_GAIN_CONTINUOUS(); }

Imu_Motion::Imu_Motion()
:   velocity_o(tf2::Vector3(0.0, 0.0, 0.0)),
    position_o(tf2::Vector3(0.0, 0.0, 0.0)),
    acceleration_o(tf2::Vector3(0.0, 0.0, 0.0))
{ 
    velocity_queue.reserve(VELOCITY_QUEUE_SIZE());
}

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

    // Computing new acceleration vector
    acceleration = tf2::quatRotate(orientation, acceleration);
    ROS_INFO("Acc: %f, %f, %f", acceleration.m_floats[0], acceleration.m_floats[1], acceleration.m_floats[2]);

    // Computing displacement and velocity for every axis
    // New displacement calculation
    position_o += ((acceleration + acceleration_o) / 4.0 * dt2) + (velocity_o * dt);
    ROS_INFO("Pos: %f, %f, %f", position_o.m_floats[0], position_o.m_floats[1], position_o.m_floats[2]);

    // New speed calculation
    velocity_o += (acceleration + acceleration_o) / 2.0 * dt;

    // Storing new acceleration
    acceleration_o = acceleration;

    // Calculating mean value of previous velocity measurements
    tf2::Vector3 meanVelocity(0.0, 0.0, 0.0);
    if(!velocity_queue.empty()) {
        for(std::vector<tf2::Vector3>::iterator it = velocity_queue.begin(); it != velocity_queue.end(); ++it) {
            meanVelocity += *it;
        }
        meanVelocity /= velocity_queue.size();
    }

    for(size_t i = 0; i < 3; ++i) {
        if(std::abs(velocity_o.m_floats[i] - meanVelocity.m_floats[i]) < VELOCITY_ATTENUATOR_TRESH_CONTINUOUS())
            velocity_o.m_floats[i] *= VELOCITY_ATTENUATOR_GAIN_INV();

        if(velocity_o.m_floats[i] < VELOCITY_ATTENUATOR_TRESH_LEVEL())
            velocity_o.m_floats[i] *= VELOCITY_ATTENUATOR_GAIN_LEVEL();
    }

    // Adding new velocity value to the queue
    if(velocity_queue.size() >= VELOCITY_QUEUE_SIZE()) {
        // Removing oldest element
        velocity_queue = std::vector<tf2::Vector3>(velocity_queue.begin() + (velocity_queue.size() - VELOCITY_QUEUE_SIZE()) + 1, velocity_queue.end());
    }
    velocity_queue.push_back(velocity_o);

    ROS_INFO("Vel: %f, %f, %f", velocity_o.m_floats[0], velocity_o.m_floats[1], velocity_o.m_floats[2]);
    printf("\n");
}

