#include "MPU_Data.h"
#include <math.h>

#define G_ACCEL_VALUE   9.8105 //[m/s^2 / g]

static constexpr double getDegToRadScaleFactor()
{
    return M_PI / 180.0;
}

MPU_Data::MPU_Data(Accel_Sensitivity_Scale_Factor_t accel_scale, float gyro_scale, bool useMagnetometer)
:   _useMagnetometer(useMagnetometer),
    _accel_scale(accel_scale),
    _gyro_scale(gyro_scale)
{
    _data_container.reserve(4);

    if(useMagnetometer) {
        ROS_ERROR("Magnetometer functionality not implemented!\n");
        throw;
    }
};

/* inputBuffer size: 12B for acc + gyro, 18B for acc + gyro + mag */ 
void MPU_Data::convertRawToData(const std::vector<uint8_t>::const_iterator inputBuffer, MPU_Data_Struct_t &outputStruct)
{
    // Accelerometer section
    int16_t ax = (int16_t)( ((uint8_t)inputBuffer[0] << 8) | (uint8_t)inputBuffer[1] );
    int16_t ay = (int16_t)( ((uint8_t)inputBuffer[2] << 8) | (uint8_t)inputBuffer[3] );
    int16_t az = (int16_t)( ((uint8_t)inputBuffer[4] << 8) | (uint8_t)inputBuffer[5] );

    // Gyroscope section
    int16_t gx = (int16_t)( ((uint8_t)inputBuffer[6] << 8) | (uint8_t)inputBuffer[7] );
    int16_t gy = (int16_t)( ((uint8_t)inputBuffer[8] << 8) | (uint8_t)inputBuffer[9] );
    int16_t gz = (int16_t)( ((uint8_t)inputBuffer[10] << 8) | (uint8_t)inputBuffer[11] );

    outputStruct.accel_x = (double)((double)ax / _accel_scale) * G_ACCEL_VALUE;
    outputStruct.accel_y = (double)((double)ay / _accel_scale) * G_ACCEL_VALUE;
    outputStruct.accel_z = (double)((double)az / _accel_scale) * G_ACCEL_VALUE;

    outputStruct.gyro_x = (double)((double)gx / _gyro_scale) * getDegToRadScaleFactor();
    outputStruct.gyro_y = (double)((double)gy / _gyro_scale) * getDegToRadScaleFactor();
    outputStruct.gyro_z = (double)((double)gz / _gyro_scale) * getDegToRadScaleFactor();
}


bool MPU_Data::setData(const std::vector<uint8_t> &inputData)
{
    if(inputData[0] != 'S')
        return false;

    _data_container.clear();

    if(_useMagnetometer) {
        _numberOfSensors = (inputData.size() - 3) / DATA_SIZE_WITH_MAG;
        ROS_ERROR("Magnetometer functionality not implemented!\n");
        return false;
    }
    else {
        _numberOfSensors = (inputData.size() - 3) / DATA_SIZE_WITHOUT_MAG;

        for(int sensor_index = 0; sensor_index < _numberOfSensors; ++sensor_index)
        {
            MPU_Data_Struct_t tempDataStruct;
            tempDataStruct.timestamp = (inputData[1] << 8) | inputData[2];

            unsigned int dataOffset = sensor_index * DATA_SIZE_WITHOUT_MAG + 3;
            convertRawToData(inputData.begin() + dataOffset, tempDataStruct);
            
            _data_container.push_back(tempDataStruct);
        }
    }

    return true;
}

void MPU_Data::createDataRosMsg(const MPU_Data_Struct_t &inputData, sensor_msgs::Imu &msg)
{
  msg.header.stamp.nsec += (int32_t)inputData.timestamp * 1000000; //[ns]
  if(msg.header.stamp.nsec > 1000000000) {// if > 1s
    ++msg.header.stamp.sec;
    msg.header.stamp.nsec -= 1000000000;
  }
  // Set quaternion matrix as not provided
  msg.orientation_covariance[0] = -1.0;

  msg.linear_acceleration.x = inputData.accel_x;
  msg.linear_acceleration.y = inputData.accel_y;
  msg.linear_acceleration.z = inputData.accel_z;
  msg.linear_acceleration_covariance = {0.0};

  msg.angular_velocity.x = inputData.gyro_x;
  msg.angular_velocity.y = inputData.gyro_y;
  msg.angular_velocity.z = inputData.gyro_z;
  msg.angular_velocity_covariance = {0.0};
}
