#include "MPU_Data.h"
#include <math.h>

#define G_ACCEL_VALUE   9.8105 //[m/s^2 / g]

static constexpr double getDegToRadScaleFactor()
{
    return M_PI / 180.0;
}

static constexpr double getMagScaleFactor()
{
    return 1000000 / 0.15;
}

MPU_Data::MPU_Data(int numberOfSensors, Accel_Sensitivity_Scale_Factor_t accel_scale, float gyro_scale, bool useMagnetometer)
:   _numberOfSensors(numberOfSensors),
    _useMagnetometer(useMagnetometer),
    _accel_scale(accel_scale),
    _gyro_scale(gyro_scale),
    _mag_scale(getMagScaleFactor())
{
    _data_container.reserve(_numberOfSensors);
    _mag_data_container.reserve(_numberOfSensors);
};

/* inputBuffer size: 12B for acc + gyro */ 
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

/* inputBuffer size: 6B for mag */ 
void MPU_Data::convertRawMagToData(const std::vector<uint8_t>::const_iterator inputBuffer, MPU_Mag_Data_Struct_t &outputStruct)
{
    if(_useMagnetometer) {
        int16_t mx = (int16_t)( ((uint8_t)inputBuffer[0] << 8) | (uint8_t)inputBuffer[1] );
        int16_t my = (int16_t)( ((uint8_t)inputBuffer[2] << 8) | (uint8_t)inputBuffer[3] );
        int16_t mz = (int16_t)( ((uint8_t)inputBuffer[4] << 8) | (uint8_t)inputBuffer[5] );

        outputStruct.x = (double)((double)my / _mag_scale);     // y mag axis same as x axis of accel and gyro
        outputStruct.y = (double)((double)mx / _mag_scale);     // x mag axis same as y axis of accel and gyro
        outputStruct.z = -(double)((double)mz / _mag_scale);    // inverted z axis regarding to accel and gyro
    }
}


bool MPU_Data::setData(const std::vector<uint8_t> &inputData)
{
    if(inputData[0] != 'S')
        return false;

    _data_container.clear();
    if(_useMagnetometer)
        _mag_data_container.clear();
        
    unsigned int dataOffset = 3;

    for(int sensor_index = 0; sensor_index < _numberOfSensors; ++sensor_index)
    {
        MPU_Data_Struct_t tempDataStruct;

        if(sensor_index == 0)
            tempDataStruct.timestamp = (inputData[1] << 8) | inputData[2];

        convertRawToData(inputData.begin() + dataOffset, tempDataStruct);

        if(_biasSet)
        {
            tempDataStruct.accel_x -= _bias_container[sensor_index].accel_x;
            tempDataStruct.accel_y -= _bias_container[sensor_index].accel_y;
            tempDataStruct.accel_z -= _bias_container[sensor_index].accel_z;
            tempDataStruct.gyro_x -= _bias_container[sensor_index].gyro_x;
            tempDataStruct.gyro_y -= _bias_container[sensor_index].gyro_y;
            tempDataStruct.gyro_z -= _bias_container[sensor_index].gyro_z;
        }

        _data_container.push_back(tempDataStruct);

        dataOffset += 12; // Step to mag sensor data or skip to next sensor

        if(_useMagnetometer)
        {
            MPU_Mag_Data_Struct_t tempMagDataStruct;
            convertRawMagToData(inputData.begin() + dataOffset, tempMagDataStruct);

            if(_biasSet)
            {
                tempMagDataStruct.x -= _mag_bias_container[sensor_index].x;
                tempMagDataStruct.y -= _mag_bias_container[sensor_index].y;
                tempMagDataStruct.z -= _mag_bias_container[sensor_index].z;
            }
            
            _mag_data_container.push_back(tempMagDataStruct);
            dataOffset += 6; // Step to next sensor data
        }
    }

    return true;
}

bool MPU_Data::setBias(const std::vector<uint8_t> &inputData)
{
    if(inputData[0] != 'B')
        return false;

    _bias_container.clear();
    if(_useMagnetometer)
        _mag_bias_container.clear();

    unsigned int dataOffset = 1;
    for(int sensor_index = 0; sensor_index < _numberOfSensors; ++sensor_index)
    {
        MPU_Data_Struct_t tempDataStruct;
        tempDataStruct.timestamp = 0;

        convertRawToData(inputData.begin() + dataOffset, tempDataStruct);

        _bias_container.push_back(tempDataStruct);
        ROS_INFO("Bias estimation for sensor nr %i:", sensor_index);
        ROS_INFO("Acc: x=%f, y=%f, z=%f", tempDataStruct.accel_x, tempDataStruct.accel_y, tempDataStruct.accel_z);
        ROS_INFO("Gyro: x=%f, y=%f, z=%f", tempDataStruct.gyro_x, tempDataStruct.gyro_y, tempDataStruct.gyro_z);

        dataOffset += 12; // Step to mag sensor data or skip to next sensor

        if(_useMagnetometer) {
            
            MPU_Mag_Data_Struct_t tempMagDataStruct;
            convertRawMagToData(inputData.begin() + dataOffset, tempMagDataStruct);
            _mag_bias_container.push_back(tempMagDataStruct);
            ROS_INFO("Mag: x=%f, y=%f, z=%f", tempMagDataStruct.x, tempMagDataStruct.y, tempMagDataStruct.z);

            dataOffset += 6; // Step to next sensor data
        }
    }

    _biasSet = true;
    ROS_INFO("Bias estimation completed\n");

    return true;
}

void MPU_Data::createDataRosMsg(const MPU_Data_Struct_t &inputData, sensor_msgs::Imu &msg)
{
#if USE_ROS_TIME
        msg.header.stamp = ros::Time::now();
#else
        msg.header.stamp.nsec += (int32_t)inputData.timestamp * 1000000; //[ns]
        if(msg.header.stamp.nsec > 1000000000) {// if > 1s
            ++msg.header.stamp.sec;
            msg.header.stamp.nsec -= 1000000000;
        }
#endif
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

void MPU_Data::createMagDataRosMsg(const MPU_Mag_Data_Struct_t &inputData, sensor_msgs::MagneticField &msg)
{
#if USE_ROS_TIME
        msg.header.stamp = ros::Time::now();
#else
        msg.header.stamp.nsec += (int32_t)inputData.timestamp * 1000000; //[ns]
        if(msg.header.stamp.nsec > 1000000000) {// if > 1s
            ++msg.header.stamp.sec;
            msg.header.stamp.nsec -= 1000000000;
        }
#endif

    msg.magnetic_field.x = inputData.x;
    msg.magnetic_field.y = inputData.y;
    msg.magnetic_field.z = inputData.z;
    msg.magnetic_field_covariance = {0.0};
}