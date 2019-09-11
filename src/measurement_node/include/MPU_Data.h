#pragma once
#include <iostream>
#include <vector>

#include "ros/ros.h"
#include <ros/console.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"

#define DATA_SIZE_WITHOUT_MAG   12
#define DATA_SIZE_WITH_MAG      18

#define USE_ROS_TIME            1

struct MPU_Data_Struct_t{
    unsigned short  timestamp;
    double          accel_x;
    double          accel_y;
    double          accel_z;
    double          gyro_x;
    double          gyro_y;
    double          gyro_z;
};

struct MPU_Mag_Data_Struct_t{
    double          x;
    double          y;
    double          z;
};

enum Accel_Sensitivity_Scale_Factor_t{
    INV_FSR_2G = 16384,
    INV_FSR_4G = 8192,
    INV_FSR_8G = 4096,
    INV_FSR_16G = 2048
};

// float constants -> can't use enums
#define INV_FSR_250DPS  131.f
#define INV_FSR_500DPS  65.5f
#define INV_FSR_1000DPS 32.8f
#define INV_FSR_2000DPS 16.4f
// Gyro_Sensitivity_Scale_Factor_t;

#define AK89xx_FSR      (4915)

class MPU_Data
{
private:
    int             _accel_scale;
    float           _gyro_scale;
    float           _mag_scale;
    unsigned int    _numberOfSensors;
    bool            _useMagnetometer;
    bool            _biasSet = false;
    std::vector<MPU_Data_Struct_t>      _data_container;
    std::vector<MPU_Data_Struct_t>      _bias_container;
    std::vector<MPU_Mag_Data_Struct_t>  _mag_data_container;
    std::vector<MPU_Mag_Data_Struct_t>  _mag_bias_container;

    void convertRawToData(const std::vector<uint8_t>::const_iterator inputBuffer, MPU_Data_Struct_t &outputStruct);
    void convertRawMagToData(const std::vector<uint8_t>::const_iterator inputBuffer, MPU_Mag_Data_Struct_t &outputStruct);

public:
    MPU_Data(int numberOfSensors, Accel_Sensitivity_Scale_Factor_t accel_scale, float gyro_scale, bool useMagnetometer = false);

    bool setData(const std::vector<uint8_t> &inputData);
    bool setBias(const std::vector<uint8_t> &inputData);

    inline bool useMagnetometer()
    {
        return _useMagnetometer;
    }

    inline std::vector<MPU_Data_Struct_t> getData()
    {
        return _data_container;
    }

    inline std::vector<MPU_Mag_Data_Struct_t> getMagData()
    {
        return _mag_data_container;
    }

    inline int getSensorsNumber()
    {
        return _numberOfSensors;
    }

    static void createDataRosMsg(const MPU_Data_Struct_t &inputData, sensor_msgs::Imu &msg);
    static void createMagDataRosMsg(const MPU_Mag_Data_Struct_t &inputData, sensor_msgs::MagneticField &msg);
};


