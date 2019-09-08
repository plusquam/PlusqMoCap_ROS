#pragma once
#include <iostream>
#include <vector>

#include "ros/ros.h"
#include <ros/console.h>
#include "sensor_msgs/Imu.h"

#define DATA_SIZE_WITHOUT_MAG   12
#define DATA_SIZE_WITH_MAG      18

struct MPU_Data_Struct_t{
    unsigned short  timestamp;
    double          accel_x;
    double          accel_y;
    double          accel_z;
    double          gyro_x;
    double          gyro_y;
    double          gyro_z;
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

class MPU_Data
{
private:
    int             _accel_scale;
    float           _gyro_scale;
    unsigned int    _numberOfSensors;
    bool            _useMagnetometer;
    std::vector<MPU_Data_Struct_t> _data_container;

    void convertRawToData(const std::vector<uint8_t>::const_iterator inputBuffer, MPU_Data_Struct_t &outputStruct);

public:
    MPU_Data(Accel_Sensitivity_Scale_Factor_t accel_scale, float gyro_scale, bool useMagnetometer = false);

    bool setData(const std::vector<uint8_t> &inputData);
    inline std::vector<MPU_Data_Struct_t> getData()
    {
        return _data_container;
    }
    static void createDataRosMsg(const MPU_Data_Struct_t &inputData, sensor_msgs::Imu &msg);
};


