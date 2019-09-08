#include <iostream>
#include <sstream>
#include <thread>
#include <mutex>

#include <serial.h>
#include <terminal.h>

#include "MPU_Data.h"

#define SERIAL_PORT_NAME "/dev/ttyACM0"

std::mutex  general_node_mutex, command_mutex, serial_mutex;

enum Command_enum_t{
  CMD_MEASUREMENT_START_STOP  = 'S',
  CMD_CONNECT                 = 'C',
  CMD_QUIT                    = 'Q',
  CMD_NONE
} command_char = CMD_NONE;

MPU_Data mpuData(INV_FSR_2G, INV_FSR_250DPS, false);

bool  serialDataReady = false;
bool  serialPortConnect = false;
bool  sendMeasurementCommandStart = false;

bool  keepRunning = true;

void command_thread_fun(void)
{
  ros::Rate loop_rate(20);

  while(ros::ok())
  {
    char c = nonblocking_getch();
    if(c) {
      switch(c)
      {
        case 'S':
        case 's':
        {
          command_mutex.lock();
          command_char = CMD_MEASUREMENT_START_STOP;
          command_mutex.unlock();
          serial_mutex.lock();
          sendMeasurementCommandStart = true;
          serial_mutex.unlock();
        }
          break;

        case 'C':
        case 'c':
        {
          command_mutex.lock();
          command_char = CMD_CONNECT;
          command_mutex.unlock();
        }
          break;

        case 'Q':
        case 'q':
        {
          command_mutex.lock();
          command_char = CMD_QUIT;
          command_mutex.unlock();
          ROS_INFO("Command thread closed.");
          return;
        }
          break;

        default:
        {
          ROS_WARN("Unknown command '%c'.\r\n", c);
        }
          break;
      }

      std::cout << std::endl;
    }

    loop_rate.sleep();
  }
}



void serial_thread_fun(void)
{
  serial::Serial serial(SERIAL_PORT_NAME, 230400U);

  if(!serial.isOpen()) {
    ROS_ERROR("Serial is not opened!");
    throw;
  }

  serialPortConnect = true;
  ROS_INFO("Port opened.");

  serial.write(std::string("C"));
  ROS_INFO("Remote device connection command sent.");

  ros::Rate loop_rate(300); // 300Hz loop rate

  general_node_mutex.lock();
  while(ros::ok() && serial.isOpen() && keepRunning)
  {
    general_node_mutex.unlock();

    serial_mutex.lock();
    if(!serialPortConnect)
      break;

    if(sendMeasurementCommandStart) {
      serial.write(std::string("S"));
      sendMeasurementCommandStart = false;
      ROS_INFO("Start/Stop command sent");
    }
    serial_mutex.unlock();
      
    size_t bytesToRead = serial.available();
    std::string inputDataLine;

    if(bytesToRead)
    {
      try{
        inputDataLine = serial.read(1);
        if(inputDataLine[0] == 'S' && bytesToRead >= DATA_SIZE_WITHOUT_MAG * 3) {
          inputDataLine += serial.read(DATA_SIZE_WITHOUT_MAG * 3 + 2);

          serial_mutex.lock();
          serialDataReady = mpuData.setData(std::vector<uint8_t>(inputDataLine.begin(), inputDataLine.end()));
          serial_mutex.unlock();
        }
      }
      catch (std::runtime_error &e)
      {
        ROS_ERROR("Cannot read from serial port: %s", e.what());
        throw;
      }
    }

    loop_rate.sleep();
    general_node_mutex.lock();
  }
  general_node_mutex.unlock();
  serial_mutex.unlock();

  if(serial.isOpen()) {
    serial.close();
    ROS_INFO("Serial port %s closed.", SERIAL_PORT_NAME);
  }
}


int main(int argc, char **argv)
{
  // Node init
  ros::init(argc, argv, "measurement_node");
  ros::NodeHandle n;

  std::cout << "List of key input commands:" << std::endl;
  std::cout << "'C' - connect with dongle," << std::endl;
  std::cout << "'S' - start/stop measurement," << std::endl;
  std::cout << "'Q' - quit node." << std::endl << std::endl;

  std::thread serial_thread;
  std::thread command_thread(command_thread_fun);

  // Output data message
  ros::Publisher imuTopicPublisher = n.advertise<sensor_msgs::Imu>("imu/data_raw", 10, true);
  sensor_msgs::Imu imu_msg;
  imu_msg.header.frame_id = "imu_tf";
  imu_msg.header.stamp.sec = 0;
  imu_msg.header.stamp.nsec = 0;

  ros::Rate loop_rate(300); // 300Hz loop rate
  
  general_node_mutex.lock();
  while (ros::ok() && keepRunning)
  {
    general_node_mutex.unlock();

    // Read data from sensors
    serial_mutex.lock();
    if(serialDataReady) {
      std::vector<MPU_Data_Struct_t> data_containers(mpuData.getData());
      serialDataReady = false;
      serial_mutex.unlock();

      // Publish IMU data
      MPU_Data::createDataRosMsg(data_containers[1], imu_msg);
      imuTopicPublisher.publish(imu_msg);
    }
    else
    {
      serial_mutex.unlock();
    }
    

    // Read command
    command_mutex.lock();
    char command_local = command_char;
    command_char = CMD_NONE;
    command_mutex.unlock();

    switch(command_local)
    {
      case CMD_CONNECT:
        if(!serialPortConnect) {
          ROS_INFO("Connecting serial port %s...", SERIAL_PORT_NAME);
          imu_msg.header.stamp.sec = 0;
          imu_msg.header.stamp.nsec = 0;
          serial_thread = std::thread(serial_thread_fun);
        }
        else {
          ROS_INFO("Disconnecting serial port %s...", SERIAL_PORT_NAME);
          serial_mutex.lock();
          serialPortConnect = false;
          serial_mutex.unlock();
          if(serial_thread.joinable())
            serial_thread.join();
        }
        break;
      case CMD_QUIT:
        ROS_INFO("Exiting node...\r\n");
        general_node_mutex.lock();
        keepRunning = false;
        general_node_mutex.unlock();
        break;
    }

    ros::spinOnce();
    loop_rate.sleep();
    general_node_mutex.lock();
  }
  general_node_mutex.unlock();

  if(command_thread.joinable())
    command_thread.join();

  if(serial_thread.joinable())
    serial_thread.join();

  ROS_INFO("NODE CLOSED");

  return 0;
}


