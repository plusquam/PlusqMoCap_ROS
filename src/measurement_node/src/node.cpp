#include <iostream>
#include <sstream>
#include <thread>
#include <mutex>
#include <unistd.h>

#include <serial.h>
#include <terminal.h>

#include "MPU_Data.h"
#include "imu_callbacks.h"

#define SERIAL_PORT_NAME "/dev/ttyACM0"
#define USE_MAG         false
#define SENSORS_NUMBER  4

std::mutex  general_node_mutex, command_mutex, serial_mutex;

enum Command_enum_t{
  CMD_MEASUREMENT_START_STOP  = 'S',
  CMD_CONNECT                 = 'C',
  CMD_CALIBRATION             = 'B',
  CMD_QUIT                    = 'Q',
  CMD_NONE
} command_char = CMD_NONE;

MPU_Data mpuData(SENSORS_NUMBER, INV_FSR_4G, INV_FSR_500DPS, USE_MAG);

bool  serialDataReady = false;
bool  serialPortConnect = false;
Command_enum_t  sendMeasurementCommandStart = CMD_NONE;

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
          sendMeasurementCommandStart = CMD_MEASUREMENT_START_STOP;
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

        case 'B':
        case 'b':
        {
          command_mutex.lock();
          command_char = CMD_CALIBRATION;
          command_mutex.unlock();
          serial_mutex.lock();
          sendMeasurementCommandStart = CMD_CALIBRATION;
          serial_mutex.unlock();
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

  serial.flush();
  (void)serial.read(serial.available());

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

    switch(sendMeasurementCommandStart)
    {
      case CMD_MEASUREMENT_START_STOP:
      {
        serial.write(std::string("S"));
        ROS_INFO("Start/Stop command sent");
      }
        break;

      case CMD_CALIBRATION:
      {
        serial.write(std::string("B"));
        ROS_INFO("Start calibration command sent");
      }
        break;
    }

    sendMeasurementCommandStart = CMD_NONE;
    serial_mutex.unlock();
      
    size_t bytesToRead = serial.available();
    std::string inputDataLine("");

    if(bytesToRead)
    {
      try{
        inputDataLine = serial.read(1);

        if(inputDataLine[0] == 'S') {
          int neededDataNumber = 5; // data for time + 'S' + '\r\n'
          
          if(mpuData.useMagnetometer())
            neededDataNumber += (DATA_SIZE_WITH_MAG * mpuData.getSensorsNumber());
          else
            neededDataNumber += (DATA_SIZE_WITHOUT_MAG * mpuData.getSensorsNumber());

          inputDataLine += serial.readline(neededDataNumber, "\r\n");

          while(inputDataLine.length() < neededDataNumber) {
            if(serial.available())
              inputDataLine += serial.readline(neededDataNumber, "\r\n");
            else {
              usleep(1000);
            }
          }

          if((int)inputDataLine.length() == neededDataNumber) {
            serial_mutex.lock();
            serialDataReady = mpuData.setData(std::vector<uint8_t>(inputDataLine.begin(), inputDataLine.end()));
            serial_mutex.unlock();
          }
          else
          {
            ROS_ERROR("Still bad length -> actual: %d | needed: %d", (int)inputDataLine.length(), neededDataNumber);
            ROS_ERROR("Last character ASCII: %d", inputDataLine.back());
          }
        }
        else if(inputDataLine[0] == 'B') {
          inputDataLine += serial.read(DATA_SIZE_WITHOUT_MAG * mpuData.getSensorsNumber() + 2);
          serial_mutex.lock();
          (void)mpuData.setBias(std::vector<uint8_t>(inputDataLine.begin(), inputDataLine.end()));
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
  ros::NodeHandle node;

  std::cout << "List of key input commands:" << std::endl;
  std::cout << "'C' - connect with dongle," << std::endl;
  std::cout << "'S' - start/stop measurement," << std::endl;
  std::cout << "'B' - start calibration," << std::endl;
  std::cout << "'Q' - quit node." << std::endl << std::endl;

  std::thread serial_thread;
  std::thread command_thread(command_thread_fun);

  // IMU raw data message
  ros::Publisher imuTopicPublisher0 = node.advertise<sensor_msgs::Imu>("imu/data_raw/0", 3, true);
  sensor_msgs::Imu imu_msg0;
  imu_msg0.header.frame_id = "imu_tf_0";
  imu_msg0.header.stamp.sec = 0;
  imu_msg0.header.stamp.nsec = 0;

  ros::Publisher imuTopicPublisher1 = node.advertise<sensor_msgs::Imu>("imu/data_raw/1", 3, true);
  sensor_msgs::Imu imu_msg1;
  imu_msg1.header.frame_id = "imu_tf_1";
  imu_msg1.header.stamp.sec = 0;
  imu_msg1.header.stamp.nsec = 0;

  ros::Publisher imuTopicPublisher2 = node.advertise<sensor_msgs::Imu>("imu/data_raw/2", 3, true);
  sensor_msgs::Imu imu_msg2;
  imu_msg2.header.frame_id = "imu_tf_2";
  imu_msg2.header.stamp.sec = 0;
  imu_msg2.header.stamp.nsec = 0;

  ros::Publisher imuTopicPublisher3 = node.advertise<sensor_msgs::Imu>("imu/data_raw/3", 3, true);
  sensor_msgs::Imu imu_msg3;
  imu_msg3.header.frame_id = "imu_tf_3";
  imu_msg3.header.stamp.sec = 0;
  imu_msg3.header.stamp.nsec = 0;
  

#if USE_MAG == true
  // Mag data message
  ros::Publisher magTopicPublisher = node.advertise<sensor_msgs::MagneticField>("imu/mag", 1, true);
  sensor_msgs::MagneticField mag_msg;
  mag_msg.header.frame_id = "imu_tf";
  mag_msg.header.stamp.sec = 0;
  mag_msg.header.stamp.nsec = 0;
#endif

  // Initialize callbacks for imu filtered data
  initializeCallbacks(node);

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
      MPU_Data::createDataRosMsg(data_containers[0], imu_msg0);
      imuTopicPublisher0.publish(imu_msg0);
      MPU_Data::createDataRosMsg(data_containers[1], imu_msg1);
      imuTopicPublisher1.publish(imu_msg1);
      MPU_Data::createDataRosMsg(data_containers[2], imu_msg2);
      imuTopicPublisher2.publish(imu_msg2);
      MPU_Data::createDataRosMsg(data_containers[3], imu_msg3);
      imuTopicPublisher3.publish(imu_msg3);

#if USE_MAG == true
      // Publish Mag data
      if(mpuData.useMagnetometer()) {
        std::vector<MPU_Mag_Data_Struct_t> mag_data_containers(mpuData.getMagData());
        MPU_Data::createMagDataRosMsg(mag_data_containers[0], mag_msg);
        magTopicPublisher.publish(mag_msg);
      }
#endif
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
          imu_msg0.header.stamp.sec = 0;
          imu_msg0.header.stamp.nsec = 0;
          imu_msg1.header.stamp.sec = 0;
          imu_msg1.header.stamp.nsec = 0;
          imu_msg2.header.stamp.sec = 0;
          imu_msg2.header.stamp.nsec = 0;
          imu_msg3.header.stamp.sec = 0;
          imu_msg3.header.stamp.nsec = 0;
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


