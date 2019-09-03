#include <iostream>
#include <sstream>
#include <thread>
#include <mutex>

#include "ros/ros.h"
#include <ros/console.h>
#include "std_msgs/String.h"

#include <serial.h>
#include <terminal.h>

std::mutex  command_mutex;
enum{
  CMD_MEASUREMENT_START_STOP  = 'S',
  CMD_CONNECT                 = 'C',
  CMD_QUIT                    = 'Q',
} command_char;

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

int main(int argc, char **argv)
{
  // Node init
  ros::init(argc, argv, "measurement_node");
  ros::NodeHandle n;

  std::cout << "List of key input commands:" << std::endl;
  std::cout << "'C' - connect with dongle," << std::endl;
  std::cout << "'S' - start/stop measuement," << std::endl;
  std::cout << "'Q' - quit node." << std::endl;

  std::thread command_thread(command_thread_fun);
  command_thread.detach();

  // serial::Serial serial("/dev/ttvACM0", 230400U);

  

  // ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(100);
  bool keepRunning = true;

  while (ros::ok() && keepRunning)
  {
    // std_msgs::String msg;
    // std::stringstream ss;
    // ss << "dawaj kostke";
    // msg.data = ss.str();
    // ROS_INFO("%s", msg.data.c_str());
    // chatter_pub.publish(msg);

    // Read command
    command_mutex.lock();
    char command_local = command_char;
    command_mutex.unlock();

    switch(command_local)
    {
      case CMD_QUIT:
        ROS_INFO("Exiting node...\r\n");
        keepRunning = false;
        break;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  if(command_thread.joinable())
    command_thread.join();

  return 0;
}


