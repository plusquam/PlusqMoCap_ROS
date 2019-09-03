#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <serial.h>
#include <terminal.h>

int main(int argc, char **argv)
{
  serial::Serial serial("/dev/ttvACM0", 230400U);

  ros::init(argc, argv, "measurement_node");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(0.010);

  while (ros::ok())
  {

    std_msgs::String msg;

    std::stringstream ss;
    ss << "dawaj kostke";
    msg.data = ss.str();


    ROS_INFO("%s", msg.data.c_str());


    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}


