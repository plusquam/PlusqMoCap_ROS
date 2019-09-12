#include "imu_callbacks.h"
#include <tf2_ros/transform_broadcaster.h>
#include <fstream>


static ros::Subscriber imuFilteredSubscriber0;
static ros::Subscriber imuFilteredSubscriber1;
static ros::Subscriber imuFilteredSubscriber2;
static ros::Subscriber imuFilteredSubscriber3;

static std::ofstream logFile0;
static std::ofstream logFile1;
static std::ofstream logFile2;
static std::ofstream logFile3;

static void initializeLogFiles()
{
    logFile0 = std::ofstream ("log_sensor_0.csv", std::ios_base::out);
    logFile0 << "id ramki, czas [ms], pozycja_x [m], pozycja_y [m], pozycja_z [m]\n";
    logFile1 = std::ofstream ("log_sensor_1.csv", std::ios_base::out);
    logFile1 << "id ramki, czas [ms], pozycja_x [m], pozycja_y [m], pozycja_z [m]\n";
    logFile2 = std::ofstream ("log_sensor_2.csv", std::ios_base::out);
    logFile2 << "id ramki, czas [ms], pozycja_x [m], pozycja_y [m], pozycja_z [m]\n";
    logFile3 = std::ofstream ("log_sensor_3.csv", std::ios_base::out);
    logFile3 << "id ramki, czas [ms], pozycja_x [m], pozycja_y [m], pozycja_z [m]\n";
}


void logData(std::ofstream &logFile, long dataNumber, long timestamp, const tf2::Vector3 &position) {
  logFile << dataNumber << ",";
  logFile << timestamp << ",";
  logFile << position.m_floats[0] << ",";
  logFile << position.m_floats[1] << ",";
  logFile << position.m_floats[2];
  logFile << "\n";
}

static void ImuDataCallback_0(const sensor_msgs::Imu::ConstPtr &msg)
{
  static tf2_ros::TransformBroadcaster br;
  static Imu_Motion imuMotion;
  static int loopCounter = 0;

  if(loopCounter > 40) { 
    static unsigned long dataCounter = 0;
    geometry_msgs::TransformStamped transformStamped;

    imuMotion.process(msg);

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "imu_tf_0";

    tf2::Vector3 position = imuMotion.getPosition();
    tf2::convert(position, transformStamped.transform.translation);
    transformStamped.transform.rotation = msg->orientation;
    br.sendTransform(transformStamped);

    if(dataCounter % 10 == 0)
      logData(logFile0, dataCounter, dataCounter * 5, position);

    dataCounter++;
  }
  else {
    loopCounter++;
  }
}

static void ImuDataCallback_1(const sensor_msgs::Imu::ConstPtr &msg)
{
  static tf2_ros::TransformBroadcaster br;
  static Imu_Motion imuMotion;
  static int loopCounter = 0;

  if(loopCounter > 40) { 
    static unsigned long dataCounter = 0;
    geometry_msgs::TransformStamped transformStamped;

    imuMotion.process(msg);

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "imu_tf_1";

    tf2::Vector3 position = imuMotion.getPosition();
    tf2::convert(position, transformStamped.transform.translation);
    transformStamped.transform.rotation = msg->orientation;
    br.sendTransform(transformStamped);

    if(dataCounter % 10 == 0)
      logData(logFile1, dataCounter, dataCounter * 5, position);

    dataCounter++;
  }
  else {
    loopCounter++;
  }
}

static void ImuDataCallback_2(const sensor_msgs::Imu::ConstPtr &msg)
{
  static tf2_ros::TransformBroadcaster br;
  static Imu_Motion imuMotion;
  static int loopCounter = 0;

  if(loopCounter > 40) { 
    static unsigned long dataCounter = 0;
    geometry_msgs::TransformStamped transformStamped;

    imuMotion.process(msg);

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "imu_tf_2";

    tf2::Vector3 position = imuMotion.getPosition();
    tf2::convert(position, transformStamped.transform.translation);
    transformStamped.transform.rotation = msg->orientation;
    br.sendTransform(transformStamped);

    if(dataCounter % 10 == 0)
      logData(logFile2, dataCounter, dataCounter * 5, position);

    dataCounter++;
  }
  else {
    loopCounter++;
  }
}

static void ImuDataCallback_3(const sensor_msgs::Imu::ConstPtr &msg)
{
  static tf2_ros::TransformBroadcaster br;
  static Imu_Motion imuMotion;
  static int loopCounter = 0;

  if(loopCounter > 40) { 
    static unsigned long dataCounter = 0;
    geometry_msgs::TransformStamped transformStamped;

    imuMotion.process(msg);

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "imu_tf_3";

    tf2::Vector3 position = imuMotion.getPosition();
    tf2::convert(position, transformStamped.transform.translation);
    transformStamped.transform.rotation = msg->orientation;
    br.sendTransform(transformStamped);

    if(dataCounter % 10 == 0)
      logData(logFile3, dataCounter, dataCounter * 5, position);

    dataCounter++;
  }
  else {
    loopCounter++;
  }
}

void initializeCallbacks(ros::NodeHandle &node)
{
    imuFilteredSubscriber0 = node.subscribe("/imu/data/0", 10, &ImuDataCallback_0);
    imuFilteredSubscriber1 = node.subscribe("/imu/data/1", 10, &ImuDataCallback_1);
    imuFilteredSubscriber2 = node.subscribe("/imu/data/2", 10, &ImuDataCallback_2);
    imuFilteredSubscriber3 = node.subscribe("/imu/data/3", 10, &ImuDataCallback_3);

    initializeLogFiles();
}

