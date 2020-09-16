#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <ce30_driver/ce30_driver.h>

using namespace std;
using namespace ce30_driver;

ros::Publisher gPub;
std::string gFrameID = "lidar";
std::string ipAddress = "192.168.1.80";
std::string topicName = "points";
int ipPort = 2368;

void DataReceiveCB(shared_ptr<PointCloud> cloud) {
  sensor_msgs::PointCloud pointcloud;
  pointcloud.header.stamp = ros::Time::now();
  pointcloud.header.frame_id = gFrameID;
  static int point_num = 320 * 20;
  pointcloud.points.reserve(point_num);
  for (auto& point : cloud->points) {
    static geometry_msgs::Point32 ros_point;
    ros_point.x = point.x;
    ros_point.y = point.y;
    ros_point.z = point.z;
    pointcloud.points.push_back(ros_point);
  }
  if (gPub.getNumSubscribers() > 0) {
    gPub.publish(pointcloud);
  }
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "ce30_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  pnh.param<std::string>("frame_id", gFrameID, gFrameID);
  pnh.param<std::string>("topic_name", topicName, topicName);
  pnh.param<std::string>("ip_address", ipAddress, ipAddress);
  pnh.param("ip_port", ipPort, 2368);

  ROS_INFO("Using frame:%s, IP: %s::%d, Topic: %s",gFrameID.c_str(), ipAddress.c_str(),ipPort, topicName.c_str());

  gPub = pnh.advertise<sensor_msgs::PointCloud>(topicName, 1);
  
  UDPServer server;
  server.SetIP(ipAddress);
  server.SetPort(ipPort);
  server.RegisterCallback(DataReceiveCB);  
  if (!server.Start()) {
    ROS_ERROR("Cannot connect to %s:%d", ipAddress.c_str(), ipPort); 
    return -1;
  }
  ROS_INFO("Connected to %s:%d", ipAddress.c_str(), ipPort); 
  while (ros::ok()) {
    server.SpinOnce();
  }
}
