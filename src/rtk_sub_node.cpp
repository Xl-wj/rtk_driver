#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"

#include <iostream>
#include <iomanip>

#include <fstream>

using namespace std;

void gpsCallback(const sensor_msgs::NavSatFix& gps_msgs) {
  //ROS_INFO("I heard time:[%ld], lat:[%lf], lng:[%lf], height[%lf]", gps_msgs.header.stamp, gps_msgs.longitude, gps_msgs.latitude, gps_msgs.altitude);
  cout << std::setprecision(15) << gps_msgs.latitude << " " << gps_msgs.longitude << " " << gps_msgs.altitude << endl;

  ofstream fs("/home/xl/rtk_pose_llh.txt", ios::app);
  fs << std::setprecision(15) << gps_msgs.latitude << " " << gps_msgs.longitude << " " << gps_msgs.altitude << endl;
}

int main(int argc,char **argv) {

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber my_sub = n.subscribe("gps_pub",50, gpsCallback);
  
  ros::spin();
  return 0;
}
