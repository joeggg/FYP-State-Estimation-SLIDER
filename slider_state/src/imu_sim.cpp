#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

#define _USE_MATH_DEFINES
#include <cmath>

const double pi = M_PI;

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_sim");
  ros::NodeHandle nh;
  ros::Rate r(100);

  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("sensor_data", 1000);

  while(ros::ok()) {
    sensor_msgs::Imu msg;
    msg.header.stamp = ros::Time::now();
    msg.orientation.x = 0;
    msg.orientation.y = 0;
    msg.orientation.z = 0;
    msg.orientation.w = 1;
    msg.linear_acceleration.x = 0;
    msg.linear_acceleration.y = 0;
    msg.linear_acceleration.z = 0;

    imu_pub.publish(msg);
    r.sleep();
  }

  return 0;
}
