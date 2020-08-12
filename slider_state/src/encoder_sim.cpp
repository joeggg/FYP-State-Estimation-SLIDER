#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

#define _USE_MATH_DEFINES
#include <cmath>

const double pi = M_PI;

int main(int argc, char** argv) {
  ros::init(argc, argv, "encoder_sim");
  ros::NodeHandle nh;
  double count = 0;

  ros::Publisher enc_pub = nh.advertise<sensor_msgs::JointState>("enc_out", 1000);
  ros::Publisher sup_pub = nh.advertise<std_msgs::Float64>("sup_out", 1000);

  std_msgs::Float64 mode;
  sensor_msgs::JointState joints;
  joints.name = {"alphaR_r", "alphaL_r", "alphaR_p", "alphaL_p", "betaR_r", "betaL_r", "betaR_p", "betaL_p", "gammaR", "gammaL"};
  joints.position = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  joints.velocity = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  joints.effort = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  ros::Rate r(1000);

  while(ros::ok()) {
    joints.header.seq = count;
    joints.position[0] = 0;//0.5*sin(count/500);
    joints.position[1] = 0;//0.5*sin(count/500);
    joints.position[4] = 0;//0.5*sin(count/300);
    joints.position[5] = 0;//0.5*sin(count/300);;

    joints.position[2] = 0.5*sin(count/300);
    joints.position[3] = 0.5*sin(count/300);
    joints.position[6] = 0.5*sin(count/300);
    joints.position[7] = 0.5*sin(count/300);

    joints.position[8] = 0;//0.25 + 0.25*sin(count/300);
    joints.position[9] = 0;//0.25 + 0.25*sin(count/300);

    mode.data = 10*sin(count/300);

    sup_pub.publish(mode);
    enc_pub.publish(joints);
    r.sleep();
    count++;
  }

  return 0;
}
