/*
 * test.cpp
 *
 *  Created on: 12 Nov 2019
 *      Author: joe
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/MarkerArray.h"
#include <iostream>
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Path.h"
#include "eigen3/Eigen/Dense"
#define _USE_MATH_DEFINES
#include <cmath>
#include <imu_3dm_gx4/FilterOutput.h>

ros::Publisher gyro_pub, accel_pub, vel_pub, pos_pub, q_pub, path_pub, marker_path_pub;

// Body state variables
Eigen::Vector3d v, p, v_z, p_z, p_z_prev;
Eigen::Quaterniond q;
Eigen::VectorXd x;
Eigen::MatrixXd y_a_past, a_bias_past, z;

// Visualisations
nav_msgs::Path path;
visualization_msgs::MarkerArray marker_path;

// Extra variables
int os_data_count = 0;
int data_count = 0;
ros::Time timeperiod;
double dt = 1;

// Constants
const double height = 0.8475;

void publish_data(void) {
  // Publish pose
  geometry_msgs::PoseStamped pos;
  pos.header.seq = data_count;
  pos.header.frame_id = "map";
  pos.pose.position.x = p(0);
  pos.pose.position.y = p(1);
  pos.pose.position.z = p(2);
  pos.pose.orientation.x = q.x();
  pos.pose.orientation.y = q.y();
  pos.pose.orientation.z = q.z();
  pos.pose.orientation.w = q.w();
  pos_pub.publish(pos);
  // Path
  path.header.frame_id = "map";
  path.poses.push_back(pos);
  // Publish path every second to reduce load
  if(data_count%100 == 0) {
    path_pub.publish(path);
    marker_path_pub.publish(marker_path);
  }
  // Set tf frame
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(p(0), p(1), p(2)));
  transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
}

void os_callback(const geometry_msgs::PoseStamped::ConstPtr& data) {
  p_z << data->pose.position.x, data->pose.position.y, data->pose.position.z;
  if((ros::Time::now() - timeperiod).toSec() <= 0.05)
    v_z = (p_z-p_z_prev)/0.05;
  timeperiod = ros::Time::now();
  p_z_prev = p_z;

  q.w() = data->pose.orientation.w;
  q.x() = data->pose.orientation.x;
  q.y() = data->pose.orientation.y;
  q.z() = data->pose.orientation.z;

  // Innovation
  p << p_z(0), p_z(1), p_z(2);
  v << v_z(0), v_z(1), v_z(2);

  publish_data();

  // Iterate sample counter
  os_data_count++;
}

/* Main */
int main(int argc, char **argv) {
  // ROS node handle
	ros::init(argc, argv, "interpolator");
	ros::NodeHandle nh;
  // ROS publishers
	pos_pub = nh.advertise<geometry_msgs::PoseStamped>("state/interpolator/pose", 1000);
  path_pub = nh.advertise<nav_msgs::Path>("state/interpolator/path", 1000);
  marker_path_pub = nh.advertise<visualization_msgs::MarkerArray>("state/interpolator/marker_path", 1000);
  // ROS subscribers
  ros::Subscriber os_pose_sub = nh.subscribe("state/orb_slam/pose", 1000, os_callback);

  timeperiod = ros::Time::now();

  // State variables initialisation
	v << 0, 0, 0;
	p << 0, 0, height;
  p_z = p;
  p_z_prev = p;
  v_z << 0, 0, 0;

  ros::Rate r(1000);
  while (ros::ok()) {
    p += 0.001*v;
    publish_data();
    data_count++;
    ros::spinOnce();
    r.sleep();
  }

	return 0;
}
