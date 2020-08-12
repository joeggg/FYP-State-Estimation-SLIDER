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
#include "include/LowPassFilter.hpp"

ros::Publisher gyro_pub, accel_pub, vel_pub, pos_pub, q_pub, path_pub, marker_path_pub;

// Body state variables
Eigen::Vector3d a, a_bias, v, p, v_z, p_z, p_z_prev;
Eigen::Quaterniond q;
Eigen::VectorXd x;
Eigen::MatrixXd y_a_past, a_bias_past, z;

// Visualisations
nav_msgs::Path path;
visualization_msgs::MarkerArray marker_path;

Eigen::MatrixXd F, G, H, P, Q, R, K, I;
Eigen::Matrix3d Rot;

// Extra variables
Eigen::VectorXd y_a_means, a_bias_means, z_means, x_gt;
Eigen::Vector3d y_a;
int imu_data_count = 0;
int os_data_count = 0;
ros::Time timeperiod;
ros::Time timestamp;
double dt = 1;

// Constants
int x_size = 12;
int error_size = 6;
int z_size = 6;
int num_samples_x = 300; // no. samples taken for covariance of x
int num_samples_z = 10; // no. samples taken for covariance of z
const double pi = M_PI;
const Eigen::Vector3d g(0, 0, 9.81);
const double enocder_sigma = 0.068;
const double height = 0.8475;

void publish_data(void) {
  // Publish pose
  geometry_msgs::PoseStamped pos;
  pos.header.seq = imu_data_count;
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
  if(imu_data_count%100 == 0) {
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

/* Callback for new imu data point */
void callback(const sensor_msgs::Imu::ConstPtr& data) {
	// Store body orientation quaternion
	q.x() = data->orientation.x;
	q.y() = data->orientation.y;
	q.z() = data->orientation.z;
	q.w() = data->orientation.w;
	// Store acceleration
	y_a << data->linear_acceleration.x, data->linear_acceleration.y, data->linear_acceleration.z;

  // Measuring time period
	dt = (ros::Time::now() - timestamp).toSec();
	timestamp = ros::Time::now();

  // Restrict possible accelerations to remove noise before updating velocity
	if(dt < 1 && dt != 0) {
    // Update global position
    p += v*dt + a*(dt*dt)/2;
    if(p(2) < 0)
      p(2) = 0;
    if(p(2) > 1)
      p(2) = 0;
    // Update velocity
    v += a*dt;
    if(v.norm() > 20)
      v << 0, 0, 0;
    if(a.norm() == 0)
      v << 0, 0, 0;

    // Rotate accel from body to map frame and remove gravity
    Eigen::Quaterniond r;
    r.w() = 0;
    r.vec() = y_a;
    Eigen::Quaterniond rotatedR = q*r*q.inverse();
    a = rotatedR.vec() - g;
    // Accelerometer bias
    a_bias = a_bias;
  }

  publish_data();

  // Iterate sample counter
	imu_data_count++;
}

void os_callback(const geometry_msgs::PoseStamped::ConstPtr& data) {
  p_z << data->pose.position.x, data->pose.position.y, data->pose.position.z;
  if((ros::Time::now() - timeperiod).toSec() <= 0.05)
    v_z = (p_z-p_z_prev)/0.05;
  timeperiod = ros::Time::now();
  p_z_prev = p_z;

  // Innovation
  p << p_z(0), p_z(1), p_z(2);
  v << v_z(0), v_z(1), v_z(2);

  publish_data();

  // Iterate sample counter
  os_data_count++;
}

void gt_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& data) {
  x_gt(0) = data->pose.position.x;
  x_gt(1) = data->pose.position.y;
  x_gt(2) = data->pose.position.z;
}

void gt_velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& data) {
  x_gt(3) = data->vector.x;
  x_gt(4) = data->vector.y;
  x_gt(5) = data->vector.z;
}

/* Main */
int main(int argc, char **argv) {
  // ROS node handle
	ros::init(argc, argv, "estimator_test");
	ros::NodeHandle nh;
  // ROS publishers
	gyro_pub = nh.advertise<geometry_msgs::Vector3>("state/test/gyro", 1000);
	accel_pub = nh.advertise<geometry_msgs::PointStamped>("state/test/acceleration", 1000);
	vel_pub = nh.advertise<geometry_msgs::PointStamped>("state/test/velocity", 1000);
	pos_pub = nh.advertise<geometry_msgs::PoseStamped>("state/test/pose", 1000);
	q_pub = nh.advertise<geometry_msgs::Quaternion>("state/test/q", 1000);
  path_pub = nh.advertise<nav_msgs::Path>("state/test/path", 1000);
  marker_path_pub = nh.advertise<visualization_msgs::MarkerArray>("state/test/marker_path", 1000);
  // ROS subscribers
	ros::Subscriber sub = nh.subscribe("slider_gazebo/imu", 1000, callback);
  ros::Subscriber os_pose_sub = nh.subscribe("state/orb_slam/pose", 1000, os_callback);
  ros::Subscriber gt_pose_sub = nh.subscribe("state/ground_truth/pose", 1000, gt_pose_callback);
  ros::Subscriber gt_velocity_sub = nh.subscribe("state/ground_truth/velocity", 1000, gt_velocity_callback);

	timestamp = ros::Time::now();
  timeperiod = ros::Time::now();

  // State variables initialisation
  a << 0, 0, 0;
	v << 0, 0, 0;
	p << 0, 0, height;
  p_z = p;
  p_z_prev = p;
  a_bias << 0, 0, 0;

	x.setZero(x_size);
  z.setZero(z_size, num_samples_z);
  y_a_past.setZero(3, num_samples_x);
  a_bias_past.setZero(3, num_samples_x);
	y_a_means.setZero(3);
  a_bias_means.setZero(3);
  z_means.setZero(z_size);
  x_gt.setZero(x_size);

  // Kalman gain
  K.setZero(x_size, z_size);

  // Identity
  I.setIdentity(x_size, x_size);

  // State transition matrices
	F.setIdentity(x_size, x_size);
  G.setIdentity(x_size, error_size);
  H.setIdentity(z_size, x_size);

  // Rotation matrix
  Rot.setIdentity();

  // Estimate covariance
	P.setIdentity(x_size, x_size);
	P = 0.001*P;

  // Process/measurement covariance matrices
	Q.setIdentity(error_size, error_size);
  R.setIdentity(z_size, z_size);

	ros::spin();

	return 0;
}
