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
ros::Time timestamp;
ros::Time timeperiod;

// Constants
int x_size = 12;
int error_size = 6;
int z_size = 6;
int num_samples_x = 1000; // no. samples taken for covariance of x
int num_samples_z = 20; // no. samples taken for covariance of z
const double pi = M_PI;
const Eigen::Vector3d g(0, 0, 9.81);
const double enocder_sigma = 0.068;
const double height = 0.8475;

void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove) {
    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols()-1;

    if( colToRemove < numCols )
        matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.block(0,colToRemove+1,numRows,numCols-colToRemove).eval();

    matrix.conservativeResize(numRows,numCols);
}

void generateCovariance(Eigen::MatrixXd& cov, Eigen::MatrixXd& data, Eigen::VectorXd& means, int num_samples, int cov_size) {
  // Calculate means
  means.setZero(cov_size);
  for(int i = 0; i < num_samples; i++) {
		means += data.col(i);
	}
  means = means/num_samples;

	// Calculate covariance
  cov.setZero();
  for(int i = 0; i < cov_size; i++) {
    for(int k = 0; k < num_samples; k++) {
      cov(i,i) += pow((data(i,k)-means(i)),2);
    }
    cov(i,i) = cov(i,i)/num_samples;
		// for(int j = i+1; j < cov_size; j++) {
		// 	for(int k = 0; k < num_samples; k++) {
		// 		cov(i,j) += (data(i,k)-means(i))*(data(j,k)-means(j));
		// 	}
		// 	cov(i,j) = cov(i,j)/num_samples;
    //   cov(j,i) = cov(i,j);
		// }
	}
}

void generateQ(void) {
  // Calculate means
  y_a_means.setZero(3);
  a_bias_means.setZero(3);
  for(int i = 0; i < num_samples_x; i++) {
		y_a_means += y_a_past.col(i);
    a_bias_means += a_bias_past.col(i);
	}
  y_a_means = a_bias_means/num_samples_x;
  a_bias_means = a_bias_means/num_samples_x;

	//	Calculate Q (covariance)
  Q.setZero();
  for(int i = 0; i < 3; i++) {
    for(int k = 0; k < num_samples_x; k++) {
      Q(i,i) += pow((y_a_past(i,k)-y_a_means(i)),2);
    }
    Q(i,i) = Q(i,i)/num_samples_x;
		// for(int j = i+1; j < 3; j++) {
		// 	for(int k = 0; k < num_samples_x; k++) {
		// 		Q(i,j) += (y_a_past(i,k)-y_a_means(i))*(y_a_past(j,k)-y_a_means(j));
		// 	}
		// 	Q(i,j) = Q(i,j)/num_samples_x;
    //   Q(j,i) = Q(i,j);
		// }
	}
  for(int i = 3; i < 6; i++) {
    for(int k = 0; k < num_samples_x; k++) {
      Q(i,i) += pow((a_bias_past(i-3,k)-a_bias_means(i-3)),2);
    }
    Q(i,i) = Q(i,i)/num_samples_x;
		// for(int j = i+1; j < 6; j++) {
		// 	for(int k = 0; k < num_samples_x; k++) {
		// 		Q(i,j) += (a_bias_past(i-3,k)-a_bias_means(i-3))*(a_bias_past(j-3,k)-a_bias_means(j-3));
		// 	}
		// 	Q(i,j) = Q(i,j)/num_samples_x;
    //   Q(j,i) = Q(i,j);
		// }
	}
}

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
	double dt = (ros::Time::now() - timestamp).toSec();
	timestamp = ros::Time::now();

  // Restrict possible accelerations to remove noise before updating velocity
	if(dt < 1) {
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
	// Add new accelerometer data and bias into matrices
	removeColumn(y_a_past, 0);
  removeColumn(a_bias_past, 0);
	y_a_past.conservativeResize(3, num_samples_x);
  a_bias_past.conservativeResize(3, num_samples_x);

	y_a_past(0,num_samples_x-1) = y_a(0);
	y_a_past(1,num_samples_x-1) = y_a(1);
	y_a_past(2,num_samples_x-1) = y_a(2);
	a_bias_past(0,num_samples_x-1) = a_bias(0);
	a_bias_past(1,num_samples_x-1) = a_bias(1);
	a_bias_past(2,num_samples_x-1) = a_bias(2);

  // Calculate Q
  if(imu_data_count%100 == 0)
    generateQ();

  Rot = q.toRotationMatrix();

  // Set F matrix (x(t+1) = f(x) or Fx)
  F << 1, 0, 0, dt, 0, 0, dt*dt/2, 0, 0, 0, 0, 0,
       0, 1, 0, 0, dt, 0, 0, dt*dt/2, 0, 0, 0, 0,
       0, 0, 1, 0, 0, dt, 0, 0, dt*dt/2, 0, 0, 0,
       0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 0, -Rot(0,0), -Rot(0,1), -Rot(0,2),
       0, 0, 0, 0, 0, 0, 0, 0, 0, -Rot(1,0), -Rot(1,1), -Rot(1,2),
       0, 0, 0, 0, 0, 0, 0, 0, 0, -Rot(2,0), -Rot(2,1), -Rot(2,2),
       0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

  // Set G matrix (error = G*sigma)
  G << 0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0, //----
       0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0, //----
       Rot(0,0), Rot(0,1), Rot(0,2), 0, 0, 0,
       Rot(1,0), Rot(1,1), Rot(1,2), 0, 0, 0,
       Rot(2,0), Rot(2,1), Rot(2,2), 0, 0, 0,
       0, 0, 0, 1, 0, 0,
       0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 0, 1;

  // Project estimate of state covariance
	P = F*P*(F.transpose()) + G*Q*(G.transpose());
  //P = P + Q;

  // Set x
  x << p, v, a, a_bias;

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

  // Add new data into z matrix (& remove oldest entry)
  removeColumn(z, 0);
  z.conservativeResize(z_size, num_samples_z);
  z(0,num_samples_z-1) = p_z(0);
  z(1,num_samples_z-1) = p_z(1);
  z(2,num_samples_z-1) = p_z(2);
  z(3,num_samples_z-1) = v_z(0);
  z(4,num_samples_z-1) = v_z(1);
  z(5,num_samples_z-1) = v_z(2);

  // Get new R estimate each second
  if(a.norm() < 0.1)
    generateCovariance(R, z, z_means, num_samples_z, z_size);

  ROS_INFO_STREAM(std::endl << "Q: " << std::endl << Q);
  ROS_INFO_STREAM(std::endl << "R: " << std::endl << R);
  // Set H matrix (y = h(x) or Hx)
  H << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;

  // Calculate Kalman gain
  Eigen::MatrixXd S = H*P*(H.transpose()) + R;
  K = P*(H.transpose())*(S.inverse());

  //K = P*(P + R).inverse();
  ROS_INFO_STREAM(std::endl << "K: " << std::endl << K);

  // Update state covariance
  P = P - K*S*(K.transpose());

  // Innovation
  x = x + K*(z.col(num_samples_z-1)-(H*x));
  p << x(0), x(1), x(2);
  v << x(3), x(4), x(5);
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
	ros::init(argc, argv, "estimator");
	ros::NodeHandle nh;
  // ROS publishers
	gyro_pub = nh.advertise<geometry_msgs::Vector3>("state/output/gyro", 1000);
	accel_pub = nh.advertise<geometry_msgs::PointStamped>("state/output/acceleration", 1000);
	vel_pub = nh.advertise<geometry_msgs::PointStamped>("state/output/velocity", 1000);
	pos_pub = nh.advertise<geometry_msgs::PoseStamped>("state/output/pose", 1000);
	q_pub = nh.advertise<geometry_msgs::Quaternion>("state/output/q", 1000);
  path_pub = nh.advertise<nav_msgs::Path>("state/output/path", 1000);
  marker_path_pub = nh.advertise<visualization_msgs::MarkerArray>("state/output/marker_path", 1000);
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
