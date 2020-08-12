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
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include <tf/transform_broadcaster.h>
#include <iostream>
#include "eigen3/Eigen/Dense"
#define _USE_MATH_DEFINES
#include <cmath>
#include <imu_3dm_gx4/FilterOutput.h>

ros::Publisher gyro_pub, accel_pub, vel_pub, pos_pub, q_pub;

// Body state variables
Eigen::Vector3d a, v, v_meas, p, p_meas;
Eigen::Quaterniond q, q_meas;
Eigen::MatrixXd x, z;

Eigen::MatrixXd F, P, Q, H, R, K;

double p_count = 0;

// Extra variables
Eigen::VectorXd state_means, meas_means;
Eigen::Vector3d a_B;
int imu_data_count = 0;
int q_sample_count = 0;
ros::Time timestamp;

// Constants
int x_size = 7;
int num_samples = 60; // no. samples taken for covariance of x
const double pi = M_PI;
const Eigen::Vector3d g(0, 0, -9.81);
const double enocder_sigma = 0.068;

void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove)
{
    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols()-1;

    if( colToRemove < numCols )
        matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.block(0,colToRemove+1,numRows,numCols-colToRemove).eval();

    matrix.conservativeResize(numRows,numCols);
}

/* Callback for new imu data point */
void callback(const sensor_msgs::Imu::ConstPtr& data) {
	// Store body orientation quaternion
	q.x() = data->orientation.x;
	q.y() = data->orientation.y;
	q.z() = data->orientation.z;
	q.w() = data->orientation.w;

	// Store acceleration
	a_B << data->linear_acceleration.x, data->linear_acceleration.y, data->linear_acceleration.z;
	// Rotate accel from body to world frame and remove gravity
	Eigen::Quaterniond r;
	r.w() = 0;
	r.vec() = a_B;
	Eigen::Quaterniond rotatedR = q.inverse()*r*q;
	a = rotatedR.vec() - g;

	double dt = (ros::Time::now() - timestamp).toSec();
	timestamp = ros::Time::now();

	if(dt < 1) {

    // Restrict possible accelerations to remove noise before updating velocity
    if(a.norm() < 100 && a.norm() > 10) {
      v += a*dt;
    }
    else if(a.norm() > 100) {}
    else
    {
      v << 0, 0, 0;
      //p(2) = 1; // Contstrains height
    }
  	// Update global position
    p(0) += v(0)*dt;
    p(1) += v(1)*dt;
    p(2) += v(2)*dt;

		// State transition matrix
		F << 1, 0, 0, dt, 0, 0, 0,
	 		 	 0, 1, 0, 0, dt, 0, 0,
			 	 0, 0, 1, 0, 0, dt, 0,
			 	 0, 0, 0, 1, 0, 0, 0,
			 	 0, 0, 0, 0, 1, 0, 0,
			 	 0, 0, 0, 0, 0, 1, 0,
			 	 0, 0, 0, 0, 0, 0, 0;

		// Calculate means of state variables
    // state_means.setZero(x_size);
    // meas_means.setZero(x_size);
		// removeColumn(x, 0);
    // removeColumn(z, 0);
		// x.conservativeResize(x_size, num_samples);
    // z.conservativeResize(x_size, num_samples);
    //
		// x(0,num_samples-1) = p(0);
		// x(1,num_samples-1) = p(1);
		// x(2,num_samples-1) = p(2);
		// x(3,num_samples-1) = v(0);
		// x(4,num_samples-1) = v(1);
		// x(5,num_samples-1) = v(2);
		// x(6,num_samples-1) = q.w() + q.x() + q.y() + q.z();
    //
    // z(0,num_samples-1) = p_meas(0);
		// z(1,num_samples-1) = p_meas(1);
		// z(2,num_samples-1) = p_meas(2);
		// z(3,num_samples-1) = v_meas(0);
		// z(4,num_samples-1) = v_meas(1);
		// z(5,num_samples-1) = v_meas(2);
		// z(6,num_samples-1) = q_meas.w() + q_meas.x() + q_meas.y() + q_meas.z();
    //
		// for(int i = 0; i < num_samples; i++) {
		// 	state_means += x.col(i);
    //   meas_means += z.col(i);
		// }
		// state_means = state_means/num_samples;
    // meas_means = meas_means/num_samples;

		//	Calculate Q (and R?)
		// Q.setZero();
    // R.setZero();
		// for(int i = 0; i < x_size; i++) {
		// 	for(int j = 0; j < x_size; j++) {
		// 		for(int k = 0; k < num_samples; k++) {
		// 			Q(i,j) += (x(i,k)-state_means(i))*(x(j,k)-state_means(j));
    //       R(i,j) += (z(i,k)-meas_means(i))*(z(j,k)-meas_means(j));
		// 		}
		// 		Q(i,j) = Q(i,j)/num_samples;
    //     R(i,j) = R(i,j)/num_samples;
		// 	}
		// }
    //
		// ROS_INFO_STREAM("delta t: " << dt << std::endl);
	  //ROS_INFO_STREAM(std::endl << meas_means);

    // // Project estimate of state covariance
		// P = F*P*(F.transpose()) + Q;
    // // Calculate Kalman gain
    // K = P*((P + R).inverse());
    // ROS_INFO_STREAM(K);

    // Update state covariance
    //P = (Eigen::MatrixXd::Identity(x_size, x_size) - K)*P;
	}

	// Publish acceleration
	geometry_msgs::PointStamped accel;
	accel.header.seq = imu_data_count;
	accel.header.frame_id = "world";
	accel.point.x = a(0);
	accel.point.y = a(1);
	accel.point.z = a(2);
	accel_pub.publish(accel);
	// // Publish velocity
	// geometry_msgs::PointStamped vel;
	// vel.header.seq = imu_data_count;
	// vel.header.frame_id = "base_link";
	// vel.point.x = v(0);
	// vel.point.y = v(1);
	// vel.point.z = v(2);
	// vel_pub.publish(vel);
	// Publish position
	geometry_msgs::PointStamped pos;
	pos.header.seq = imu_data_count;
	pos.header.frame_id = "world";
	pos.point.x = p(0);
	pos.point.y = p(1);
	pos.point.z = p(2);
	pos_pub.publish(pos);
	// Publish orientation
	geometry_msgs::Quaternion quat;
	quat.x = q.x();
	quat.y = q.y();
	quat.z = q.z();
	quat.w = q.w();
	q_pub.publish(quat);

  static tf::TransformBroadcaster br;
  tf::Transform transform;

  // Base link frame
  transform.setOrigin(tf::Vector3(0, 0, 1.50));
  transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));

  // Camera frame setup
	transform.setOrigin(tf::Vector3(0, 0, 0.2));
	transform.setRotation(tf::Quaternion(0, 0, 0, 1));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "camera"));

	// Publish angular velocity data
	//gyro_pub.publish(data->angular_velocity);
  ROS_INFO_STREAM(p);
	imu_data_count++;
}

void p_callback(const geometry_msgs::PointStamped& data) {
  p_meas << data.point.x, data.point.y, data.point.z;
}

void q_callback(const geometry_msgs::Quaternion& data) {
  q_meas.x() = data.x;
  q_meas.y() = data.y;
  q_meas.z() = data.z;
  q_meas.w() = data.w;
}

/* Main */
int main(int argc, char **argv) {
	ros::init(argc, argv, "estimator");
	ros::NodeHandle nh;

  // ROS publishers and subscribers
	gyro_pub = nh.advertise<geometry_msgs::Vector3>("gyro_out", 1000);
	accel_pub = nh.advertise<geometry_msgs::PointStamped>("accel_out", 1000);
	vel_pub = nh.advertise<geometry_msgs::PointStamped>("vel_out", 1000);
	pos_pub = nh.advertise<geometry_msgs::PointStamped>("pos_out", 1000);
	q_pub = nh.advertise<geometry_msgs::Quaternion>("q_out", 1000);
	ros::Subscriber sub = nh.subscribe("/gx4/imu/data", 1000, callback);
  ros::Subscriber p_sub = nh.subscribe("p_out", 1000, p_callback);
  ros::Subscriber q_sub = nh.subscribe("q_out", 1000, q_callback);

	timestamp = ros::Time::now();

  // State variables initialisation
  a << 0, 0, 0;
	v << 0, 0, 0;
  v_meas << 0, 0, 0;
	p << 0, 0, 1.50;
  p_meas << 0, 0, 1.50;

	x.setZero(x_size, num_samples);
  z.setZero(x_size, num_samples);
	state_means.setZero(x_size);
  meas_means.setZero(x_size);

  // Kalman gain
  K.setZero(x_size, x_size);

  // State transition matrices
	F.setIdentity(x_size, x_size);
  H.setIdentity(x_size, x_size);

  // Estimate covariance
	P.setIdentity(x_size, x_size);
	P = 0.001*P;

  // Process/measurement covariance matrices
	Q.setZero(x_size, x_size);
  R.setZero(x_size, x_size);

	ros::spin();

	return 0;
}
