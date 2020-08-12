#include <iostream>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>

#define _USE_MATH_DEFINES
#include <cmath>

const double pi = M_PI;
const double sigma = 0.068;

enum support_mode {
	none,
	single_sup_L,
	single_sup_R,
	double_sup
};

support_mode mode = single_sup_L;
double timestamp = 0;

ros::Publisher dcm_pub, com_pub, p_pub, q_pub;

// State variables
tf::Quaternion alphaL, alphaR, betaL, betaR;
Eigen::Vector3d p, rfoot_p, lfoot_p, rhip_p, lhip_p;
Eigen::Quaterniond q, rhip_q, lhip_q, rfoot_q, lfoot_q;
Eigen::Vector3d com_v;
Eigen::Vector3d rleg_com, lleg_com, com, com_old;

// Constants
double w = 0.60;
double l = 1.00;
Eigen::Vector3d g(0, 0, 9.81);

// Encoder data processing
void enc_callback(const sensor_msgs::JointState& data) {
	double dt = ros::Time::now().toSec() - timestamp;
	switch(mode) {
		// case double_sup: {
		// 	// Create quaternions for ankle and hip orientations
		// 	alphaR.setEuler(data.position[2], data.position[0], 0);
		// 	alphaL.setEuler(data.position[3], data.position[1], 0);
		// 	betaR.setEuler(data.position[6], data.position[4], 0);
		// 	betaL.setEuler(data.position[7], data.position[5], 0);
		// 	// Find body orientation
		//   double roll = asin((sin(90 + data.position[0]) - sin(90 - data.position[1]))/0.60);
		//   double pitch = -(data.position[7] - data.position[3]);
		// 	double yaw = 0;
		// 	q.setEuler(pitch, roll, yaw);
		//
		// 	rfoot_p = {0, 0.50, 0};
		// 	lfoot_p = {0, 1.10, 0};
		// 	rhip_p = {cos(pi/2 - data.position[2]), 0.50, sin(pi/2 - data.position[2])};
		// 	lhip_p = {cos(pi/2 - data.position[3]), 1.10, sin(pi/2 - data.position[3])};
		// 	rfoot_q = {0, 0, 0, 1};
		// 	lfoot_q = {0, 0, 0, 1};
		// 	p = {cos(pi/2 - data.position[2]), 0.80, sin(pi/2 - data.position[2])};
		//
		// 	break;
		// }
		case single_sup_R: {
			// Displacement temporary variable
			Eigen::Vector3d dv;
			// True leg lengths
			double lr_eff = l - data.position[8];
			double ll_eff = l - data.position[9];

			// Ensure right foot on ground
			rfoot_p(2) = 0.0;
			rfoot_q.setIdentity();

			// Right hip position and orientation
			rhip_p << rfoot_p(0)+lr_eff*sin(data.position[2]), rfoot_p(1)+lr_eff*sin(data.position[0]), lr_eff*cos(data.position[0])*cos(data.position[2]);
			rhip_q = Eigen::AngleAxisd(-data.position[0], Eigen::Vector3d::UnitX())*
								Eigen::AngleAxisd(data.position[2], Eigen::Vector3d::UnitY())*
									Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());

			// Right leg centre of mass
			rleg_com = (rhip_p + rfoot_p)/2;

			// Left hip position and orientation
			dv << 0, w*cos(data.position[4]), w*sin(data.position[4]);
			lhip_p = rhip_p + rhip_q*dv;
			lhip_q = rhip_q*Eigen::AngleAxisd(data.position[4], Eigen::Vector3d::UnitX())*
											Eigen::AngleAxisd(data.position[6], Eigen::Vector3d::UnitY())*
											Eigen::AngleAxisd(data.position[5], Eigen::Vector3d::UnitX())*
											Eigen::AngleAxisd(data.position[7], Eigen::Vector3d::UnitY());

			// Body position and orientation
			dv << 0, w/2*cos(data.position[4]), w/2*sin(data.position[4]);
			p = rhip_p + rhip_q*dv;
			q = rhip_q*Eigen::AngleAxisd(data.position[4], Eigen::Vector3d::UnitX())*
											Eigen::AngleAxisd(data.position[6], Eigen::Vector3d::UnitY());


			// Left foot position and orientation
			dv << 0, 0, -ll_eff;
			lfoot_p = lhip_p + lhip_q*dv;
			lfoot_q = lhip_q*Eigen::AngleAxisd(data.position[1], Eigen::Vector3d::UnitX())*
											Eigen::AngleAxisd(data.position[3], Eigen::Vector3d::UnitY());

			// Left leg centre of mass
			lleg_com = (lhip_p + lfoot_p)/2;

			// Centre of mass position
			com_old = com;
			com = (p + rleg_com + lleg_com)/3;

			break;
		}
		case single_sup_L: {
			// Displacement temporary variable
			Eigen::Vector3d dv;
			// True leg lengths
			double lr_eff = l - data.position[8];
			double ll_eff = l - data.position[9];

			// Ensure left foot is on ground
			lfoot_p(2) = 0.0;
			lfoot_q.setIdentity();

			// Left hip position and orientation
			lhip_p << lfoot_p(0)+ll_eff*sin(data.position[3]), lfoot_p(1)+ll_eff*sin(data.position[1]), ll_eff*cos(data.position[1])*cos(data.position[3]);
			lhip_q = Eigen::AngleAxisd(-data.position[1], Eigen::Vector3d::UnitX())*
								Eigen::AngleAxisd(data.position[3], Eigen::Vector3d::UnitY())*
									Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());

			// Left leg centre of mass
			lleg_com = (lhip_p + lfoot_p)/2;

			// Right hip position and orientation
			dv << 0, -w*cos(data.position[5]), w*sin(data.position[5]);
			rhip_p = lhip_p + lhip_q*dv;
			rhip_q = lhip_q*Eigen::AngleAxisd(-data.position[5], Eigen::Vector3d::UnitX())*
											Eigen::AngleAxisd(data.position[7], Eigen::Vector3d::UnitY())*
											Eigen::AngleAxisd(-data.position[4], Eigen::Vector3d::UnitX())*
											Eigen::AngleAxisd(data.position[6], Eigen::Vector3d::UnitY());

			// Body position and orientation
			dv << 0, -w/2*cos(data.position[5]), w/2*sin(data.position[5]);
			p = lhip_p + lhip_q*dv;
			q = lhip_q*Eigen::AngleAxisd(-data.position[5], Eigen::Vector3d::UnitX())*
											Eigen::AngleAxisd(data.position[7], Eigen::Vector3d::UnitY());


			// Right foot position and orientation
			dv << 0, 0, -lr_eff;
			rfoot_p = rhip_p + rhip_q*dv;
			rfoot_q = rhip_q*Eigen::AngleAxisd(-data.position[0], Eigen::Vector3d::UnitX())*
											Eigen::AngleAxisd(data.position[2], Eigen::Vector3d::UnitY());

			// Right leg centre of mass
			rleg_com = (rhip_p + rfoot_p)/2;

			// Centre of mass position
			com_old = com;
			com = (p + rleg_com + lleg_com)/3;


			break;
		}
	}
	double runtime = ros::Time::now().toSec() - (dt + timestamp);
	timestamp = ros::Time::now().toSec();

	// Calculate divergent component of motion
	com_v = (dt/0.001)*((com-com_old)/dt) + (1-(dt/0.001))*com_v;
	Eigen::Vector3d dcm;
	dcm = com + (com_v/sqrt(9.81/p(2)));

	geometry_msgs::PointStamped dcm_msg;
	dcm_msg.header.frame_id = "world";
	dcm_msg.point.x = dcm(0);
	dcm_msg.point.y = dcm(1);
	dcm_msg.point.z = dcm(2);
	dcm_pub.publish(dcm_msg);

	// CoM publisher
	geometry_msgs::PointStamped com_msg;
	com_msg.header.frame_id = "world";
	com_msg.point.x = com(0);
	com_msg.point.y = com(1);
	com_msg.point.z = com(2);
	com_pub.publish(com_msg);

	// p publisher
	geometry_msgs::PointStamped p_msg;
	p_msg.header.frame_id = "world";
	p_msg.point.x = p(0);
	p_msg.point.y = p(1);
	p_msg.point.z = p(2);
	p_pub.publish(p_msg);

	// q publisher
	geometry_msgs::Quaternion q_msg;
	q_msg.x = q.x();
	q_msg.y = q.y();
	q_msg.z = q.z();
	q_msg.w = q.w();
	q_pub.publish(q_msg);

	//ROS_INFO_STREAM(dcm);
	ROS_INFO_STREAM(1000*runtime<<"ms");

	static tf::TransformBroadcaster br;
  tf::Transform transform;
	// Right foot frame
	transform.setOrigin(tf::Vector3(rfoot_p(0), rfoot_p(1), rfoot_p(2)));
	transform.setRotation(tf::Quaternion(rfoot_q.x(), rfoot_q.y(), rfoot_q.z(), rfoot_q.w()));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "rfoot_link"));
	// Left foot frame
	transform.setOrigin(tf::Vector3(lfoot_p(0), lfoot_p(1), lfoot_p(2)));
	transform.setRotation(tf::Quaternion(lfoot_q.x(), lfoot_q.y(), lfoot_q.z(), lfoot_q.w()));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "lfoot_link"));
	// Right hip frame
	transform.setOrigin(tf::Vector3(rhip_p(0), rhip_p(1), rhip_p(2)));
	transform.setRotation(tf::Quaternion(rhip_q.x(), rhip_q.y(), rhip_q.z(), rhip_q.w()));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "rhip_link"));
	// Left hip frame
	transform.setOrigin(tf::Vector3(lhip_p(0), lhip_p(1), lhip_p(2)));
	transform.setRotation(tf::Quaternion(lhip_q.x(), lhip_q.y(), lhip_q.z(), lhip_q.w()));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "lhip_link"));
	// // Body frame
	transform.setOrigin(tf::Vector3(p(0), p(1), p(2)));
	transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
}

void sup_callback(const std_msgs::Float64& data) {
	if(data.data > 0.5) {
		mode = single_sup_L;
	}
	else {
		mode = single_sup_R;
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "encoder_receiver");
	ros::NodeHandle nh;

	ros::Subscriber enc_sub = nh.subscribe("enc_out", 1000, enc_callback);
	ros::Subscriber sup_sub = nh.subscribe("sup_out", 1000, sup_callback);

	p_pub = nh.advertise<geometry_msgs::PointStamped>("p_out", 1000);
	q_pub = nh.advertise<geometry_msgs::Quaternion>("q_out", 1000);
	com_pub = nh.advertise<geometry_msgs::PointStamped>("com_out", 1000);
	dcm_pub = nh.advertise<geometry_msgs::PointStamped>("dcm_out", 1000);

	rfoot_q.setIdentity();
	rfoot_p << 0, 0.50, 0;
	lfoot_q.setIdentity();
	lfoot_p << 0, 1.00, 0;
	com_v << 0, 0, 0;

	ros::spin();

	return 0;
}
