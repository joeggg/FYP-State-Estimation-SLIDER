#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PointStamped.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#define _USE_MATH_DEFINES
#include <cmath>

const double pi = M_PI;

Eigen::Quaterniond q_W;
Eigen::Vector3d a_W, v_W, p_W;

ros::Publisher marker_pub;
visualization_msgs::Marker marker;

// Update orientation estimate
void q_callback(const geometry_msgs::Quaternion::ConstPtr& data) {
  q_W.x() = data->x;
  q_W.y() = data->y;
  q_W.z() = data->z;
  q_W.w() = data->w;

  // Broadcast transform
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q(q_W.x(), q_W.y(), q_W.z(), q_W.w());
  transform.setOrigin(tf::Vector3(p_W(0), p_W(1), p_W(2)));
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));

  marker.pose.orientation.x = q_W.x();
  marker.pose.orientation.y = q_W.y();
  marker.pose.orientation.z = q_W.z();
  marker.pose.orientation.w = q_W.w();
  marker_pub.publish(marker);
}

// Update joint estimate
void e_callback(const sensor_msgs::JointState& data) {
  geometry_msgs::Quaternion alphaL, alphaR, betaL, betaR;
  alphaR = tf::createQuaternionMsgFromRollPitchYaw(data.position[0], data.position[2], 0);
  alphaL = tf::createQuaternionMsgFromRollPitchYaw(data.position[1], data.position[3], 0);
  betaR = tf::createQuaternionMsgFromRollPitchYaw(data.position[4], data.position[6], 0);
  betaL = tf::createQuaternionMsgFromRollPitchYaw(data.position[5], data.position[7], 0);
  // alphaR = Eigen::AngleAxisd(data.position[0], Eigen::Vector3d::UnitX());
  // alphaL = Eigen::AngleAxisd(data.position[1], Eigen::Vector3d::UnitX());
  // betaR = Eigen::AngleAxisd(data.position[2], Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(data.position[4], Eigen::Vector3d::UnitY());
  // betaL = Eigen::AngleAxisd(data.position[3], Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(data.position[5], Eigen::Vector3d::UnitY());
  double roll = asin((sin(90 + data.position[0]) - sin(90 - data.position[1]))/0.60);
  double pitch = -(data.position[7] - data.position[3]);

  ROS_INFO_STREAM("roll: " << roll);
  ROS_INFO_STREAM("pitch: " << pitch);
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  // Right foot frame
  transform.setOrigin(tf::Vector3(0, 0.50, 0));
  transform.setRotation(tf::Quaternion(0, 0, 0, 1));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "rfoot_link"));
  // Left foot frame
  transform.setOrigin(tf::Vector3(0, 1.10, 0));
  transform.setRotation(tf::Quaternion(0, 0, 0, 1));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "lfoot_link"));
  // Right hip frame
  transform.setOrigin(tf::Vector3(cos(pi/2 - data.position[2]), 0.50, sin(pi/2 - data.position[2])));
  transform.setRotation(tf::Quaternion(alphaR.x, alphaR.y, alphaR.z, alphaR.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "rhip_link"));
  // Left hip frame
  transform.setOrigin(tf::Vector3(cos(pi/2 - data.position[3]), 1.10, sin(pi/2 - data.position[3])));
  transform.setRotation(tf::Quaternion(alphaL.x, alphaL.y, alphaL.z, alphaL.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "lhip_link"));

  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, 0);
  // Body frame
  transform.setOrigin(tf::Vector3(cos(pi/2 - data.position[2]), 0.80, sin(pi/2 - data.position[2])));
  transform.setRotation(tf::Quaternion(q.x, q.y, q.z, q.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));

}

// Update position esitmate
void p_callback(const geometry_msgs::PointStamped::ConstPtr& data) {
  p_W(0) = data->point.x;
  p_W(1) = data->point.y;
  p_W(2) = data->point.z;

  // Broadcast transform
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q(q_W.x(), q_W.y(), q_W.z(), q_W.w());
  transform.setOrigin(tf::Vector3(p_W(0), p_W(1), p_W(2)));
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));

  marker.pose.position.x = p_W(0);
  marker.pose.position.y = p_W(1);
  marker.pose.position.z = p_W(2) - 1.50;
  marker_pub.publish(marker);
}

// Main
int main(int argc, char** argv) {
  ros::init(argc, argv, "state_visualisation");
  ros::NodeHandle nh;
  ros::Rate r(1);
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
  // Initialise quaternion and acceleration
  q_W.x() = 0.0;
  q_W.y() = 0.0;
  q_W.z() = 0.0;
  q_W.w() = 1.0;
  a_W << 0, 0, 0;
  v_W << 0, 0, 0;
  p_W << 0, 0, 1.50;
  ros::Subscriber base_pos_sub = nh.subscribe("pos_out", 1000, p_callback);
  ros::Subscriber base_q_sub = nh.subscribe("q_out", 1000, q_callback);
  ros::Subscriber enc_sub = nh.subscribe("enc_out", 1000, e_callback);

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q(q_W.x(), q_W.y(), q_W.z(), q_W.w());
  transform.setOrigin(tf::Vector3(p_W(0), p_W(1), p_W(2)));
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));

  transform.setOrigin(tf::Vector3(0, 0.50, 0));
  transform.setRotation(tf::Quaternion(0, 0, 0, 1));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "rfoot_link"));

  // static tf::TransformBroadcaster br2;
  // transform.setOrigin(tf::Vector3(0, 0.30, 0));
  // transform.setRotation(tf::Quaternion(0, 0, 0, 0));
  // br2.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "lfoot_link"));

  uint32_t shape = visualization_msgs::Marker::CUBE;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "base_state";
    marker.id = 0;
    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;


    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    // Publish the marker
    marker_pub.publish(marker);

  ros::spin();
  return 0;
}
