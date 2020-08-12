#include "ros/ros.h"
#include "gazebo_msgs/LinkStates.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/Float64.h"

// gt = ground truth, os = orb slam
ros::Publisher gt_pose_pub,
               gt_velocity_pub,
               gt_path_pub,
               os_pose_pub,
               os_path_pub,
               os_marker_path_pub,
               mse_pub,
               test_mse_pub,
               interp_mse_pub,
               os_mse_pub,
               e_pub,
               test_e_pub,
               interp_e_pub,
               os_e_pub;

geometry_msgs::PoseStamped gt_pose, os_pose, out_pose, test_pose, interp_pose;
geometry_msgs::Vector3Stamped gt_velocity;
nav_msgs::Path gt_path, os_path;
visualization_msgs::MarkerArray os_marker_path;
std_msgs::Float64 mse_msg, test_mse_msg, interp_mse_msg, os_mse_msg;
std_msgs::Float64 e_msg, test_e_msg, interp_e_msg, os_e_msg;


int gt_count = 0;
int os_count = 0;
const double height = 0.8475;
double total = 0;
double os_total = 0;
double test_total = 0;
double interp_total = 0;
double mse = 0;
double os_mse = 0;
double test_mse = 0;
double interp_mse = 0;
double samples = 0;

void gt_callback(const gazebo_msgs::LinkStates::ConstPtr& data) {
  gt_pose.header.seq = gt_count;
  gt_pose.header.frame_id = "map";
  gt_pose.pose.position = data->pose[1].position;
  gt_pose.pose.orientation = data->pose[1].orientation;
  gt_pose_pub.publish(gt_pose);

  gt_velocity.header.seq = gt_count;
  gt_velocity.header.frame_id = "map";
  gt_velocity.vector = data->twist[1].linear;
  gt_velocity_pub.publish(gt_velocity);

  gt_path.poses.push_back(gt_pose);
  if(gt_count%100 == 0)
    gt_path_pub.publish(gt_path);

  double out_dist, os_dist, test_dist, interp_dist;
  if(out_pose.pose.position.z != 0) {
    samples++;
    out_dist = pow(out_pose.pose.position.x-gt_pose.pose.position.x, 2) +
                pow(out_pose.pose.position.y-gt_pose.pose.position.y, 2) +
                pow(out_pose.pose.position.z-gt_pose.pose.position.z, 2);
    total += out_dist;
    mse = total/samples;
    mse_msg.data = mse;
    e_msg.data = out_dist;

    os_dist = pow(os_pose.pose.position.x-gt_pose.pose.position.x, 2) +
                   pow(os_pose.pose.position.y-gt_pose.pose.position.y, 2) +
                   pow(os_pose.pose.position.z-gt_pose.pose.position.z, 2);
    os_total += os_dist;
    os_mse = os_total/samples;
    os_mse_msg.data = os_mse;
    os_e_msg.data = os_dist;

    test_dist = pow(test_pose.pose.position.x-gt_pose.pose.position.x, 2) +
                   pow(test_pose.pose.position.y-gt_pose.pose.position.y, 2) +
                   pow(test_pose.pose.position.z-gt_pose.pose.position.z, 2);
    test_total += test_dist;
    test_mse = test_total/samples;
    test_mse_msg.data = test_mse;
    test_e_msg.data = test_dist;

    interp_dist = pow(interp_pose.pose.position.x-gt_pose.pose.position.x, 2) +
                pow(interp_pose.pose.position.y-gt_pose.pose.position.y, 2) +
                pow(interp_pose.pose.position.z-gt_pose.pose.position.z, 2);
    interp_total += interp_dist;
    interp_mse = interp_total/samples;
    interp_mse_msg.data = interp_mse;
    interp_e_msg.data = interp_dist;

    mse_pub.publish(mse_msg);
    test_mse_pub.publish(test_mse_msg);
    interp_mse_pub.publish(interp_mse_msg);
    os_mse_pub.publish(os_mse_msg);
    e_pub.publish(e_msg);
    test_e_pub.publish(test_e_msg);
    interp_e_pub.publish(interp_e_msg);
    os_e_pub.publish(os_e_msg);
  }
  ROS_INFO_STREAM("Output position MSE: " << mse << " total: " << total << std::endl << "current error: " << out_dist << std::endl);
  ROS_INFO_STREAM("ORB SLAM position MSE: " << os_mse << " total: " << os_total << std::endl << "current error: " << os_dist << std::endl);
  ROS_INFO_STREAM("Test position MSE: " << test_mse << " total: " << test_total << std::endl << "current error: " << test_dist << std::endl);
  ROS_INFO_STREAM("Interpolator position MSE: " << interp_mse << " total: " << interp_total << std::endl << "current error: " << interp_dist << std::endl);

  if(samples == 3000) {
    mse = 0; total = 0;
    os_mse = 0; os_total = 0;
    test_mse = 0; test_total = 0;
  }
  gt_count++;
}

void os_callback(const geometry_msgs::PoseStamped::ConstPtr& data) {
  os_pose.header.seq = os_count;
  os_pose.header.frame_id = "map";
  os_pose.pose.position = data->pose.position;
  os_pose.pose.position.z = data->pose.position.z + height;
  os_pose.pose.orientation = data->pose.orientation;
  os_pose_pub.publish(os_pose);

  // Marker path
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.header.seq = os_count;
  marker.ns = "output";
  marker.id = os_count;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = os_pose.pose;
  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 0.01;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.6784;
  marker.color.g = 0.4980;
  marker.color.b = 0.6588;
  os_marker_path.markers.push_back(marker);
  os_marker_path_pub.publish(os_marker_path);

  os_path.poses.push_back(os_pose);
  os_path_pub.publish(os_path);
  os_count++;
}

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& data) {
  out_pose.pose.position.x = data->pose.position.x;
  out_pose.pose.position.y = data->pose.position.y;
  out_pose.pose.position.z = data->pose.position.z;
}

void test_callback(const geometry_msgs::PoseStamped::ConstPtr& data) {
  test_pose.pose.position.x = data->pose.position.x;
  test_pose.pose.position.y = data->pose.position.y;
  test_pose.pose.position.z = data->pose.position.z;
}

void interp_callback(const geometry_msgs::PoseStamped::ConstPtr& data) {
  interp_pose.pose.position.x = data->pose.position.x;
  interp_pose.pose.position.y = data->pose.position.y;
  interp_pose.pose.position.z = data->pose.position.z;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ground_truth_receiver");
	ros::NodeHandle nh;

  ros::Subscriber ground_truth_sub = nh.subscribe("gazebo/link_states", 1000, gt_callback);
  ros::Subscriber orb_slam_sub = nh.subscribe("orb_slam2_rgbd/pose", 1000, os_callback);
  ros::Subscriber output_sub = nh.subscribe("state/output/pose", 1000, pose_callback);
  ros::Subscriber test_sub = nh.subscribe("state/test/pose", 1000, test_callback);
  ros::Subscriber interp_sub = nh.subscribe("state/interpolator/pose", 1000, interp_callback);

  gt_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("state/ground_truth/pose", 1000);
  gt_velocity_pub = nh.advertise<geometry_msgs::Vector3Stamped>("state/ground_truth/velocity", 1000);
  gt_path_pub = nh.advertise<nav_msgs::Path>("state/ground_truth/path", 1000);

  os_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("state/orb_slam/pose", 1000);
  os_path_pub = nh.advertise<nav_msgs::Path>("state/orb_slam/path", 1000);
  os_marker_path_pub = nh.advertise<visualization_msgs::MarkerArray>("state/orb_slam/marker_path", 1000);

  mse_pub = nh.advertise<std_msgs::Float64>("state/output/mse", 1000);
  test_mse_pub = nh.advertise<std_msgs::Float64>("state/test/mse", 1000);
  interp_mse_pub = nh.advertise<std_msgs::Float64>("state/interpolator/mse", 1000);
  os_mse_pub = nh.advertise<std_msgs::Float64>("state/orb_slam/mse", 1000);

  e_pub = nh.advertise<std_msgs::Float64>("state/output/error", 1000);
  test_e_pub = nh.advertise<std_msgs::Float64>("state/test/error", 1000);
  interp_e_pub = nh.advertise<std_msgs::Float64>("state/interpolator/error", 1000);
  os_e_pub = nh.advertise<std_msgs::Float64>("state/orb_slam/error", 1000);

  gt_path.header.frame_id = "map";
  os_path.header.frame_id = "map";

  ros::spin();
  return 0;
}
