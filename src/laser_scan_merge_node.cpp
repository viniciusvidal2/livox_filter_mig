#include <ros/ros.h>
#include <vector>
#include <string>
#include <stdio.h>
#include <math.h>
#include <unordered_map>

#include "laser_scan_filter.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_scan_filter_node");
  ros::NodeHandle nh;
  ros::NodeHandle np("~");
  ROS_INFO("Initialyzing laser scan filter node ...");

  // Reading parameters
  float scan_x_ping, scan_y_ping, scan_z_ping, scan_roll_ping, scan_pitch_ping, scan_yaw_ping;
  std::string ping_frame, scan_frame;
  np.param("/ping2scan/input_frame", ping_frame, static_cast<std::string>("ping_frame"));
  np.param("/ping2scan/output_frame", scan_frame, static_cast<std::string>("base_link"));
  np.param("/ping2scan/x", scan_x_ping, 0.0f);
  np.param("/ping2scan/y", scan_y_ping, 0.0f);
  np.param("/ping2scan/z", scan_z_ping, 0.0f);
  np.param("/ping2scan/roll", scan_roll_ping, 0.0f);
  np.param("/ping2scan/pitch", scan_pitch_ping, 0.0f);
  np.param("/ping2scan/yaw", scan_yaw_ping, 0.0f);

  // Print parameters
  ROS_INFO("Relative pose: %s -> %s", ping_frame.c_str(), scan_frame.c_str());
  ROS_INFO("x: %.2f, y: %.2f, z: %.2f, roll: %.2f, pitch: %.2f, yaw: %.2f", 
          scan_x_ping, scan_y_ping, scan_z_ping, scan_roll_ping, scan_pitch_ping, scan_yaw_ping);

  // Create a map to store the parameters
  std::unordered_map<std::string, float> params;
  params["scan_x_ping"] = scan_x_ping;
  params["scan_y_ping"] = scan_y_ping;
  params["scan_z_ping"] = scan_z_ping;
  params["scan_roll_ping"] = scan_roll_ping;
  params["scan_pitch_ping"] = scan_pitch_ping;
  params["scan_yaw_ping"] = scan_yaw_ping;
  std::unordered_map<std::string, std::string> frames;
  frames["ping_frame"] = ping_frame;
  frames["scan_frame"] = scan_frame;

  // Create the cloud filter object
  LaserScanFilter ls_filter(nh, params, frames);
  
  ROS_INFO("Listening to laser scans from livox and ping ...");
  ros::spin();

  return 0;
}
