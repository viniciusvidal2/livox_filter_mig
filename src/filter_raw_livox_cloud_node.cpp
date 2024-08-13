#include <ros/ros.h>
#include <vector>
#include <string>
#include <stdio.h>
#include <math.h>
#include <unordered_map>

#include "cloud_filter.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "filter_raw_cloud_node");
  ros::NodeHandle nh;
  ros::NodeHandle np("~");
  ROS_INFO("Initialyzing raw point cloud filter node ...");

  // Reading parameters
  bool apply_filter;
  np.param("/raw_cloud_filter/apply_filter", apply_filter, false);
  bool filter_range, filter_intensity, filter_boat_points, debug_cloud;
  np.param("/raw_cloud_filter/filter_range", filter_range, false);
  np.param("/raw_cloud_filter/filter_intensity", filter_intensity, false);
  np.param("/raw_cloud_filter/filter_boat_points", filter_boat_points, false);
  np.param("/raw_cloud_filter/debug_cloud", debug_cloud, false);
  float boat_length, boat_width, max_height, max_xy_range;
  np.param("/raw_cloud_filter/boat_length", boat_length, -1.0f);
  np.param("/raw_cloud_filter/boat_width", boat_width, 1.0f);
  np.param("/raw_cloud_filter/max_height", max_height, -1.0f);
  np.param("/raw_cloud_filter/max_xy_range", max_xy_range, 1000.0f);
  float min_intensity;
  np.param("/raw_cloud_filter/min_intensity", min_intensity, 0.0f);
  float angle_resolution; // [deg]
  np.param("/pointcloud2scan/angle_resolution", angle_resolution, 1.0f);
  float x_in_out, y_in_out, z_in_out, roll_in_out, pitch_in_out, yaw_in_out;
  std::string in_frame, out_frame;
  np.param("/livox2scan/input_frame", in_frame, static_cast<std::string>("body_frame"));
  np.param("/livox2scan/output_frame", out_frame, static_cast<std::string>("base_link"));
  np.param("/livox2scan/x", x_in_out, 0.0f);
  np.param("/livox2scan/y", y_in_out, 0.0f);
  np.param("/livox2scan/z", z_in_out, 0.0f);
  np.param("/livox2scan/roll", roll_in_out, 0.0f);
  np.param("/livox2scan/pitch", pitch_in_out, 0.0f);
  np.param("/livox2scan/yaw", yaw_in_out, 0.0f);

  // Print parameters
  ROS_INFO("Applying filter: %s", apply_filter ? "true" : "false");
  ROS_INFO("Filtering boat points: %s", filter_boat_points ? "true" : "false");
  ROS_INFO("Filtering by range: %s", filter_range ? "true" : "false");
  ROS_INFO("Filtering by intensity: %s", filter_intensity ? "true" : "false");
  ROS_INFO("Debug cloud: %s", debug_cloud ? "true" : "false");
  ROS_INFO("Boat length: %.2f meters", boat_length);
  ROS_INFO("Boat width: %.2f meters", boat_width);
  ROS_INFO("Max height: %.2f meters", max_height);
  ROS_INFO("Max XY range: %.2f meters", max_xy_range);
  ROS_INFO("Min intensity: %.2f units", min_intensity);
  ROS_INFO("Angle resolution: %.2f degrees", angle_resolution);
  ROS_INFO("Relative pose: %s -> %s", in_frame.c_str(), out_frame.c_str());
  ROS_INFO("x: %.2f, y: %.2f, z: %.2f, roll: %.2f, pitch: %.2f, yaw: %.2f", x_in_out, y_in_out, z_in_out, roll_in_out, pitch_in_out, yaw_in_out);

  // Create a map to store the parameters
  std::unordered_map<std::string, float> params;
  params["boat_length"] = boat_length;
  params["boat_width"] = boat_width;
  params["max_height"] = max_height;
  params["max_xy_range"] = max_xy_range;
  params["min_intensity"] = min_intensity;
  params["angle_resolution"] = angle_resolution;
  params["x_in_out"] = x_in_out;
  params["y_in_out"] = y_in_out;
  params["z_in_out"] = z_in_out;
  params["roll_in_out"] = roll_in_out;
  params["pitch_in_out"] = pitch_in_out;
  params["yaw_in_out"] = yaw_in_out;
  std::unordered_map<std::string, bool> flags;
  flags["apply_filter"] = apply_filter;
  flags["filter_range"] = filter_range;
  flags["filter_boat_points"] = filter_boat_points;
  flags["filter_intensity"] = filter_intensity;
  flags["debug_cloud"] = debug_cloud;
  std::unordered_map<std::string, std::string> frames;
  frames["in_frame"] = in_frame;
  frames["out_frame"] = out_frame;

  // Create the cloud filter object
  CloudFilter cloud_filter(nh, params, flags, frames);
  
  ROS_INFO("Listening to sensors data ...");
  ros::spin();

  return 0;
}
