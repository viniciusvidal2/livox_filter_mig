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
  bool apply_filter = false;
  np.param("/raw_cloud_filter/apply_filter", apply_filter, false);
  if (!apply_filter) 
  {
    ROS_WARN("Raw point cloud filter is disabled. Exiting ...");
    return 0;
  }
  bool filter_range = false, filter_intensity = false, filter_boat_points = false, debug_cloud = false;
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

  // Create a map to store the parameters
  std::unordered_map<std::string, float> params;
  params["boat_length"] = boat_length;
  params["boat_width"] = boat_width;
  params["max_height"] = max_height;
  params["max_xy_range"] = max_xy_range;
  params["min_intensity"] = min_intensity;
  params["angle_resolution"] = angle_resolution;
  std::unordered_map<std::string, bool> flags;
  flags["apply_filter"] = apply_filter;
  flags["filter_range"] = filter_range;
  flags["filter_boat_points"] = filter_boat_points;
  flags["filter_intensity"] = filter_intensity;
  flags["debug_cloud"] = debug_cloud;

  // Create the cloud filter object
  CloudFilter cloud_filter(nh, params, flags);
  
  ROS_INFO("Listening to sensors data ...");
  ros::spin();

  return 0;
}
