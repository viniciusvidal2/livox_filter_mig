#include <ros/ros.h>
#include <vector>
#include <string>
#include <stdio.h>
#include <math.h>
#include <unordered_map>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/common.h>

// Type defs for point types PCL
typedef pcl::PointXYZI PointIn;


class CloudFilter {
public:
  CloudFilter(ros::NodeHandle &nh, std::unordered_map<std::string, float> &params, std::unordered_map<std::string, bool> &flags) 
  {
    apply_filter_ = flags["apply_filter"];

    if (flags["apply_filter"])
    {
      // Subscribe to the input point cloud
      cloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 1, &CloudFilter::cloudCallback, this);
      // Advertise the filtered point cloud
      cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar_filtered", 1);

      // Set the filter parameters
      min_intensity_ = params["min_intensity"];
      max_xy_range_ = params["max_xy_range"];
      const float boat_length = params["boat_length"];
      const float boat_width = params["boat_width"];
      const float max_height = params["max_height"];
      negative_range_.x = -boat_length / 2.0;
      negative_range_.y = -boat_width / 2.0;
      negative_range_.z = -1.0;
      positive_range_.x = boat_length / 2.0;
      positive_range_.y = boat_width / 2.0;
      positive_range_.z = max_height;

      // Set the filter flags
      filter_range_ = flags["filter_range"];
      filter_boat_points_ = flags["filter_boat_points"];
      filter_intensity_ = flags["filter_intensity"];
    }
  }

  ~CloudFilter() {}

  // Cloud callback
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
  {
    pcl::PointCloud<PointIn>::Ptr cloud_in (new pcl::PointCloud<PointIn>());
    pcl::fromROSMsg(*cloud_msg, *cloud_in);
    
    // Output message
    pcl::PointCloud<PointIn>::Ptr cloud_out (new pcl::PointCloud<PointIn>());
    cloud_out->points.reserve(cloud_in->points.size());
    cloud_out->header.frame_id = "livox_frame";

    for (const auto& p : cloud_in->points) 
    {
      // Filter boat points
      if (filter_boat_points_) 
      {
        if (filterBoatPoints(p)) 
        {
          continue;
        }
      }

      // Filter by range
      if (filter_range_) 
      {
        if (filterRange(p)) 
        {
          continue;
        }
      }

      // Filter by intensity
      if (filter_intensity_) 
      {
        if (filterIntensity(p)) 
        {
          continue;
        }
      }

      // Add the point to the output cloud
      cloud_out->points.emplace_back(p);
    }

    // Publish the result
    if (!cloud_out->points.empty()) 
    {
      sensor_msgs::PointCloud2 out_msg;
      out_msg.header = cloud_msg->header;
      pcl::toROSMsg(*cloud_out, out_msg);
      cloud_pub_.publish(out_msg);
    }
  }

private:
  /// @brief Filter boat points
  /// @param p Point to be filtered
  /// @return True if the point should be filtered out, false otherwise
  const bool filterBoatPoints(const PointIn& p) 
  {
    // If way to low (under water) or too high (above boat), filter out
    if (p.x < negative_range_.z || p.x > positive_range_.z) 
    {
      return true;
    }
    // If inside a 2D box that surrounds the boat in XY plane, filter out as well
    if (p.x > negative_range_.x && p.x < positive_range_.x && p.y > negative_range_.y && p.y < positive_range_.y) 
    {
      return true;
    }
    return false;
  }

  /// @brief Filter by range
  /// @param p Point to be filtered
  /// @return True if the point should be filtered out, false otherwise
  const bool filterRange(const PointIn& p) 
  {
    return std::sqrt(p.x * p.x + p.y * p.y) > max_xy_range_;
  }

  /// @brief Filter by intensity
  /// @param p Point to be filtered
  /// @return True if the point should be filtered out, false otherwise
  const bool filterIntensity(const PointIn& p) 
  {
    return p.intensity < min_intensity_;
  }

  // Filtered point cloud publisher
  ros::Publisher cloud_pub_;
  // Raw point cloud subscriber
  ros::Subscriber cloud_sub_;
  // Filter parameters
  pcl::PointXYZ negative_range_, positive_range_;
  float min_intensity_, max_xy_range_;
  // Filter flags
  bool apply_filter_, filter_range_, filter_intensity_, filter_boat_points_;
};

/// MAIN
///
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
  bool filter_range = false, filter_intensity = false, filter_boat_points = false;
  np.param("/raw_cloud_filter/filter_range", filter_range, false);
  np.param("/raw_cloud_filter/filter_intensity", filter_intensity, false);
  np.param("/raw_cloud_filter/filter_boat_points", filter_boat_points, false);
  float boat_length, boat_width, max_height, max_xy_range;
  np.param("/raw_cloud_filter/boat_length", boat_length, -1.0f);
  np.param("/raw_cloud_filter/boat_width", boat_width, 1.0f);
  np.param("/raw_cloud_filter/max_height", max_height, -1.0f);
  np.param("/raw_cloud_filter/max_xy_range", max_xy_range, 1000.0f);
  float min_intensity;
  np.param("/raw_cloud_filter/min_intensity", min_intensity, 0.0f);

  // Print parameters
  ROS_INFO("Applying filter: %s", apply_filter ? "true" : "false");
  ROS_INFO("Filtering boat points: %s", filter_boat_points ? "true" : "false");
  ROS_INFO("Filtering by range: %s", filter_range ? "true" : "false");
  ROS_INFO("Filtering by intensity: %s", filter_intensity ? "true" : "false");
  ROS_INFO("Boat length: %.2f", boat_length);
  ROS_INFO("Boat width: %.2f", boat_width);
  ROS_INFO("Max height: %.2f", max_height);
  ROS_INFO("Max XY range: %.2f", max_xy_range);
  ROS_INFO("Min intensity: %.2f", min_intensity);

  // Create a map to store the parameters
  std::unordered_map<std::string, float> params;
  params["boat_length"] = boat_length;
  params["boat_width"] = boat_width;
  params["max_height"] = max_height;
  params["max_xy_range"] = max_xy_range;
  params["min_intensity"] = min_intensity;
  std::unordered_map<std::string, bool> flags;
  flags["apply_filter"] = apply_filter;
  flags["filter_range"] = filter_range;
  flags["filter_boat_points"] = filter_boat_points;
  flags["filter_intensity"] = filter_intensity;

  // Create the cloud filter object
  CloudFilter cloud_filter(nh, params, flags);
  
  ROS_INFO("Listening to sensors data ...");
  ros::spin();

  return 0;
}
