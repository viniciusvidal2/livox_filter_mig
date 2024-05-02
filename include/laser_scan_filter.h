#include <ros/ros.h>
#include <vector>
#include <string>
#include <stdio.h>
#include <math.h>
#include <unordered_map>
#include <mutex>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Bool.h>

#ifndef LASERSCANFILTER_H
#define LASERSCANFILTER_H
class LaserScanFilter 
{
public:
    LaserScanFilter(ros::NodeHandle &nh, std::unordered_map<std::string, float> &params,
                std::unordered_map<std::string, std::string> &frames);

    ~LaserScanFilter() = default;

    // Ping callback
    void pingCallback(const sensor_msgs::LaserScanConstPtr &ping_msg);

    // Livox callback
    void livoxCallback(const sensor_msgs::LaserScanConstPtr &livox_msg);

private:
    /// @brief Merge input scans
    /// @param[in] ping_msg: ping scan message
    /// @param[in] livox_msg: livox scan message
    /// @param[in] output_msg: output message
    void mergeScans(const sensor_msgs::LaserScan& ping_msg, const sensor_msgs::LaserScan& livox_msg, sensor_msgs::LaserScan& output_msg);

    /// @brief Transform the scan using the relative transform offsets
    /// @param[in] scan_msg: input scan message
    /// @return transformed scan message
    sensor_msgs::LaserScan transformScan(const sensor_msgs::LaserScan& scan_msg);

    /// @brief Publish the closest reading to the boat plus the respective angle
    /// @param[in] scan: input scan message
    void publishClosestReading(const sensor_msgs::LaserScan& scan);

    // Filtered point cloud publisher
    ros::Publisher merged_scan_pub_;
    // Raw point cloud subscriber
    ros::Subscriber ping_sub_, livox_sub_;
    // Filter parameters
    const std::size_t ping_queue_size_ = 5;
    std::vector<sensor_msgs::LaserScan> ping_queue_;
    const double timestamp_diff_ = 0.1; // [s]
    // Frames
    std::string ping_frame_, scan_frame_;
    float scan_x_ping_, scan_y_ping_, scan_z_ping_, scan_roll_ping_, scan_pitch_ping_, scan_yaw_ping_;
    Eigen::Matrix4f scan_T_ping_;

    // Debug publishers
    ros::Publisher debug_pub_size_readings_, debug_pub_livox_on_, debug_pub_ping_on_;
    ros::Publisher debug_pub_closest_reading_;
};
#endif // LASERSCANFILTER_H
