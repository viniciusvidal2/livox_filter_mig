#include "laser_scan_filter.h"


LaserScanFilter::LaserScanFilter(ros::NodeHandle &nh, std::unordered_map<std::string, float> &params,
                        std::unordered_map<std::string, std::string> &frames) 
{
    // Subscribe to the livox scan
    livox_sub_ = nh.subscribe("/livox/scan", 1, &LaserScanFilter::livoxCallback, this);
    // Subscribe to the ping scan
    ping_sub_ = nh.subscribe("/ping360_node/sonar/scan", 1, &LaserScanFilter::pingCallback, this);
    
    // Publisher for the synchronized scan
    merged_scan_pub_ = nh.advertise<sensor_msgs::LaserScan>("/scan", 1);
    
    // Set the frames
    ping_frame_ = frames["ping_frame"];
    scan_frame_ = frames["scan_frame"];
    // Set the offsets
    scan_x_ping_ = params["scan_x_ping"];
    scan_y_ping_ = params["scan_y_ping"];
    scan_z_ping_ = params["scan_z_ping"];
    scan_roll_ping_ = params["scan_roll_ping"];
    scan_pitch_ping_ = params["scan_pitch_ping"];
    scan_yaw_ping_ = params["scan_yaw_ping"];

    // Create homogeneous matrix from the relative pose
    Eigen::Matrix3f scan_R_ping = (Eigen::AngleAxisf(scan_roll_ping_, Eigen::Vector3f::UnitX())
                        * Eigen::AngleAxisf(scan_pitch_ping_, Eigen::Vector3f::UnitY())
                        * Eigen::AngleAxisf(scan_yaw_ping_, Eigen::Vector3f::UnitZ())).toRotationMatrix();
    scan_T_ping_ << scan_R_ping(0, 0), scan_R_ping(0, 1), scan_R_ping(0, 2), scan_x_ping_,
                    scan_R_ping(1, 0), scan_R_ping(1, 1), scan_R_ping(1, 2), scan_y_ping_,
                    scan_R_ping(2, 0), scan_R_ping(2, 1), scan_R_ping(2, 2), scan_z_ping_,
                    0.0, 0.0, 0.0, 1.0;

    // Debug publishers
    debug_pub_size_readings_ = nh.advertise<geometry_msgs::Point32>("/debug/readings_size", 1);
    debug_pub_livox_on_ = nh.advertise<std_msgs::Bool>("/debug/livox_on", 1);
    debug_pub_ping_on_ = nh.advertise<std_msgs::Bool>("/debug/ping_on", 1);
    debug_pub_closest_reading_ = nh.advertise<geometry_msgs::Point32>("/debug/closest_reading", 1);
}

void LaserScanFilter::pingCallback(const sensor_msgs::LaserScanConstPtr &ping_msg)
{
    // Add the ping to the queue
    if (ping_queue_.size() < ping_queue_size_)
    {
        ping_queue_.emplace_back(*ping_msg);
    }
    else
    {
        ping_queue_.erase(ping_queue_.begin());
        ping_queue_.emplace_back(*ping_msg);
    }

    // Publish the debug message to show that ping is on
    std_msgs::Bool ping_on;
    ping_on.data = true;
    debug_pub_ping_on_.publish(ping_on);
}

void LaserScanFilter::livoxCallback(const sensor_msgs::LaserScanConstPtr &livox_msg)
{
    // Output message with the input livox message
    sensor_msgs::LaserScan output_msg = *livox_msg;

    // Check in the ping queue for any available ping in close range to timestamp
    for (const auto &ping_msg : ping_queue_)
    {
        // Check if the ping is in the range of the livox scan
        if (std::abs(ping_msg.header.stamp.toSec() - livox_msg->header.stamp.toSec()) < timestamp_diff_)
        {
            // Transform the ping scan to the scan frame
            sensor_msgs::LaserScan transformed_ping = transformScan(ping_msg);
            // Add the ranges and intensities of the ping to the livox scan
            mergeScans(transformed_ping, *livox_msg, output_msg);
        }
        // Get the size of the readings to publish for debug
        geometry_msgs::Point32 readings_size;
        readings_size.x = ping_msg.ranges.size();
        readings_size.y = livox_msg->ranges.size();
        debug_pub_size_readings_.publish(readings_size);
    }

    // Publish the debug message to show that livox is on
    std_msgs::Bool livox_on;
    livox_on.data = true;
    debug_pub_livox_on_.publish(livox_on);
    // Publish the closest reading debug message
    publishClosestReading(output_msg);

    // Publish the merged scan
    merged_scan_pub_.publish(output_msg);
}

sensor_msgs::LaserScan LaserScanFilter::transformScan(const sensor_msgs::LaserScan& scan)
{
    // If there is no transform, just return the input scan
    if (scan_T_ping_ == Eigen::Matrix4f::Identity())
    {
        return scan;
    }

    sensor_msgs::LaserScan transformed_scan = scan;
    // For each range value, apply the offset in 2D - XY
    for (int i = 0; i < static_cast<int>(scan.ranges.size()); ++i)
    {
        // Calculate the angle of the range
        const float angle = scan.angle_min + i*scan.angle_increment;
        // Apply the offset in 2D
        const float x = scan.ranges[i]*std::cos(angle);
        const float y = scan.ranges[i]*std::sin(angle);
        // Apply the transformation
        const Eigen::Vector4f transformed_point = scan_T_ping_*Eigen::Vector4f(x, y, 0.0, 1.0);
        // Update the range value
        transformed_scan.ranges[i] = std::sqrt(transformed_point.x()*transformed_point.x() + transformed_point.y()*transformed_point.y());
    }

    return transformed_scan;
}

void LaserScanFilter::mergeScans(const sensor_msgs::LaserScan& ping_msg, const sensor_msgs::LaserScan& livox_msg, sensor_msgs::LaserScan& output_msg)
{
    // If not the same size, just return the livox message
    if (ping_msg.ranges.size() != livox_msg.ranges.size())
    {
        output_msg = livox_msg;
        return;
    }
    
    // Assuming that both messages have the same number of ranges, merge the scans by always getting the closest non zero reading
    for (int i = 0; i < static_cast<int>(ping_msg.ranges.size()); ++i)
    {
        // Check if one the the values is either inf or nan
        if (std::isinf(ping_msg.ranges[i]) || std::isnan(ping_msg.ranges[i]))
        {
            output_msg.ranges[i] = (std::isinf(livox_msg.ranges[i]) || std::isnan(livox_msg.ranges[i])) ? 0.0 : livox_msg.ranges[i];
        }
        else if (std::isinf(livox_msg.ranges[i]) || std::isnan(livox_msg.ranges[i]))
        {
            output_msg.ranges[i] = ping_msg.ranges[i];
        }
        // If non zero values, compare, and get the closest
        std::vector<float> ranges;
        if (livox_msg.ranges[i] != 0.0)
        {
            ranges.push_back(livox_msg.ranges[i]);
        }
        if (ping_msg.ranges[i] != 0.0)
        {
            ranges.push_back(ping_msg.ranges[i]);
        }
        output_msg.ranges[i] = ranges.size() > 0 ? *std::min_element(ranges.begin(), ranges.end()) : 0.0f;
    }
}

void LaserScanFilter::publishClosestReading(const sensor_msgs::LaserScan& scan)
{
    // Find the closest reading with respective angle
    float closest_reading = std::numeric_limits<float>::max();
    float closest_angle = 0.0; // [rad]
    for (std::size_t i = 0; i < scan.ranges.size(); ++i)
    {
        if (scan.ranges[i] < closest_reading)
        {
            closest_reading = scan.ranges[i];
            closest_angle = scan.angle_min + static_cast<float>(i)*scan.angle_increment;
        }
    }
    // Publish the closest reading
    geometry_msgs::Point32 closest_reading_msg;
    closest_reading_msg.x = closest_reading;
    closest_reading_msg.y = closest_angle*180.0/M_PI; // [deg]
    debug_pub_closest_reading_.publish(closest_reading_msg);
}
