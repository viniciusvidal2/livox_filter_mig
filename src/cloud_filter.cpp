#include "cloud_filter.h"


CloudFilter::CloudFilter(ros::NodeHandle &nh, std::unordered_map<std::string, float> &params, std::unordered_map<std::string, bool> &flags) 
{
    apply_filter_ = flags["apply_filter"];
    publish_debug_cloud_ = flags["debug_cloud"];

    // Subscribe to the input point cloud
    cloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 1, &CloudFilter::cloudCallback, this);
    // Advertise the filtered point cloud
    if (publish_debug_cloud_)
    {
        debug_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar_cloud_debug", 1);
    }
    out_scan_pub_ = nh.advertise<sensor_msgs::LaserScan>("/livox/scan", 1);

    // Convertion from point cloud to scan
    angle_resolution_ = params["angle_resolution"]*M_PI/180.0f; // [rad]

    if (apply_filter_)
    {
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

// Cloud callback
void CloudFilter::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<PointIn>::Ptr cloud_in (new pcl::PointCloud<PointIn>());
    pcl::fromROSMsg(*cloud_msg, *cloud_in);

    if (cloud_in->empty()) 
    {
        return;
    }

    // Check if we are applying the filter at all
    if (!apply_filter_) 
    {
        if (publish_debug_cloud_) 
        {
            debug_cloud_pub_.publish(*cloud_msg);
        }

        return;
    }
    
    // Output debug point cloud
    pcl::PointCloud<PointIn>::Ptr cloud_out (new pcl::PointCloud<PointIn>());
    cloud_out->points.reserve(cloud_in->points.size());
    cloud_out->header.frame_id = cloud_msg->header.frame_id;
    // Output laser scan
    sensor_msgs::LaserScan scan_out;
    scan_out.header = cloud_msg->header;
    scan_out.scan_time = 0.1; // 10 Hz

    // Work on the input point cloud
    std::vector<float> ranges, intensities, angles;
    ranges.reserve(cloud_in->points.size());
    intensities.reserve(cloud_in->points.size());
    angles.reserve(cloud_in->points.size());
    for (const auto& p : cloud_in->points) 
    {
        // Filter Z values that are out of possible boat collision
        if (p.z < negative_range_.z || p.z > positive_range_.z) 
        {
            continue;
        }

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

        // Calculate the angle and range of the point
        // Z positive, X forward, Y left
        // 0 rad is forward (X), positive is counter-clockwise
        const float angle = std::atan2(p.y, p.x);
        const float range = std::sqrt(p.x * p.x + p.y * p.y);
        
        // Fill the output laser scan candidates
        angles.emplace_back(angle);
        ranges.emplace_back(range);
        intensities.emplace_back(p.intensity);

        // Add the point to the debug cloud
        if (publish_debug_cloud_) 
        {
            cloud_out->points.emplace_back(p);
        }
    }

    // Publish the debug cloud
    if (!cloud_out->points.empty() && publish_debug_cloud_) 
    {
        sensor_msgs::PointCloud2 out_msg;
        out_msg.header = cloud_msg->header;
        pcl::toROSMsg(*cloud_out, out_msg);
        debug_cloud_pub_.publish(out_msg);
    }

    // Prapare the output laser scan values according to angle resolution
    if (!ranges.empty()) 
    {
        std::pair<float, float> min_max_range, min_max_angle;
        filterRangeAndIntensityVectors(ranges, intensities, angles, min_max_range, min_max_angle);

        // Publish the output laser scan
        scan_out.time_increment = scan_out.scan_time/static_cast<float>(ranges.size() - 1);
        scan_out.angle_min = min_max_angle.first;
        scan_out.angle_max = min_max_angle.second;
        scan_out.angle_increment = angle_resolution_;
        scan_out.range_min = min_max_range.first;
        scan_out.range_max = min_max_range.second;
        scan_out.ranges = ranges;
        scan_out.intensities = intensities;
        out_scan_pub_.publish(scan_out);
    }
  }

void CloudFilter::filterRangeAndIntensityVectors(std::vector<float>& ranges, std::vector<float>& intensities, std::vector<float>& angles, 
                        std::pair<float, float>& min_max_range, std::pair<float, float>& min_max_angle) 
{
    // Sort the ranges and intensities based on the angles
    std::vector<std::pair<float, std::pair<float, float>>> angles_ranges_intensities;
    for (size_t i = 0; i < ranges.size(); ++i) 
    {
        angles_ranges_intensities.emplace_back(std::make_pair(angles[i], std::make_pair(ranges[i], intensities[i])));
    }
    std::sort(angles_ranges_intensities.begin(), angles_ranges_intensities.end(), 
    [](const std::pair<float, std::pair<float, float>>& a, const std::pair<float, std::pair<float, float>>& b)
     { return a.first < b.first; });

    // Refill the original vectors with the sorted values
    for (std::size_t i = 0; i < angles_ranges_intensities.size(); ++i) 
    {
        angles[i] = angles_ranges_intensities[i].first;
        ranges[i] = angles_ranges_intensities[i].second.first;
        intensities[i] = angles_ranges_intensities[i].second.second;
    }

    // Filter the ranges and intensities based on the angle resolution
    std::vector<float> filtered_ranges, filtered_intensities;
    filtered_ranges.reserve(ranges.size());
    filtered_intensities.reserve(intensities.size());
    float current_angle = angles[0], min_range = 1e6, max_range = -1e6;
    for (std::size_t i = 1; i < angles.size(); ++i) 
    {
        if (angles[i] - current_angle >= angle_resolution_) 
        {
            filtered_ranges.emplace_back(ranges[i]);
            filtered_intensities.emplace_back(intensities[i]);
            current_angle += angle_resolution_;
            if (ranges[i] < min_range) 
            {
                min_range = ranges[i];
            }
            if (ranges[i] > max_range) 
            {
                max_range = ranges[i];
            }
        }
    }

    // Fill the minimum and maximum range values
    min_max_range = std::make_pair(min_range, max_range);
    min_max_angle = std::make_pair(angles.front(), current_angle - angle_resolution_);

    // Update the original vectors
    ranges = filtered_ranges;
    intensities = filtered_intensities;
}

const bool CloudFilter::filterBoatPoints(const PointIn& p) 
{
    // If inside a 2D box that surrounds the boat in XY plane, filter out as well
    if (p.x > negative_range_.x && p.x < positive_range_.x && p.y > negative_range_.y && p.y < positive_range_.y) 
    {
        return true;
    }
    return false;
}

const bool CloudFilter::filterRange(const PointIn& p) 
{
    return std::sqrt(p.x * p.x + p.y * p.y) > max_xy_range_;
}

const bool CloudFilter::filterIntensity(const PointIn& p) 
{
    return p.intensity < min_intensity_;
}
