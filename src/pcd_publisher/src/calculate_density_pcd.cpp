#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <visualization_msgs/Marker.h>
#include <pcl/io/pcd_io.h>
#include <sstream>
#include <Eigen/Dense>

ros::Publisher marker_pub;  // Define a global publisher for the Marker

void processPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const std::string& topic)
{
    // Convert to PCL data type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Define the voxel dimensions and position
    float selected_point_x = 0.08;  // Example selected point x-coordinate
    float selected_point_y = 1.18;  // Example selected point y-coordinate
    float selected_point_z = 0.51;  // Example selected point z-coordinate
    float voxel_size = 0.2;  // The size of the voxel

    Eigen::Vector4f minPoint(selected_point_x - voxel_size/2, selected_point_y - voxel_size/10, selected_point_z - voxel_size/2, 0);
    Eigen::Vector4f maxPoint(selected_point_x + voxel_size/2, selected_point_y + voxel_size/10, selected_point_z + voxel_size/2, 0);

    // Filter points within the voxel
    pcl::CropBox<pcl::PointXYZ> cropBoxFilter;
    cropBoxFilter.setMin(minPoint);
    cropBoxFilter.setMax(maxPoint);
    cropBoxFilter.setInputCloud(cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    cropBoxFilter.filter(*cloudFiltered);

    // Count and output the number of points within the voxel
    int count = cloudFiltered->points.size();
    ROS_INFO("Number of points in the voxel for topic %s: %d", topic.c_str(), count);

    // Calculate and output the density
    float voxel_volume = (voxel_size*1000/7) * (voxel_size/10*1000/7) * (voxel_size*1000/7); // Assuming a cubic voxel
    float density = static_cast<float>(count) / voxel_volume * 50;
    ROS_INFO("Density of points in the voxel for topic %s: %f", topic.c_str(), density);

    // Create and publish the Marker for the voxel
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "voxel";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = selected_point_x;
    marker.pose.position.y = selected_point_y;
    marker.pose.position.z = selected_point_z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = voxel_size;
    marker.scale.y = voxel_size/5;
    marker.scale.z = voxel_size;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.3;
    marker.lifetime = ros::Duration();
    marker_pub.publish(marker);
}

void rtabmapCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    processPointCloud(cloud_msg, "rtabmap/cloud_map");
}

void pcdCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    processPointCloud(cloud_msg, "/filtered_pcd");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calculate_density_pcd");
    ros::NodeHandle nh;

    marker_pub = nh.advertise<visualization_msgs::Marker>("/voxel", 1);  // Initialize the publisher
    ros::Subscriber rtabmap_sub = nh.subscribe("rtabmap/cloud_map", 1000, rtabmapCallback);
    ros::Subscriber pcd_sub = nh.subscribe("/filtered_pcd", 1000, pcdCallback);

    ros::spin();

    return 0;
}
