#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <visualization_msgs/Marker.h>
#include <pcl/io/pcd_io.h>
#include <sstream>

ros::Publisher marker_pub;  // Define a global publisher for the Marker

void mapDataCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Convert to PCL data type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Define the voxel dimensions and position

    float selected_point_x = 0.1;  // Example selected point x-coordinate
    float selected_point_y = 1.3;  // Example selected point y-coordinate
    float selected_point_z = 0.65;  // Example selected point z-coordinate

    float voxel_size = 0.5;  // The size of the voxel
    Eigen::Vector4f minPoint(selected_point_x - voxel_size/2, selected_point_y - voxel_size/2, selected_point_z - voxel_size/2, 0);
    Eigen::Vector4f maxPoint(selected_point_x + voxel_size/2, selected_point_y + voxel_size/2, selected_point_z + voxel_size/2, 0);

    // Filter points within the voxel
    pcl::CropBox<pcl::PointXYZ> cropBoxFilter;
    cropBoxFilter.setMin(minPoint);
    cropBoxFilter.setMax(maxPoint);
    cropBoxFilter.setInputCloud(cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    cropBoxFilter.filter(*cloudFiltered);

    // Count and output the number of points within the voxel
    int count = cloudFiltered->points.size();
    ROS_INFO("Number of points in the voxel: %d", count);

    // Create and publish the Marker for the voxel
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "voxel";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker to the center of the voxel
    marker.pose.position.x = selected_point_x;
    marker.pose.position.y = selected_point_y;
    marker.pose.position.z = selected_point_z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker (size of the voxel)
    marker.scale.x = voxel_size;
    marker.scale.y = voxel_size;
    marker.scale.z = voxel_size;

    // Set the color and transparency of the marker
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.3;

    marker.lifetime = ros::Duration();

    // Publish the marker
    marker_pub.publish(marker);

    // // Generate a unique filename for each iteration
    // static int file_count = 0;
    // std::stringstream ss;
    // ss << "/home/irol/catkin_ws/src/pcd_publisher/pcd_iteration/cloud_" << file_count << ".pcd";

    // // convert to PCL data type
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::fromROSMsg(*cloud_msg, *cloud);

    // // save PCD file
    // pcl::io::savePCDFileASCII(ss.str(), *cloud);
    // ROS_INFO("Saved %s", ss.str().c_str());

    // file_count++;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "count_pcd_in_voxel");
    ros::NodeHandle nh;

    marker_pub = nh.advertise<visualization_msgs::Marker>("/voxel", 1);  // Initialize the publisher
    ros::Subscriber sub = nh.subscribe("rtabmap/cloud_map", 1000, mapDataCallback);

    ros::spin();

    return 0;
}
