#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

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
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "count_pcd_in_voxel");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("rtabmap/cloud_map", 1000, mapDataCallback);

    ros::spin();

    return 0;
}
