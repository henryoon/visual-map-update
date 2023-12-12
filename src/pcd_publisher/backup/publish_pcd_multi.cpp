#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

#include <pcl/point_cloud.h>
#include <pcl/common/concatenate.h>
#include <tf2_ros/transform_broadcaster.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_publisher");
    ros::NodeHandle nh;
    ros::Publisher original_pub = nh.advertise<sensor_msgs::PointCloud2>("pcd", 1);  // Publisher for the original PCD
    ros::Publisher filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_pcd", 1);  // Publisher for the filtered PCD

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);  // Store the original point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud01(new pcl::PointCloud<pcl::PointXYZRGB>);  // Store the filtered point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud02(new pcl::PointCloud<pcl::PointXYZRGB>);  // Store the filtered point cloud

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr x1_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr y1_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr z1_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr x2_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr y2_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr z2_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());

    sensor_msgs::PointCloud2 original_output;
    sensor_msgs::PointCloud2 filtered_output01;
    sensor_msgs::PointCloud2 filtered_output02;
    
    if(pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/irol/catkin_ws/src/pcd_publisher/pcd_file/global_1112.pcd", *original_cloud) == -1)
    {
        PCL_ERROR("Couldn't read the PCD file.\n");
        return -1;
    }

    *filtered_cloud01 = *original_cloud;  // Copy the original point cloud to the filtered point cloud
    *filtered_cloud02 = *original_cloud;  // Copy the original point cloud to the filtered point cloud

    // Define your box region limits based on a selected point and a range
    float selected_point_x1 = -0.0;  // Example selected point x-coordinate
    float selected_point_y1 = 1.5;  // Example selected point y-coordinate
    float selected_point_z1 = 0.65;  // Example selected point z-coordinate

    float selected_point_x2 = -1.0;  // Example selected point x-coordinate
    float selected_point_y2 = 1.5;  // Example selected point y-coordinate
    float selected_point_z2 = 0.65;  // Example selected point z-coordinate

    float x_range = 0.5;  // The range around the x-coordinate to filter out
    float y_range = 1.0;  // The range around the y-coordinate to filter out
    float z_range = 0.5;  // The range around the z-coordinate to filter out

    // Apply the pass-through filter for each axis
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(original_cloud);
    
    // For the x-axis
    pass.setFilterFieldName("x");
    pass.setFilterLimits(selected_point_x1 - x_range/2, selected_point_x1 + x_range/2);
    pass.setNegative(true);  // true means remove points inside the range
    pass.filter(*x1_filtered);
    // For the y-axis
    pass.setFilterFieldName("y");
    pass.setFilterLimits(selected_point_y1 - y_range/2, selected_point_y1 + y_range/2);
    pass.setNegative(true);
    pass.filter(*y1_filtered);
    // For the z-axis
    pass.setFilterFieldName("z");
    pass.setFilterLimits(selected_point_z1 - z_range/2, selected_point_z1 + z_range/2);
    pass.setNegative(true);
    pass.filter(*z1_filtered);

    // For the x-axis
    pass.setFilterFieldName("x");
    pass.setFilterLimits(selected_point_x2 - x_range/2, selected_point_x2 + x_range/2);
    pass.setNegative(true);  // true means remove points inside the range
    pass.filter(*x2_filtered);
    // For the y-axis
    pass.setFilterFieldName("y");
    pass.setFilterLimits(selected_point_y2 - y_range/2, selected_point_y2 + y_range/2);
    pass.setNegative(true);
    pass.filter(*y2_filtered);
    // For the z-axis
    pass.setFilterFieldName("z");
    pass.setFilterLimits(selected_point_z2 - z_range/2, selected_point_z2 + z_range/2);
    pass.setNegative(true);
    pass.filter(*z2_filtered);

    *filtered_cloud01 = *x1_filtered + *y1_filtered + *z1_filtered; 
    *filtered_cloud02 = *x2_filtered + *y2_filtered + *z2_filtered;

    // Convert to ROS message
    pcl::toROSMsg(*original_cloud, original_output);
    original_output.header.frame_id = "map";

    pcl::toROSMsg(*filtered_cloud01, filtered_output01);
    filtered_output01.header.frame_id = "map";

    pcl::toROSMsg(*filtered_cloud02, filtered_output02);
    filtered_output02.header.frame_id = "map";

    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        original_pub.publish(original_output);     // Publish the original PCD
        filtered_pub.publish(filtered_output01, filtered_output02);     // Publish the filtered PCD
        loop_rate.sleep();
    }

    return 0;
}
