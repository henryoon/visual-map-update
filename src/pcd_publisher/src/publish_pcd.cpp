#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

#include <pcl/point_cloud.h>
#include <pcl/common/concatenate.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Float32MultiArray.h>

// Global variables
float selected_point_x = 0.0;
float selected_point_y = 0.0;
float selected_point_z = 0.0;

float x_range = 0.4;  // The range around the x-coordinate to filter out
float y_range = 0.8;  // The range around the y-coordinate to filter out
float z_range = 0.25;  // The range around the z-coordinate to filter out

bool is_selected_point_updated = false;  // Whether the selected point is published or not

// Callback function for the '/tf_B2P_selected' topic
void selectedPointCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (msg->data.size() >= 4)
    {
        // Assuming the message format is [Tag ID, x, y, z]
        selected_point_x = msg->data[1];
        selected_point_y = msg->data[2];
        selected_point_z = msg->data[3];
        is_selected_point_updated = true;
        // ROS_INFO("Selected point: (%f, %f, %f)", selected_point_x, selected_point_y, selected_point_z);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_publisher");
    ros::NodeHandle nh;
    ros::Subscriber selected_point_sub = nh.subscribe("/tf_B2P_selected", 1, selectedPointCallback);  // Subscriber for the selected point
    ros::Publisher original_pub = nh.advertise<sensor_msgs::PointCloud2>("pcd", 1);  // Publisher for the original PCD
    ros::Publisher filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_pcd", 1);  // Publisher for the filtered PCD

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);  // Store the original point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);  // Store the filtered point cloud

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr x_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr y_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr z_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());

    sensor_msgs::PointCloud2 original_output;
    sensor_msgs::PointCloud2 filtered_output;
    
    if(pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/irol/catkin_ws/src/pcd_publisher/pcd_file/global_1210_.pcd", *original_cloud) == -1)
    {
        PCL_ERROR("Couldn't read the PCD file.\n");
        return -1;
    }

    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        if(selected_point_sub.getNumPublishers() > 0)
        {
            *filtered_cloud = *original_cloud;  // Copy the original point cloud to the filtered point cloud

            // Apply the pass-through filter for each axis
            pcl::PassThrough<pcl::PointXYZRGB> pass;
            pass.setInputCloud(original_cloud);
            
            // For the x-axis
            pass.setFilterFieldName("x");
            pass.setFilterLimits(selected_point_x - x_range/2, selected_point_x + x_range/2);
            pass.setNegative(true);  // true means remove points inside the range
            pass.filter(*x_filtered);

            // For the y-axis
            pass.setFilterFieldName("y");
            pass.setFilterLimits(selected_point_y - y_range/2, selected_point_y + y_range/2);
            pass.setNegative(true);  // true means remove points inside the range
            pass.filter(*y_filtered);

            // For the z-axis
            pass.setFilterFieldName("z");
            pass.setFilterLimits(selected_point_z - z_range/2, selected_point_z + z_range/2);
            pass.setNegative(true);  // true means remove points inside the range
            pass.filter(*z_filtered);

            *filtered_cloud = *x_filtered + *y_filtered + *z_filtered;

            // Convert to ROS message
            pcl::toROSMsg(*original_cloud, original_output);
            original_output.header.frame_id = "map";

            pcl::toROSMsg(*filtered_cloud, filtered_output);
            filtered_output.header.frame_id = "map";

            original_pub.publish(original_output);     // Publish the original PCD
            filtered_pub.publish(filtered_output);     // Publish the filtered PCD

            is_selected_point_updated = false;  // Reset the flag
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}