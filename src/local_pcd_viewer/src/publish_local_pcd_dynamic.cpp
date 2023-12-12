#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "local_pcd_filter");
    ros::NodeHandle nh;

    // Set up a publisher
    ros::Publisher dynamic_origin_pub = nh.advertise<sensor_msgs::PointCloud2>("/local_pcd/dynamic_origin", 1);
    ros::Publisher dynamic_filter_pub = nh.advertise<sensor_msgs::PointCloud2>("/local_pcd/dynamic", 1);

    // Load the PCD file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr x_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr y_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr z_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());

    sensor_msgs::PointCloud2 original_output;
    sensor_msgs::PointCloud2 filtered_output;

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/irol/catkin_ws/src/local_pcd_viewer/local_pcd_files/localvmap_dynamic_1118_90.pcd", *original_cloud) == -1)
    {
        ROS_ERROR("Couldn't read file localvmap_dynamic_1118.pcd");
        return (-1);
    }

    *filtered_cloud = *original_cloud; // Copy the original point cloud to the filtered point cloud

    float selected_point_x = 0.3; // Example selected point x-coordinate
    float selected_point_y = 1.3; // Example selected point y-coordinate
    float selected_point_z = 0.65; // Example selected point z-coordinate

    float range = 1.0;

    // Apply PassThrough filter
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(original_cloud);

    // For the x-axis
    pass.setFilterFieldName("x");
    pass.setFilterLimits(selected_point_x - range / 2, selected_point_x + range / 2);
    pass.setNegative(false); // true means remove points inside the range
    pass.filter(*x_filtered);

    // For the y-axis
    // pass.setFilterFieldName("y");
    // pass.setFilterLimits(selected_point_y - range / 2, selected_point_y + range / 2);
    // pass.setNegative(false); // true means remove points inside the range
    // pass.filter(*y_filtered);

    // For the z-axis
    // pass.setFilterFieldName("z");
    // pass.setFilterLimits(selected_point_z - range / 2, selected_point_z + range / 2);
    // pass.setNegative(false); // true means remove points inside the range
    // pass.filter(*z_filtered);

    *filtered_cloud = *x_filtered;

    // convert to ROS message
    pcl::toROSMsg(*original_cloud, original_output);
    original_output.header.frame_id = "map";
    pcl::toROSMsg(*filtered_cloud, filtered_output);
    filtered_output.header.frame_id = "map";

    // Publishing loop
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        dynamic_origin_pub.publish(original_output);
        dynamic_filter_pub.publish(filtered_output);
        loop_rate.sleep();
    }

    return 0;
}
