#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char** argv) {
    // Initialize ROS and name our node "local_pcd_publisher"
    ros::init(argc, argv, "local_pcd_publisher");
    ros::NodeHandle nh;

    // Create a ROS publisher to publish messages on the "local_pcd" topic
    ros::Publisher pcd_pub = nh.advertise<sensor_msgs::PointCloud2>("local_pcd", 1);

    // Set the loop rate, this is how often we publish the PCD data
    ros::Rate loop_rate(1); // 1 Hz or once per second

    // Array of PCD file names
    std::vector<std::string> pcd_files = {
        // "/home/irol/hj_ws/src/local_pcd_viewer/local_pcd_files/304_1104_localmap01.pcd",
        "/home/irol/hj_ws/src/local_pcd_viewer/local_pcd_files/304_1104_localmap02.pcd",
        // "/home/irol/hj_ws/src/local_pcd_viewer/local_pcd_files/304_1104_localmap03.pcd",
        // "/home/irol/hj_ws/src/local_pcd_viewer/local_pcd_files/304_1104_localmap04.pcd"
    };

    while (ros::ok()) {
        for (const auto& file_name : pcd_files) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

            // Load the PCD file
            if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(file_name, *cloud) == -1) {
                PCL_ERROR("Couldn't read file %s \n", file_name.c_str());
                return (-1);
            }

            // Convert to ROS data type
            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(*cloud, output);
            output.header.frame_id = "map"; // Change to your frame_id
            output.header.stamp = ros::Time::now();

            // Publish the data
            pcd_pub.publish(output);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}