#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char** argv) {
    // Initialize ROS and name our node "local_pcd_publisher"
    ros::init(argc, argv, "local_pcd_publisher");
    ros::NodeHandle nh;

    // Create an array of ROS publishers, one for each PCD file
    std::vector<ros::Publisher> pcd_publishers;
    std::vector<std::string> pcd_files = {
        "/home/irol/catkin_ws/src/local_pcd_viewer/local_pcd_files/localvmap_right_1106.pcd",
        "/home/irol/catkin_ws/src/local_pcd_viewer/local_pcd_files/localvmap_front_1106.pcd",
        "/home/irol/catkin_ws/src/local_pcd_viewer/local_pcd_files/localvmap_left_1106.pcd",
    };

    // Initialize publishers for each PCD topic
    for (size_t i = 0; i < pcd_files.size(); ++i) {
        std::stringstream ss;
        ss << "local_pcd_" << std::setw(2) << std::setfill('0') << i + 1; // topic names like local_pcd_01, local_pcd_02, etc.
        pcd_publishers.push_back(nh.advertise<sensor_msgs::PointCloud2>(ss.str(), 1));
    }

    // Set the loop rate, this is how often we publish the PCD data
    ros::Rate loop_rate(1); // 1 Hz or once per second

    while (ros::ok()) {
        for (size_t i = 0; i < pcd_files.size(); ++i) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

            // Load the PCD file
            if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_files[i], *cloud) == -1) {
                PCL_ERROR("Couldn't read file %s \n", pcd_files[i].c_str());
                return (-1);
            }

            // Convert to ROS data type
            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(*cloud, output);
            output.header.frame_id = "map"; // Change to your frame_id
            output.header.stamp = ros::Time::now();

            // Publish the data on the corresponding topic
            pcd_publishers[i].publish(output);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
