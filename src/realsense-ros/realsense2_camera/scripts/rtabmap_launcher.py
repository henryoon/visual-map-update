#!/usr/bin/env python3
import rospy
import roslaunch
import time

def launch_rtabmap():
    # Initialize the ROS node
    rospy.init_node('rtabmap_launcher', anonymous=True)

    # Create a UUID and initialize roslaunch
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    # Define the launch file and arguments
    cli_args = ['rtabmap_ros', 'rtabmap.launch', 
                'args:=--delete_db_on_start', 
                'rgb_topic:=/camera/color/raw_image',
                'depth_topic:=/camera/aligned_depth_to_color/raw_image',
                'camera_info_topic:=/camera/color/camera_info',
                'depth_camera_info_topic:=/camera/depth/camera_info',
                'servicemapviz:=false', 
                'rviz:=true']
    roslaunch_args = cli_args[2:]

    # Create the launch file object
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

    # Start the launch
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    parent.start()

    rospy.loginfo("rtabmap started.")
    return parent

def stop_rtabmap(parent):
    parent.shutdown()
    rospy.loginfo("rtabmap stopped.")

if __name__ == "__main__":
    try:
        rtabmap_launch = launch_rtabmap()
        time.sleep(10)  # Run rtabmap for 10 seconds
        stop_rtabmap(rtabmap_launch)
    except rospy.ROSInterruptException:
        pass
