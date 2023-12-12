#!/bin/bash
# Launch the first file
echo "Launching publish_pcd.launch"
roslaunch pcd_publisher publish_pcd.launch &
# Wait for a specified time
echo "Waiting for 10 seconds"
sleep 10
# Launch the second file
echo "Launching object_position.launch"# Launch the second file
roslaunch pcd_publisher object_position.launch