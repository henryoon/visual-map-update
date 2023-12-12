#!/usr/bin/env python3

import rospy
import rospkg
import numpy as np
import matplotlib.pyplot as plt
# import rtabmap_ros.msg as rtabmap_msg
# import rtabmap_ros.srv as rtabmap_srv
from rtabmap_ros.msg import MapData
from rtabmap_ros.srv import GetMap

# Initialize ROS node
rospy.init_node('map_resolution')

# Define ROS services and topics
map_data_topic = '/rtabmap/mapData'
get_map_data = rospy.ServiceProxy('/rtabmap/get_map_data', GetMap)

# Define ground truth poses (optional)
# gt_poses = np.loadtxt('/path/to/ground_truth_poses.txt')

# Call ROS service to get map data
map_data = get_map_data().data
print(map_data)

# Compute RMSE
# if gt_poses is not None:
#     est_poses = np.array([[node.pose.position.x, node.pose.position.y, node.pose.position.z] for node in map_data.graph.nodes])
#     rmse = np.sqrt(np.mean(np.sum((est_poses - gt_poses)**2, axis=1)))

# Compute Map Consistency
map_consistency = map_data.graph.get_metric('MapConsis')

# Compute Map Completeness
map_completeness = map_data.graph.get_metric('MapCompl')

# Compute Map Density
map_density = map_data.graph.get_metric('MapDensity')

# Print the results
# if gt_poses is not None:
#     print('RMSE: {:.2f} meters'.format(rmse))
print('Map Consistency: {:.2f}'.format(map_consistency))
print('Map Completeness: {:.2f}%'.format(map_completeness * 100))
print('Map Density: {:.2f} points/m^3'.format(map_density))

# Visualize the map (optional)
plt.figure()
plt.scatter(est_poses[:,0], est_poses[:,1])
plt.show()