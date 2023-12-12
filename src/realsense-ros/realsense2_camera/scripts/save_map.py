#!/usr/bin/env python3

import math
from math import sin, cos, pi

import rospy
import tf
from rtabmap_ros.msg import MapData


class Save_map:
    def __init__(self):
        self.pub = rospy.Publisher('map', MapData, latch=True)
        self.count = 0
    def callback(self, msg):
        map_data = MapData()
        map_data.header = msg.header
        map_data.graph = msg.graph
        map_data.nodes = msg.nodes
        self.pub.publish(map_data)
        # self.count = 1


if __name__ == '__main__':
    rospy.init_node('save_map')
    save_map = Save_map()
    sub = rospy.Subscriber('/rtabmap/mapData', MapData, save_map.callback)
    rospy.spin()
