#!/usr/bin/env python
# -*- coding: utf-8 -*-

##############################################
#   /pedsim_simulator/parameter_descriptions
#   /pedsim_simulator/parameter_updates
#   /pedsim_simulator/robot_position
# v /pedsim_simulator/simulated_agents
#   /pedsim_simulator/simulated_groups
#   /pedsim_simulator/simulated_walls
#   /pedsim_simulator/simulated_waypoints
#   /pedsim_visualizer/forces
#   /pedsim_visualizer/tracked_groups
#   /pedsim_visualizer/tracked_persons
#   /pedsim_visualizer/walls
#   /pedsim_visualizer/walls_array
#   /pedsim_visualizer/waypoints
##############################################
from genpy.rostime import Duration
import rospy
import image_geometry
import numpy as np
import math
import sys
from campusrover_msgs.msg import BboxAngle, BboxAngleArray
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Vector3, Quaternion
from pedsim_msgs.msg import AgentGroups, AgentGroup

from tf.transformations import *


obstacles_msg = BboxAngleArray()
obstacles_msg.header.seq = 0

obstacle_msg = BboxAngle()


def save_agent_info(bbox_2d):
    global CAMERA_MODEL, FIRST_TIME, marker, angle_pub, marker_pub, obstacle_msg, id

    if FIRST_TIME:
        return

    id = 0

    for item in bbox_2d.detections:
        if (0 < item.results[0].id < 5):
            # get bbox center
            points2D = (item.bbox.center.x, item.bbox.center.y)

            obstacle_msg.angle = tmp
            obstacle_msg.label = item.results[0].id
            obstacles_msg.obstacles.append(obstacle_msg)

    marker_pub.publish(markers)

    obstacles_msg.header.stamp = rospy.Time.now()
    obstacles_msg.header.seq = obstacles_msg.header.seq + 1

    angle_pub.publish(obstacles_msg)


def camera_info_callback(camera_info):
    global FIRST_TIME, CAMERA_MODEL

    if FIRST_TIME:
        FIRST_TIME = False

    # Setup camera model
        rospy.loginfo('Setting up camera model')
        CAMERA_MODEL.fromCameraInfo(camera_info)


def listener(agent_info, Detection_2D):

    rospy.Subscriber(agent_info, AgentGroups, save_agent_info)
    # Keep python from exiting until this node is stopped
    try:
        print('working')
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Shutting down')


if __name__ == '__main__':

    rospy.init_node('bbox_project_to_lidar', anonymous=True)

    agent_info = '/pedsim_simulator/simulated_agents'
    Detection_2D = '/detectnet/detections'

    listener(camera_info, Detection_2D)
