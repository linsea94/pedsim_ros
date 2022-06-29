#!/usr/bin/env python3
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
import numpy as np
import sys
import pandas as pd
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Vector3, Quaternion
from pedsim_msgs.msg import AgentStates, AgentState

FIRST_TIME = True
file_name = rospy.get_param("/collect_data/file_name")
file_path = '/home/linsea/motion_ws/src/pedsim_ros/data_process/datas/'+file_name+'.csv'


def save_agent_info(agents_info):
    global FIRST_TIME

    agents_lst=[]

    for item in agents_info.agent_states:
        agent_info = [agents_info.header.seq, item.id, item.pose.position.x, item.pose.position.z, 
                        item.pose.position.y, item.twist.linear.x, item.twist.linear.z, item.twist.linear.y]
        agents_lst.append(agent_info)

    df2 = pd.DataFrame(agents_lst)
    df2.columns = ['frame_number', 'ID', 'pos_x', 'pos_z', 'pos_y', 'vel_x', 'vel_z', 'vel_y']
    
    with open(file_path,mode= 'a') as f:
        df2.to_csv(f, header = False, index = False)


def listener(agent_info):

    rospy.Subscriber(agent_info, AgentStates, save_agent_info)

    # Keep python from exiting until this node is stopped
    try:
        print('working')
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Shutting down')


if __name__ == '__main__':

    rospy.init_node('collect_data', anonymous=True)

    agents_info = '/pedsim_simulator/simulated_agents'

    listener(agents_info)
