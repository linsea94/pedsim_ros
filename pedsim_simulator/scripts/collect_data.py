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
import os
home = os.path.expanduser("~")

FIRST_TIME = True
file_name = rospy.get_param("/collect_data/file_name")
file_path = home +'/motion_ws/src/pedsim_ros/data_process/datas/split_data/'+file_name+'.csv'


def save_agent_info(agents_info):
    global FIRST_TIME

    agents_lst=[]

    for item in agents_info.agent_states:
        
        #make time
        l_nsecs = len(str(agents_info.header.stamp.nsecs))
        if(9-l_nsecs):
            nsecs = str(0)* (9-l_nsecs)+ str(agents_info.header.stamp.nsecs)
        else:
            nsecs = str(agents_info.header.stamp.nsecs)
        time = np.round(float(str(agents_info.header.stamp.secs) + '.' + nsecs), 4)
        
        #rate (2.5)
        if (time % 0.4) < 0.0999:
            # agent_info = [ time, item.id, item.pose.position.x, item.pose.position.z, 
            #                 item.pose.position.y, item.twist.linear.x, item.twist.linear.z, item.twist.linear.y]
            agent_info = [ time, item.id, item.pose.position.x, item.pose.position.y, item.twist.linear.x, item.twist.linear.y]
            agents_lst.append(agent_info)

    try:
        df2 = pd.DataFrame(agents_lst)
        df2.columns = ['time', 'ID', 'pos_x', 'pos_z', 'pos_y', 'vel_x', 'vel_z', 'vel_y']

        with open(file_path,mode= 'a') as f:
            if FIRST_TIME:
                df2.to_csv(f, header = True, index = False)
                FIRST_TIME = False
            else:
                df2.to_csv(f, header = False, index = False)
    except:
        pass


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
    # rate = rospy.Rate(2.5)

    agents_info = '/pedsim_simulator/simulated_agents'

    listener(agents_info)
