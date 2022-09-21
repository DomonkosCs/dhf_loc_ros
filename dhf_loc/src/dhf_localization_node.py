#!/usr/bin/env python

import rospy
import message_filters
import tf2_ros
from tf.transformations import euler_from_quaternion
from nav_msgs.srv import GetMap
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import numpy as np
# import matplotlib.pyplot as plt

from dhflocalization.gridmap import GridMap

class DhfLocalizationNode():
    def __init__(self) -> None:
        rospy.init_node('dhf_localization_node')

        self.sub_scan = message_filters.Subscriber("scan", LaserScan)
        self.sub_odom = message_filters.Subscriber("odom", Odometry )

        # rate of odom is 30 hz, rate of scan is 5 hz
        time_sync = message_filters.ApproximateTimeSynchronizer([self.sub_scan,self.sub_odom],100,0.1)
        time_sync.registerCallback(self.cb_scan_odom)
    
    def cb_scan_odom(self,scan_msg, odom_msg):

        planar_pose = self.extract_odom_msg(odom_msg)
        scan = self.extract_scan_msg(scan_msg)
        
    
    def extract_odom_msg(self,odom_msg):
        pose = odom_msg.pose.pose
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)

        x = pose.position.x
        y = pose.position.y

        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]

        return [x,y,yaw]

    def extract_scan_msg(self,scan_msg):
        ranges = [None if elem == float('inf') else round(elem,3) for elem in scan_msg.ranges]

        return ranges


    def get_map(self):
        rospy.wait_for_service("static_map")
        try:
            map_srv = rospy.ServiceProxy("static_map", GetMap)
            map_response = map_srv()
            map_message = map_response.map
            self.process_map_message(map_message)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    
    def process_map_message(self,map_message):
        map_data = np.asarray( map_message.data )
        width = map_message.info.width
        height = map_message.info.height
        resolution = map_message.info.resolution
        map_array = map_data.reshape(height,width)
        center_x = 10 #TODO
        center_y = 10.05 #TODO
        ogm = GridMap.load_grid_map_from_array(map_array,resolution,center_x,center_y)
        



if __name__ == '__main__':
    dhf_localization_node = DhfLocalizationNode()
    rospy.spin()