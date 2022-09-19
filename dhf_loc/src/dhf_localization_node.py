#!/usr/bin/env python

import rospy
from nav_msgs.srv import GetMap

import numpy as np
import matplotlib.pyplot as plt
from dhflocalization.gridmap import GridMap

class DhfLocalizationNode():
    def __init__(self) -> None:
        rospy.init_node('dhf_localization_node')
        pass

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
    dhf_localization_node.get_map()
    rospy.spin()