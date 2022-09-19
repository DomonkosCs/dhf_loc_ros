#!/usr/bin/env python

import rospy
from nav_msgs.srv import GetMap

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
            print( map_response.map.info )
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)



if __name__ == '__main__':
    dhf_localization_node = DhfLocalizationNode()
    dhf_localization_node.get_map()
    rospy.spin()