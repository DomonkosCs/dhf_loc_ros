#!/usr/bin/env python

import rospy
import message_filters
from tf.transformations import euler_from_quaternion, quaternion_from_euler
# TODO replace with tf_conversions, see tf2 tutorial
import tf2_ros
from nav_msgs.srv import GetMap
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped

import numpy as np
import matplotlib.pyplot as plt

from dhflocalization.gridmap import GridMap
from dhflocalization.filters import EDH, EKF
from dhflocalization.kinematics import OdometryMotionModel
from dhflocalization.measurement import MeasurementModel,MeasurementProcessor

class DhfLocalizationNode():
    def __init__(self) -> None:
        rospy.init_node('dhf_localization_node')
        rospy.loginfo("Node created")

        self.sub_scan = message_filters.Subscriber("scan", LaserScan)
        self.sub_odom = message_filters.Subscriber("odom", Odometry )
        self.gridmap = None
        self.prev_odom = None
        self.filter_initialized = False
        self.measurement_processer = None
        self.br = tf2_ros.TransformBroadcaster()


        self.gridmap = self.get_map()
        # rate of odom is 30 hz, rate of scan is 5 hz
        time_sync = message_filters.ApproximateTimeSynchronizer([self.sub_scan,self.sub_odom],100,0.1)
        time_sync.registerCallback(self.cb_scan_odom)
    
    def cb_scan_odom(self,scan_msg, odom_msg):
        if self.gridmap is None:
            rospy.loginfo("Waiting for map.")
            return

        if not self.filter_initialized:
            self.init_filter()

        odom = self.extract_odom_msg(odom_msg)
        scan = self.extract_scan_msg(scan_msg)

        if self.prev_odom is None:
            self.prev_odom = odom
            rospy.loginfo("Ignoring first detection.")
            return

        angles = np.linspace(0, 2 * np.pi, len(scan), endpoint=False)
        angle_range =[
                (angle, range)
                for angle, range in zip(angles, scan)
                if range is not None
            ]

        measurement = self.measurement_processer.filter_measurements(angle_range)

        control_input = [self.prev_odom,odom]

        self.edh.propagate(control_input)
        ekf_propagated_state = self.ekf.propagate(control_input, return_state=True)
        self.edh.update(ekf_propagated_state.covar,measurement)
        self.ekf.update(measurement)
        
        rospy.loginfo(self.edh.filtered_states[-1].pose)

        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = self.edh.filtered_states[-1].pose[0,0]
        transform.transform.translation.y = self.edh.filtered_states[-1].pose[1,0]
        transform.transform.translation.z = 0.0
        quat = quaternion_from_euler(0,0,self.edh.filtered_states[-1].pose[2,0])
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z= quat[2]
        transform.transform.rotation.w = quat[3]
        
        self.br.sendTransform(transform)
        


        self.prev_odom = odom

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
            gridmap = self.process_map_message(map_message)
            return gridmap
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)
            return None
            
    
    def process_map_message(self,map_message):
        map_data = np.asarray( map_message.data )
        width = map_message.info.width
        height = map_message.info.height
        resolution = map_message.info.resolution
        map_array = map_data.reshape(height,width)

        map_array = self.convert_occupancy_representation(map_array)
        center_x = 10 #TODO
        center_y = 10.05 #TODO
        return GridMap.load_grid_map_from_array(np.flip(map_array,0),resolution,center_x,center_y)
    
    def convert_occupancy_representation(self,map_array):
        """Converts cell occupancy value representation.

        The ``map_server`` node uses the value ``100`` to indicate occupied cells,
        ``-1`` for unknown and `0` for free. This function tranforms these values to
        `0` representing free and unknown cells, and `1` representing the occupied ones.

        Args:
            map_array (:obj:`np.array`): Occ. values of the map cells in a 2D array.
        
        Returns: 
            :obj:`np.array`: Array with the same shape containing the converted values.


        """
        max_val = map_array.max()
        return np.where(map_array < max_val,0,1)
    
    def init_filter(self):
        cfg_max_ray_number = 500
        cfg_odometry_alpha_1 = 0.1
        cfg_odometry_alpha_2 = 0.1
        cfg_odometry_alpha_3 = 0.1
        cfg_odometry_alpha_4 = 0.1

        motion_model = OdometryMotionModel(
            [
                cfg_odometry_alpha_1,
                cfg_odometry_alpha_2,
                cfg_odometry_alpha_3,
                cfg_odometry_alpha_4,
            ]
        )

        cfg_measurement_range_noise_std = 0.01
        measurement_model = MeasurementModel(self.gridmap, cfg_measurement_range_noise_std)

        cfg_edh_particle_number = 1000
        cfg_edh_lambda_number = 10
        cfg_init_gaussian_mean = np.array([-3.0, 1.0, 0])
        cfg_init_gaussian_covar = np.array(
            [[0.1**2, 0, 0], [0, 0.1**2, 0], [0, 0, 0.05**2]]
        )

        self.measurement_processer = MeasurementProcessor(max_ray_number=cfg_max_ray_number)
        self.ekf = EKF(motion_model, measurement_model)
        self.edh = EDH(
            motion_model=motion_model,
            measurement_model=measurement_model,
            particle_num=cfg_edh_particle_number,
            lambda_num=cfg_edh_lambda_number,
        )
        self.edh.init_particles_from_gaussian(
            cfg_init_gaussian_mean, cfg_init_gaussian_covar, return_state=False
        )

        # Another option is to set the return_state flag on edh.init_particles_from_gaussian,
        # and use returned state to initialize ekf.
        self.ekf.init_state(mean=cfg_init_gaussian_mean, covar=cfg_init_gaussian_covar)
            



if __name__ == '__main__':
    dhf_localization_node = DhfLocalizationNode()
    rospy.spin()