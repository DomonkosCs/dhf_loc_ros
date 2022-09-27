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

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.sub_scan = message_filters.Subscriber("scan", LaserScan)
        self.sub_odom = message_filters.Subscriber("odom", Odometry)
        
        self.gridmap = None
        self.prev_odom = None
        self.filter_initialized = False
        self.measurement_processer = None

        self.srv_get_map = rospy.ServiceProxy("static_map", GetMap)
        self.gridmap = self.get_map()

        # rate of odom is 30 hz, rate of scan is 5 hz
        time_sync_sensors = message_filters.ApproximateTimeSynchronizer([self.sub_scan,self.sub_odom],100,0.1)
        time_sync_sensors.registerCallback(self.cb_scan_odom)
    
    def cb_scan_odom(self,scan_msg, odom_msg):
        """Callback to handle the sensor messages.

        This is only called if the two sensor messages are sufficiently close in time.

        Args: 
            scan_msg (:obj:`sensor_msgs.msg.LaserScan`): Laser scan from LiDAR.
            odom_msg (:obj:`nav_msgs.msg.Odomery`): Odom message from the wheel encoders.
        """
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
        
        
        measurement = self.process_scan(scan)
        measurement = self.measurement_processer.filter_measurements(measurement)

        control_input = [self.prev_odom,odom]

        self.edh.propagate(control_input)
        ekf_propagated_state = self.ekf.propagate(control_input, return_state=True)
        self.edh.update(ekf_propagated_state.covar,measurement)
        self.ekf.update(measurement)
        
        rospy.loginfo(self.edh.filtered_states[-1].pose)

        filtered_state = self.edh.filtered_states[-1].pose

        self.broadcast_pose(filtered_state)

        self.prev_odom = odom

    def extract_odom_msg(self,odom_msg):
        """Creates a planar pose vector from the odom message.

        Args:
            odom_msg (:obj:`nav_msgs.msg.Odomery`)

        Returns:
            :obj:`list` Containing the x,y position in ``m`` and the yaw angle in ``rad``.
        """
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
        """Extracts distance readings from the laser scan.

        ``inf`` readings are substituted by ``None``.

        Args:
            scan_msg (:obj:`sensor_msgs.msg.LaserScan`)

        Returns:
            :obj:`list`: List, containing the ranges for each angle.

        """
        ranges = [None if elem == float('inf') else round(elem,3) for elem in scan_msg.ranges]

        return ranges

    def process_scan(self,scan):
        """Appends angles to the range-only scan readings.

        Assumes that every range reading is evenly distributed accross 360 degs.
        Ranges with value ``None`` are excluded together with their angle.

        Args:
            scan (:obj:`list`): Range readings.
        
        Returns:
            :obj:`list of (angle,range)`

        """
        angles = np.linspace(0, 2 * np.pi, len(scan), endpoint=False)
        angle_range =[
                (angle, range)
                for angle, range in zip(angles, scan)
                if range is not None
            ]
        return angle_range

    def get_map_from_srv(self):
        """Requests the occupancy grid map.

        Calls the service ``static_map`` from the ``map_server`` node which returns a request (:obj:`nav_msgs.srv.GetMap`)

        Returns:
            :obj:`nav_msgs.srv.GetMap`: The message for successful srv. call, 
            or ``None`` if the call has failed.

        """
        rospy.wait_for_service("static_map")
        try:
            map_response = self.srv_get_map()
            return map_response
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)
            return None
            
    def get_map(self):
        """Creates the ``GridMap`` for the localization.

        Returns:
            :obj:`dhflocalization.gridmap.GridMap`: The internal object for handling the OGM.

        """
        map_response = self.get_map_from_srv()
        map_message = map_response.map
        map_data = np.asarray( map_message.data )

        width = map_message.info.width
        height = map_message.info.height
        resolution = map_message.info.resolution
        map_array = map_data.reshape(height,width)
        map_array = self.convert_occupancy_representation(map_array)

        center_x = 10 #TODO
        center_y = 10.05 #TODO

        occupancy_grid_map = GridMap.load_grid_map_from_array(np.flip(map_array,0),resolution,center_x,center_y)

        return occupancy_grid_map
    
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
    
    def broadcast_pose(self,pose):
        """Broadcast the transormation between the map and the robot.

        Uses the ``map`` frame as a base and the ``base_footprint`` as the child.

        Args:
            pose (:obj:`(3,1) np.ndarray`): Array containing the x,y poses and the yaw angle.
        """
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = pose[0,0]
        transform.transform.translation.y = pose[1,0]
        transform.transform.translation.z = 0.0

        quat = quaternion_from_euler(0,0,pose[2,0])
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z= quat[2]
        transform.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(transform)
    
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