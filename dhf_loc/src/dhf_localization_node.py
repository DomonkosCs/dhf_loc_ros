#!/usr/bin/env python

import rospy
import message_filters
from tf.transformations import (
    euler_from_quaternion,
    quaternion_from_euler,
    translation_matrix,
    quaternion_matrix,
    concatenate_matrices,
    translation_from_matrix,
    quaternion_from_matrix,
)

# TODO replace with tf_conversions, see tf2 tutorial
import tf2_ros
from nav_msgs.srv import GetMap
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped

import numpy as np

from dhflocalization.gridmap import GridMap
from dhflocalization.filters import EDH, EKF
from dhflocalization.kinematics import OdometryMotionModel
from dhflocalization.measurement import MeasurementModel, MeasurementProcessor
from dhflocalization.customtypes import ParticleState, StateHypothesis


class DhfLocalizationNode:
    def __init__(self) -> None:
        rospy.init_node("dhf_localization_node")
        rospy.loginfo("Node created")

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.sub_scan = message_filters.Subscriber("scan", LaserScan)
        self.sub_odom = message_filters.Subscriber("odom", Odometry)

        self.gridmap = None
        self.prev_odom = None
        self.prior = None
        self.ekf_prior = None
        self.filter_initialized = False
        self.measurement_processer = None

        self.srv_get_map = rospy.ServiceProxy("static_map", GetMap)
        self.gridmap = self.get_map()

        # rate of odom is 30 hz, rate of scan is 5 hz
        time_sync_sensors = message_filters.ApproximateTimeSynchronizer(
            [self.sub_scan, self.sub_odom], 100, 0.1
        )
        time_sync_sensors.registerCallback(self.cb_scan_odom)

    def cb_scan_odom(self, scan_msg, odom_msg):
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
            self.filter_initialized = True

        odom = self.extract_odom_msg(odom_msg)
        scan = self.extract_scan_msg(scan_msg)

        scan_timestamp = scan_msg.header.stamp

        if self.prev_odom is None:
            self.prev_odom = odom
            rospy.loginfo("Ignoring first detection.")
            return

        measurement = self.process_scan(scan)
        measurement = self.measurement_processer.filter_measurements(measurement)

        control_input = [self.prev_odom, odom]

        prediction = self.edh.propagate(self.prior, control_input)
        ekf_prediction = self.ekf.propagate(self.ekf_prior, control_input)
        posterior = self.edh.update(prediction, ekf_prediction, measurement)
        ekf_posterior = self.ekf.update(ekf_prediction, measurement)

        particle_mean = posterior.mean()
        rospy.loginfo(particle_mean)

        map_to_base_tr = self.transformation_matrix_from_state(particle_mean)

        base_to_odom = self.tf_buffer.lookup_transform(
            "odom", "base_footprint", rospy.Time()
        )
        base_to_odom_tr = self.transformation_matrix_from_msg(base_to_odom)

        map_to_odom_msg = self.msg_from_transformation_matrix(
            map_to_base_tr @ base_to_odom_tr, scan_timestamp
        )
        self.tf_broadcaster.sendTransform(map_to_odom_msg)

        self.prior = posterior
        self.ekf_prior = ekf_posterior

        self.prev_odom = odom

    def transformation_matrix_from_state(self, state):
        tran = translation_matrix([state[0], state[1], 0])
        rot = quaternion_matrix(quaternion_from_euler(0, 0, state[2]))
        transformation_matrix = concatenate_matrices(tran, rot)
        return transformation_matrix

    def transformation_matrix_from_msg(self, msg):
        tran = msg.transform.translation
        tran_matrix = translation_matrix([tran.x, tran.y, tran.z])
        rot = msg.transform.rotation
        rot_matrix = quaternion_matrix([rot.x, rot.y, rot.z, rot.w,])

        transformation_matrix = concatenate_matrices(tran_matrix, rot_matrix)
        return transformation_matrix

    def msg_from_transformation_matrix(self, tr_matrix, stamp):
        tran = translation_from_matrix(tr_matrix)
        rot = quaternion_from_matrix(tr_matrix)

        tolerance = 0.1

        msg = TransformStamped()
        msg.header.stamp = rospy.Time(stamp.to_time() + tolerance)
        msg.header.frame_id = "map"
        msg.child_frame_id = "odom"
        msg.transform.translation.x = tran[0]
        msg.transform.translation.y = tran[1]
        msg.transform.translation.z = tran[2]

        msg.transform.rotation.x = rot[0]
        msg.transform.rotation.y = rot[1]
        msg.transform.rotation.z = rot[2]
        msg.transform.rotation.w = rot[3]

        return msg

    def extract_odom_msg(self, odom_msg):
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
            pose.orientation.w,
        )

        x = pose.position.x
        y = pose.position.y

        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]

        return [x, y, yaw]

    def extract_scan_msg(self, scan_msg):
        """Extracts distance readings from the laser scan.

        ``inf`` readings are substituted by ``None``.

        Args:
            scan_msg (:obj:`sensor_msgs.msg.LaserScan`)

        Returns:
            :obj:`list`: List, containing the ranges for each angle.

        """
        ranges = [
            None if elem == float("inf") else round(elem, 3) for elem in scan_msg.ranges
        ]

        return ranges

    def process_scan(self, scan):
        """Appends angles to the range-only scan readings.

        Assumes that every range reading is evenly distributed accross 360 degs.
        Ranges with value ``None`` are excluded together with their angle.

        Args:
            scan (:obj:`list`): Range readings.

        Returns:
            :obj:`list of (angle,range)`

        """
        angles = np.linspace(0, 2 * np.pi, len(scan), endpoint=False)
        angle_range = [
            (angle, range) for angle, range in zip(angles, scan) if range is not None
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
            rospy.loginfo("Map received")
            return map_response
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s" % e)
            return None

    def get_map(self):
        """Creates the ``GridMap`` for the localization.

        Returns:
            :obj:`dhflocalization.gridmap.GridMap`: The internal object for handling the OGM.

        """
        map_response = self.get_map_from_srv()
        map_message = map_response.map
        map_data = np.asarray(map_message.data)

        width = map_message.info.width
        height = map_message.info.height
        resolution = map_message.info.resolution
        map_array = map_data.reshape(height, width)
        map_array = self.convert_occupancy_representation(map_array)

        center_x = 10  # TODO
        center_y = 10.05  # TODO

        occupancy_grid_map = GridMap.load_grid_map_from_array(
            np.flip(map_array, 0), resolution, center_x, center_y
        )

        return occupancy_grid_map

    def convert_occupancy_representation(self, map_array):
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
        return np.where(map_array < max_val, 0, 1)

    def broadcast_pose(self, pose):
        """Broadcast the transormation between the map and the robot.

        Uses the ``map`` frame as a base and the ``base_footprint`` as the child.

        Args:
            pose (:obj:`(3,1) np.ndarray`): Array containing the x,y poses and the yaw angle.
        """
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = pose[0]
        transform.transform.translation.y = pose[1]
        transform.transform.translation.z = 0.0

        quat = quaternion_from_euler(0, 0, pose[2])
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
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
        measurement_model = MeasurementModel(
            self.gridmap, cfg_measurement_range_noise_std
        )

        cfg_edh_particle_number = 100
        cfg_edh_lambda_number = 10
        cfg_init_gaussian_mean = np.array([-3.0, 1.0, 0])
        cfg_init_gaussian_covar = np.array(
            [[0.1 ** 2, 0, 0], [0, 0.1 ** 2, 0], [0, 0, 0.05 ** 2]]
        )

        self.measurement_processer = MeasurementProcessor(
            max_ray_number=cfg_max_ray_number
        )
        self.ekf = EKF(motion_model, measurement_model)
        self.edh = EDH(
            motion_model=motion_model,
            measurement_model=measurement_model,
            particle_num=cfg_edh_particle_number,
            lambda_num=cfg_edh_lambda_number,
        )

        self.prior = ParticleState.init_from_gaussian(
            cfg_init_gaussian_mean, cfg_init_gaussian_covar, cfg_edh_particle_number
        )
        self.ekf_prior = StateHypothesis(
            state_vector=cfg_init_gaussian_mean, covar=cfg_init_gaussian_covar
        )


if __name__ == "__main__":
    dhf_localization_node = DhfLocalizationNode()
    rospy.spin()
