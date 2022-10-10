#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from rospy.timer import Rate
import tf
import json

topicdata = []
eps = 0.1


def callback_odom(data):
    pose = data.pose.pose
    timestamp = data.header.stamp.to_sec()
    quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )

    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]
    if not len(topicdata) or abs((timestamp - topicdata[-1]["t"]) > eps):
        topicdata.append(
            {
                "t": timestamp,
                "pose": [
                    round(pose.position.x, 3),
                    round(pose.position.y, 3),
                    round(yaw, 3),
                ],
                "truth": [],
                "scan": [],
                "amcl": [],
                "amcl_covar": None,
            }
        )
    else:
        topicdata[-1]["pose"] = [
            round(pose.position.x, 3),
            round(pose.position.y, 3),
            round(yaw, 3),
        ]


def callback_truth(data):
    pose = data.pose.pose
    timestamp = data.header.stamp.to_sec()
    quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )

    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]
    if not len(topicdata) or abs((timestamp - topicdata[-1]["t"]) > eps):
        topicdata.append(
            {
                "t": timestamp,
                "pose": [],
                "truth": [
                    round(pose.position.x, 3),
                    round(pose.position.y, 3),
                    round(yaw, 3),
                ],
                "scan": [],
                "amcl": [],
                "amcl_covar": None,
            }
        )
    else:
        topicdata[-1]["truth"] = [
            round(pose.position.x, 3),
            round(pose.position.y, 3),
            round(yaw, 3),
        ]


def callback_scan(data):
    timestamp = data.header.stamp.to_sec()
    ranges = [None if elem == float("inf") else round(elem, 3) for elem in data.ranges]
    if not len(topicdata) or abs((timestamp - topicdata[-1]["t"]) > eps):
        topicdata.append(
            {
                "t": timestamp,
                "pose": [],
                "truth": [],
                "scan": ranges,
                "amcl": [],
                "amcl_covar": None,
            }
        )
    else:
        topicdata[-1]["scan"] = ranges


def callback_amcl(data):
    pose = data
    timestamp = data[2].to_sec()
    quaternion = (pose[1][0], pose[1][1], pose[1][2], pose[1][3])

    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]
    if not len(topicdata) or abs((timestamp - topicdata[-1]["t"]) > eps):
        topicdata.append(
            {
                "t": timestamp,
                "pose": [],
                "truth": [],
                "scan": [],
                "amcl": [round(pose[0][0], 3), round(pose[0][1], 3), round(yaw, 3)],
                "amcl_covar": None,
            }
        )
    else:
        topicdata[-1]["amcl"] = [
            round(pose[0][0], 3),
            round(pose[0][1], 3),
            round(yaw, 3),
        ]


def callback_amcl_covar(data):
    timestamp = data.header.stamp.to_sec()
    covar = data.pose.covariance
    if not len(topicdata) or abs((timestamp - topicdata[-1]["t"]) > eps):
        topicdata.append(
            {
                "t": timestamp,
                "pose": [],
                "truth": [],
                "scan": [],
                "amcl": [],
                "amcl_covar": covar,
            }
        )
    else:
        topicdata[-1]["amcl_covar"] = covar


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the preous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node("topicexporter", anonymous=True)
    rate = rospy.Rate(5)
    rospy.Subscriber("odom", Odometry, callback_odom)
    rospy.Subscriber("scan", LaserScan, callback_scan)
    rospy.Subscriber("ground_truth/state", Odometry, callback_truth)
    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, callback_amcl_covar)
    listenr = tf.TransformListener()
    listenr.waitForTransform(
        "/map", "/base_footprint", rospy.Time(), rospy.Duration(3.0)
    )
    # spin() simply keeps python from exiting until this node is stopped

    while not rospy.is_shutdown():
        try:
            t = listenr.getLatestCommonTime("/map", "/base_footprint")
            (trans, rot) = listenr.lookupTransform("/map", "/base_footprint", t)
            callback_amcl([trans, rot, t])

        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            continue
        rate.sleep()

    if rospy.is_shutdown():
        rospy.loginfo(topicdata)


if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        with open("topicexport.json", "w") as file:
            json.dump({"data": topicdata}, file)
