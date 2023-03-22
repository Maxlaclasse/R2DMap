#!/usr/bin/env python

import rospy
import tf2_ros
import tf.transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from sensor_msgs.msg import JointState
import math

class EncoderOdometry:
    def __init__(self):
        rospy.init_node('encoder_odometry_node')
        self.wheel_separation = 0.25
        self.wheel_radius = 0.065 / 2.0

        self.x = 0
        self.y = 0
        self.theta = 0
        self.prev_left_wheel_position = None
        self.prev_right_wheel_position = None

        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

    def joint_state_callback(self, joint_state):
        if len(joint_state.name) != 2:
            rospy.logerr("Joint state message should contain two joint states")
            return

        left_wheel_index = joint_state.name.index('left_wheel_joint')
        right_wheel_index = joint_state.name.index('right_wheel_joint')

        left_wheel_position = joint_state.position[left_wheel_index]
        right_wheel_position = joint_state.position[right_wheel_index]

        if self.prev_left_wheel_position is None or self.prev_right_wheel_position is None:
            self.prev_left_wheel_position = left_wheel_position
            self.prev_right_wheel_position = right_wheel_position
            return

        delta_left_wheel = left_wheel_position - self.prev_left_wheel_position
        delta_right_wheel = right_wheel_position - self.prev_right_wheel_position

        self.prev_left_wheel_position = left_wheel_position
        self.prev_right_wheel_position = right_wheel_position

        # Compute odometry using joint positions
        delta_left = self.wheel_radius * delta_left_wheel
        delta_right = self.wheel_radius * delta_right_wheel
        delta_theta = (delta_right - delta_left) / self.wheel_separation
        delta_x = (delta_right + delta_left) / 2.0
        delta_y = 0.0

        self.x += delta_x * math.cos(self.theta) - delta_y * math.sin(self.theta)
        self.y += delta_x * math.sin(self.theta) + delta_y * math.cos(self.theta)
        self.theta += delta_theta

        # Create and publish odometry message
        odom = Odometry()
        odom.header.stamp = joint_state.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        quaternion_msg = Quaternion(*quaternion)
        odom.pose.pose.orientation = quaternion_msg

        self.odom_pub.publish(odom)

        # Broadcast transform
        tf_transform = TransformStamped()
        tf_transform.header.stamp = joint_state.header.stamp
        tf_transform.header.frame_id = 'odom'
        tf_transform.child_frame_id = 'base_link'

        tf_transform.transform.translation.x = self.x
        tf_transform.transform.translation.y = self.y
        tf_transform.transform.rotation = quaternion_msg

        self.tf_broadcaster.sendTransform(tf_transform)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    encoder_odometry = EncoderOdometry()
    encoder_odometry.run()
