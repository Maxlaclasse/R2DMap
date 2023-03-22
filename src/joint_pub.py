#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

rospy.init_node('joint_state_publisher')

joint_names = ['right_wheel_joint', 'left_wheel_joint']
positions = [0, 0]
velocities = []
efforts = []

pub = rospy.Publisher('joint_states', JointState, queue_size=10)

rate = rospy.Rate(10)  # 1 Hz
seq = 0
while not rospy.is_shutdown():
    header = Header(seq=seq, stamp=rospy.Time.now(), frame_id='')
    joint_state = JointState(header=header, name=joint_names, position=positions,
                             velocity=velocities, effort=efforts)
    pub.publish(joint_state)
    seq += 1
    positions[0]=positions[0]+0.1
    positions[1]=positions[1]+0.1
    rate.sleep()
