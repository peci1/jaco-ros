#! /usr/bin/env python
# coding=utf-8

__author__ = 'Bc. Matěj Balga'

import roslib
import rospy
import math

roslib.load_manifest('jaco_moveit')

from jaco_msgs.msg import (
    FingerPosition
)

from sensor_msgs.msg import JointState


class JacoStateMux:
    def __init__(self):
        self.current_joints_position = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.current_finger_position = (0.0, 0.0, 0.0)

        """Initialize ROS subscribers"""
        self.subscriber_fingers = rospy.Subscriber('/jaco_arm_driver/out/finger_position',
                                                   FingerPosition,
                                                   self.finger_position_update,
                                                   queue_size=1)

        self.subscriber_joints = rospy.Subscriber('/jaco_arm_driver/out/joint_state',
                                                  JointState,
                                                  self.joint_state_update,
                                                  queue_size=1)

        '''Initialize ROS publishers'''
        self.jaco_state_publisher = rospy.Publisher('/joint_states',
                                                    JointState,
                                                    queue_size=1)

    def finger_position_update(self, finger_position):
        f = (finger_position.finger1,
             finger_position.finger2,
             finger_position.finger3)
        deg2rad = 1.0 / 180.0 * math.pi
        self.current_finger_position = tuple([deg2rad * i for i in f])

    def joint_state_update(self, joint_state):
        self.current_joints_position = joint_state.position

    def mux(self):
        repeater = rospy.Rate(10)

        rospy.sleep(1)  # Wait for msgs to initialize default values

        while not rospy.is_shutdown():
            new_state = JointState()

            new_state.header.stamp = rospy.Time.now()

            new_state.name = ('jaco_joint_1',
                              'jaco_joint_2',
                              'jaco_joint_3',
                              'jaco_joint_4',
                              'jaco_joint_5',
                              'jaco_joint_6',
                              'jaco_joint_finger_1',
                              'jaco_joint_finger_2',
                              'jaco_joint_finger_3',)

            new_state.position = self.current_joints_position + self.current_finger_position

            self.jaco_state_publisher.publish(new_state)

            repeater.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('joint_state_mux')
        jaco_state_mux = JacoStateMux()
        jaco_state_mux.mux()

    except rospy.ROSInterruptException:
        pass

