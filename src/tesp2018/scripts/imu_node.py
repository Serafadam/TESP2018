#!/usr/bin/env python
import rospy
import numpy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3


class ImuNode(object):
    def __init__(self):
        rospy.init_node('tesp')
        self.linear_acceleration_data = Vector3()
        self.acceleration_subscriber = rospy.Subscriber('/imu', Imu, self.imu_callback)

    def imu_callback(self, data):
        self.linear_acceleration_data = data.linear_acceleration
        print self.acceleration_data


if __name__ == '__main__':
    imu = ImuNode()
    rospy.spin()
