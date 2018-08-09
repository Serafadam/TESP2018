#!/usr/bin/env python
from __future__ import absolute_import
import rospy
import numpy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
#from tf.transformations import euler_from_quaternion
from quaternion import Quaternion

from pythonosc import osc_message_builder
from pythonosc import udp_client

pd_ip = "172.16.20.57"
pd_port = 11112
client = udp_client.SimpleUDPClient(pd_ip, pd_port)

class ImuNode(object):
    def __init__(self):

        rospy.init_node('tesp')
        self.linear_acceleration_data = Vector3()
        self.acceleration_subscriber = rospy.Subscriber('/imu_throttle', Imu, self.imu_callback)

    def imu_callback(self, data):
        self.linear_acceleration_data = data.linear_acceleration
        #print self.acceleration_data

        orientation_q = data.orientation
        q = Quaternion(orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z)
        (yaw, roll, pitch) = q.yaw_pitch_roll
        self.write_vibrators(roll, pitch, yaw)
        #print(roll, pitch, yaw)

    def write_vibrators(self, roll, pitch, yaw):
        
        global client
        
        if roll<0:
            right_amp = 4* abs(roll)
            left_amp = 0.001
        else:
            right_amp = 0.001
            left_amp = 4* abs(roll)

        if pitch>=0:
            front_amp = 4* abs(pitch)
            back_amp = 0.001
        else:
            front_amp = 0.001
            back_amp = 4* abs(pitch)
        
        client.send_message("/front", front_amp)
        client.send_message("/back", back_amp)
        client.send_message("/left", left_amp)
        client.send_message("/right", right_amp)
        print(front_amp, back_amp, left_amp, right_amp)






if __name__ == '__main__':
    imu = ImuNode()
    rospy.spin()
