#!/usr/bin/env python
import rospy
import numpy
from sensor_msgs.msg import LaserScan
from pythonosc import udp_client

pd_ip = "172.16.20.57"
pd_port = 11112
client = udp_client.SimpleUDPClient(pd_ip, pd_port)


class LidarNode(object):
    def __init__(self):
        rospy.init_node('tesp',disable_signals=True)

        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.data_counter = 0
        self.start_time = 0
        #self.pub = rospy.Publisher('/lidar_output', numpy_msg(Floats),queue_size=10)
    def lidar_callback(self, data):
        #print len(data.ranges)
        laser = [data.ranges[0],data.ranges[45], data.ranges[90],data.ranges[135], data.ranges[179], data.ranges[-45], data.ranges[-90], data.ranges[-135]]

        self.shake_it_baby(laser)


    def shake_it_baby(self, lasers):

        if lasers[6] > 0:
            right_amp = 0.1 / abs(lasers[6])
            left_amp = 0.001
        if lasers[2] > 0:
            right_amp  = 0.001
            left_amp = 0.1 / abs(lasers[2])

        if lasers[0] > 0:
            front_amp =0.1 / abs(lasers[0])
            back_amp = 0.001
        if lasers[4]:
            front_amp = 0.001
            back_amp = 0.1 / abs(lasers[4])

        client.send_message("/front", front_amp)
        client.send_message("/back", back_amp)
        client.send_message("/left", left_amp)
        client.send_message("/right", right_amp)
        print(front_amp, back_amp, left_amp, right_amp)


    def data_saving(self,data):
        if self.data_counter == 0:
            self.start_time = rospy.get_time()
        test_file = open('robot_lidar.txt', 'a+')
        test_file.write(str(data))
        self.data_counter += 1
        print self.data_counter
        if self.data_counter == 250:
            test_file.close()
            print self.start_time
            print rospy.get_time()
            print 'end'
            rospy.signal_shutdown('lol')


if __name__ == '__main__':

    lidar = LidarNode()
    rospy.spin()
