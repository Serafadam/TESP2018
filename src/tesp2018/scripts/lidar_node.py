#!/usr/bin/env python
import rospy
import numpy
from sensor_msgs.msg import LaserScan


class LidarNode(object):
    def __init__(self):
        rospy.init_node('tesp',disable_signals=True)
        self.laser = None
        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.data_counter = 0
        self.start_time = 0

    def lidar_callback(self, data):
        self.laser = data
        print self.laser
        self.data_saving(self.laser)

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
