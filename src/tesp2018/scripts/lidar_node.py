#!/usr/bin/env python
import rospy
import numpy
from sensor_msgs.msg import LaserScan
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import Float64MultiArray


class LidarNode(object):
    def __init__(self):
        rospy.init_node('tesp',disable_signals=True)

        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.data_counter = 0
        self.start_time = 0
        self.pub = rospy.Publisher('/lidar_output', numpy_msg(Floats),queue_size=10)
    def lidar_callback(self, data):
        #print len(data.ranges)
        laser = [data.ranges[0],data.ranges[45], data.ranges[90],data.ranges[135], data.ranges[179], data.ranges[-45], data.ranges[-90], data.ranges[-135]]
        #print type(laser)

        laser_ros = numpy.array(laser)
        laser_ros.layout.dim=[8]
        laser_ros.data = laser
        print laser_ros
        self.pub.publish(laser_ros)
        #self.data_saving(self.laser)

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
