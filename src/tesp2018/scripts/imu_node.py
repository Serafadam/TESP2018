#!/usr/bin/env python
import rospy
import numpy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from tf import transformations


class ImuNode(object):
    def __init__(self):
        rospy.init_node('tesp',disable_signals=True)
        self.linear_acceleration_data = Vector3()
        self.acceleration_subscriber = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.data_counter = 0
        self.start_time = 0

    def imu_callback(self, data):
        self.orientation = transformations.euler_from_quaternion(
            [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        print self.orientation
        self.data_saving(self.orientation)

    def data_saving(self,data):
        if self.data_counter == 0:
            self.start_time = rospy.get_time()
        test_file = open('robot_tilt_angles.txt', 'a+')
        test_file.write(str(data))
        self.data_counter += 1
        print self.data_counter
        if self.data_counter == 4000:
            test_file.close()
            print self.start_time
            print rospy.get_time()
            print 'end'
            rospy.signal_shutdown('lol')




if __name__ == '__main__':

    imu = ImuNode()
    rospy.spin()
