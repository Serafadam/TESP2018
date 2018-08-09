#!/usr/bin/env python
import rospy
import numpy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from tf import transformations
from audio_lib_only import Sine


class ImuNode(object):
    def __init__(self):
        rospy.init_node('tesp',disable_signals=True)
        self.orientation_subscriber = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.vibrator_front_left = Sine(110, 0)
        self.vibrator_front_right = Sine(110, 0)
        self.vibrator_back_left = Sine(110, 0)
        self.vibrator_back_right = Sine(110, 0)
        self.data_counter = 0
        self.start_time = 0

    def imu_callback(self, data):
        orientation = None
        orientation = transformations.euler_from_quaternion(
            [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        #print orientation
        #self.data_saving(orientation)
        self.vibrate(orientation)

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

    def vibrate(self,orientation):
        self.vibrator_front_right.amplitude = 0
        self.vibrator_back_right.amplitude = 0
        self.vibrator_front_left.amplitude = 0
        self.vibrator_back_left.amplitude = 0
        if orientation[1] > 0:
            self.vibrator_front_left.amplitude = orientation[1]*100
            self.vibrator_front_right.amplitude = orientation[1]*100
        if orientation[1] < 0:
            self.vibrator_back_left.amplitude = abs(orientation[1])*100
            self.vibrator_back_right.amplitude = abs(orientation[1])*100
        if orientation[0] > 0:
            self.vibrator_front_right.amplitude = orientation[0]*100
            self.vibrator_back_right.amplitude = orientation[0]*100
        if orientation[0] < 0:
            self.vibrator_front_left.amplitude = abs(orientation[0])*100
            self.vibrator_back_left.amplitude = abs(orientation[0])*100

        final_vibration = self.vibrator_front_left + self.vibrator_back_left + self.vibrator_back_right + self.vibrator_front_right
        final_vibration.play(0.5,blocking=False,device=1)
        print
        print orientation[0]
        print orientation[1]
        print orientation[2]



if __name__ == '__main__':

    imu = ImuNode()
    rospy.spin()
