#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16
from sensor_msgs.msg import Imu

def send():
	pub = rospy.Publisher('number', Imu, queue_size=10)
	IMU = Imu()
	'''IMU.orientation_covariance = [0,1,2,3,4,5,6,7,8]
	IMU.angular_velocity_covariance = [0,1,2,3,4,5,6,7,8]
	IMU.linear_acceleration_covariance = [0,1,2,3,4,5,6,7,8]'''


	rate = rospy.Rate(10)
	rospy.init_node('tester', anonymous=True)

	while not rospy.is_shutdown():
    	#pub.publish(rumble)
    	pub.publish(IMU)
    	rate.sleep() 




if __name__ == '__main__':
    try:
    	send()
    except rospy.ROSInterruptException:
        pass