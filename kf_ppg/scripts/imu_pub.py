#!/usr/bin/python3
import rospy
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import NavSatFix, Imu
from tf.transformations	import euler_from_quaternion

imu_pub = rospy.Publisher('imu/data', Imu, queue_size=1)

def callback_heading(msg):
    imu_msg = Imu()
    imu_msg.orientation = msg.pose[1].orientation
    imu_pub.publish(imu_msg)


if __name__=='__main__':
    rospy.init_node('imu_pub')
    rospy.Subscriber('/gazebo/model_states', ModelStates, callback_heading)
    rospy.spin()
