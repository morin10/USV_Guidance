#!/usr/bin/python3
import rospy
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import NavSatFix, Imu
from tf.transformations	import euler_from_quaternion

gps_pub = rospy.Publisher('ublox_gps/fix', NavSatFix, queue_size=1)
gps_msg = NavSatFix()

def callback_heading(msg):
    gps_msg.latitude = msg.pose[1].position.x
    gps_msg.longitude = msg.pose[1].position.y
    gps_msg.altitude = msg.pose[1].position.z


if __name__=='__main__':
    rospy.init_node('gps_pub')
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rospy.Subscriber('/gazebo/model_states', ModelStates, callback_heading)
        gps_pub.publish(gps_msg)
        rate.sleep()


