#!/usr/bin/python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

wp_pub = rospy.Publisher('/heron_info/waypoints', Path, queue_size=1)
waypoints = [[5, 7], [-3, 2], [-5, -4], [3, -3]]

if __name__=='__main__':
    rospy.init_node('wp_pub')
    path = Path()

        # print(self.waypoints)
    for wp in waypoints:
        stamp = PoseStamped()
        # print(wp)
        stamp.pose.position.x = wp[0]
        stamp.pose.position.y = wp[1]

        path.poses.append(stamp)

    wp_pub.publish(path)


