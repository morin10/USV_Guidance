#!/usr/bin/env python3

import rospy

import math
import time
from std_msgs.msg import Float64 as joint_command
from gazebo_msgs.msg import ModelStates

from sensor_msgs.msg import NavSatFix
from udp_uv.msg import wplist
from udp_uv.msg import device
from tf.transformations	import euler_from_quaternion
from geometry_msgs.msg import TwistWithCovarianceStamped
import numpy as np


class Heron_PN_Guidance():
	def __init__(self):
		self.wp_flag = 0
		self.fin_flag = 0
		self.wp_id = -1
		self.first_iter_flag = 1
		self.prv_x = 0
		self.prv_y = 0
		self.prv_psi = 0

		self.wp_num = 0

		self.acc = 0.2

		self.output_max = 20

		self.vel_msg_left = joint_command()
		self.vel_msg_right = joint_command()
		self.m_dX = 0
		self.m_dY = 0
		self.m_dYaw = 0 #heading

		self.wp_cnt = 0
		self.wp_flag = 0

		self.os_x = 0
		self.os_y = 0

		self.os_v_x = 0.0
		self.os_v_y = 0.0

		self.gps_prev_time = 0.0
		self.gps_curr_time = 0.0

		self.mode = 0

		rospy.Subscriber('/wplist', wplist, self._rcv_wp_heron)
		rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback_gps)

		self.pub_left_wheel = rospy.Publisher('turtlebot3_waffle_sim/left_wheel_velocity_controller/command', joint_command, queue_size=10)
		self.pub_right_wheel = rospy.Publisher('turtlebot3_waffle_sim/right_wheel_velocity_controller/command', joint_command, queue_size=10)


	def callback_gps(self,msg):
		# get gps(UTM) x, y, v_x, v_y
		self.os_x = msg.pose[1].position.x
		self.os_y = msg.pose[1].position.y

		self.os_v_x = msg.twist[1].linear.x
		self.os_v_y = msg.twist[1].linear.y


		# #########TODO#########
		self.gps_curr_time = time.time()
		self.pn_guidance()
		#########TODO#########


	def pn_guidance(self):
		# next_way_point_x = 10
		# next_way_point_y = 10

		next_way_point_x = self.wp_list_x[self.wp_cnt]
		next_way_point_y = self.wp_list_y[self.wp_cnt]

		cur_x = self.os_x  
		cur_y = self.os_y
		cur_v_x = self.os_v_x
		cur_v_y = self.os_v_y

		dt = (self.gps_curr_time - self.gps_prev_time) * (10**-6)

		direction_os_to_wp = math.atan2(next_way_point_y-cur_y, next_way_point_x-cur_x)
		print(direction_os_to_wp)


		direction_v = math.atan2(cur_v_y, cur_v_x)
		print(direction_v)

		line_of_sight = direction_os_to_wp - direction_v

		if line_of_sight >= math.pi:
			line_of_sight = line_of_sight - math.pi
		elif line_of_sight <= -math.pi:
			line_of_sight = line_of_sight + math.pi



		k = 0.000000003
		delta = -k * line_of_sight/dt


		self.vel_msg_left.data = 10 + delta
		self.vel_msg_right.data = 10 - delta

		print("line_of_sight : " , line_of_sight)
		print("left : " , self.vel_msg_left.data)
		print("right : " , self.vel_msg_right.data)


		self.pub_left_wheel.publish(self.vel_msg_left)
		self.pub_right_wheel.publish(self.vel_msg_right)


		self.gps_prev_time = self.gps_curr_time


		distance_os_to_wp = math.sqrt((next_way_point_y-cur_y)**2 + (next_way_point_x-cur_x)**2)

		if (distance_os_to_wp <= 0.2):
			if (self.wp_cnt == self.wp_num - 1):
				self.fin_flag = 1
				self.wp_cnt = 0
				return

			if (self.fin_flag == 0):
				self.wp_cnt = self.wp_cnt + 1




# -------------------- cal_con2 -------------------------------------------------------		
	def _rcv_wp_heron(self,msg):
		if self.wp_id is not msg.id:
			self.wp_cnt = 0
			self.wp_list_x = msg.x
			self.wp_list_y = msg.y
			self.wp_num = msg.size
			self.wp_id = msg.id
			self.wp_flag = 1;

	

	def run(self):

		rospy.spin()

if __name__ == '__main__':
	try:
		rospy.init_node('con_heron_node', anonymous=True)

		heron_node = Heron_PN_Guidance()
		heron_node.run()

	except rospy.ROSInterruptException:
		pass
