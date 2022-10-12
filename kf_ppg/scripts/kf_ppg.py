#!/usr/bin/env python3
import rospy
import message_filters
from nav_msgs.msg import Path
from std_msgs.msg import Int64
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64 as joint_command
from sensor_msgs.msg import NavSatFix
from tf.transformations	import euler_from_quaternion
from kingfisher_msgs.msg import Drive
import numpy as np

class KF_PPG(object):

	def __init__(self):
		# gain tuning
		self.g_rho = rospy.get_param("/gain_rho")
		self.g_psi_se = rospy.get_param("/gain_psi_small")
		self.g_psi_be = rospy.get_param("/gain_psi_large")
		self.g_psi = self.g_psi_se
		self.scale = 10

		# limit tuning
		self.max_v = rospy.get_param("/max_v")
		self.rho_th = rospy.get_param("/rho_threshold")
		self.thrust_max = rospy.get_param("/max_thrust")
		self.acpt_ratio = rospy.get_param("/acceptance_ratio")

		# wp init
		self.wp_list = []
		self.wp_cnt = 0

		# flag
		self.start_flag = False
		self.fin_flag = False

		# turtulebot msgs
		# self.v_left = joint_command()
		# self.v_right = joint_command()

		# Kingfisher msgs
		self.v_msg = Drive()

		# publisher
		# self.pub_left = rospy.Publisher('turtlebot3_waffle_sim/left_wheel_velocity_controller/command', joint_command, queue_size=10)
		# self.pub_right = rospy.Publisher('turtlebot3_waffle_sim/right_wheel_velocity_controller/command', joint_command, queue_size=10)
		self.pub_thrust = rospy.Publisher('/cmd_drive', Drive, queue_size=10)

		# subscrisber
		rospy.Subscriber('/heron_info/waypoints', Path, self.wp_cb)
		self.imu_sub = message_filters.Subscriber('/imu/data', Imu)
		self.gps_sub = message_filters.Subscriber('/ublox_gps/fix', NavSatFix)
		ts = message_filters.TimeSynchronizer([self.imu_sub, self.gps_sub], 10)
		ts.registerCallback(self.sensor_cb)

	def wp_cb(self, msg):
		wp_list_temp = []
		for pose in msg.poses:
			wp_x = pose.pose.position.x
			wp_y = pose.pose.position.y
			wp_list_temp.append([wp_x, wp_y])
		self.wp_list = wp_list_temp
		self.start_flag = True
			
	def sensor_cb(self,imu_msg, gps_msg):
		if not self.start_flag:
			print('Waiting waypoint...')
			return
		
		if self.fin_flag:
			print("Successfully done!!")
			# turtlebot
			# self.v_left.data = 0
			# self.v_right.data = 0
			# self.pub_left.publish(self.v_left)
			# self.pub_right.publish(self.v_right)
			# kingfhisher
			self.v_msg.left = 0
			self.v_msg.right = 0
			self.pub_thrust.publish(self.v_msg)
			return

		_, _, curr_psi = euler_from_quaternion([ imu_msg.orientation.x, imu_msg.orientation.y,\
							imu_msg.orientation.z, imu_msg.orientation.w ])
		# gazebo
		# curr_x = gps_msg.latitude
		# curr_y = gps_msg.longitude

		# Real GPS data
		curr_x = gps_msg.latitude
		curr_y = gps_msg.longitude
		curr_x, curr_y = self.lla2utm(curr_x, curr_y)

		dsrd_x, dsrd_y = self.wp_list[self.wp_cnt]
		x_err = dsrd_x - curr_x
		y_err = dsrd_y - curr_y
		rho_err = np.sqrt(np.square(x_err) + np.square(y_err))

		if rho_err < self.acpt_ratio:
			self.wp_cnt += 1
			if self.wp_cnt >= len(self.wp_list):
				self.fin_flag = True
		if not self.fin_flag:
			self.control_thrust(curr_psi, x_err, y_err)


	def lla2utm(self, curr_x, curr_y):
		dLatitude = curr_x
		dLongitude = curr_y

		dLat = dLatitude * np.pi/180
		dLon = dLongitude * np.pi/180

		lon0_f = np.floor(dLongitude/6)*6+3
		lon0 = lon0_f*np.pi/180
		k0 = 0.9996
		FE = 500000
		FN = (dLatitude < 0)*10000000

		Wa = 6378137
		We = 0.081819190842965
		WN = Wa/np.sqrt( 1 - np.power(We,2)*np.power(np.sin(dLat),2))
		WT = np.power(np.tan(dLat), 2)
		WC = (np.power(We,2)/(1-np.power(We,2)))*np.power(np.cos(dLat),2)
		WLA = (dLon - lon0)*np.cos(dLat)
		WM = (Wa*((1 - np.power(We,2)/4 - 3*np.power(We,4)/64 - 5*np.power(We,6)/256)*dLat - (3*np.power(We,2)/8 + 3*np.power(We,4)/32 + 45*np.power(We,6)/1024)*np.sin(2*dLat) + (15*np.power(We,4)/256 + 45*np.power(We,6)/1024)*np.sin(4*dLat) - (35*np.power(We,6)/3072)*np.sin(6*dLat)) )

		Weps = 0.006739496742333
		# Easting
		m_dUTM_X = (FE + k0*WN*(WLA + (1 - WT + WC)*np.power(WLA,3)/6	+ (5 - 18*WT + np.power(WT,2) + 72*WC - 58*Weps)*pow(WLA,5)/120))
		# Northing
		m_dUTM_Y = (FN + k0*WM + k0*WN*np.tan(dLat)*(np.power(WLA,2)/2 + (5 - WT + 9*WC + 4*np.power(WC,2))*np.power(WLA,4)/24 + (61 - 58*WT + np.power(WT,2) + 600*WC - 330*Weps)*np.power(WLA,6)/720))
		# Zone
		m_nUTM_Zone = np.int(np.floor(lon0_f/6)+31)

		return m_dUTM_X, m_dUTM_Y

	def control_thrust(self, curr_psi, x_err, y_err):

		rho_err = np.sqrt( np.square(x_err) + np.square(y_err) )
		
		dsrd_psi = np.arctan2( y_err, x_err )
		psi_err = dsrd_psi - curr_psi
		if psi_err >= np.pi:
			while psi_err >= np.pi:
				psi_err = psi_err - 2*np.pi
		elif psi_err < -np.pi:
			while psi_err < -np.pi:
				psi_err = psi_err + 2*np.pi

		v = self.g_rho * rho_err

		# default v clipping with max_v
		if v > self.max_v:
			v = self.max_v
		elif v < 0:
			v = 0

		# gain setting
		if np.abs(psi_err) <= self.rho_th:	
			self.g_psi = self.g_psi_se
		elif np.abs(psi_err) > self.rho_th:
			self.g_psi = self.g_psi_be
		
		w = -self.g_psi*psi_err

		left = v + 0.5*w
		right = v - 0.5*w


		# clipping thruster input with maximum value
		if ((np.absolute(left) > self.thrust_max) or (np.absolute(right) > self.thrust_max)):	
			ratio = self.thrust_max / np.maximum(np.absolute(left), np.absolute(right))
			left = left*ratio
			right = right*ratio

		# turtlebot
		# if left < 0.4:
		# 	left = 0.4
		# if right < 0.4:
		# 	right = 0.4

		# kingfisher
		if left < 0:
			left = 0
		if right < 0:
			right = 0

		print('=================================================')
		print('wp_cnt          : %d ' % (self.wp_cnt))
		print('distance err    : %.2f   [m]' % (rho_err))
		print('heading err 	   : %.2f   [deg] ' % (psi_err*(180.0/np.pi)))
		print('heading         : %.2f   [deg]' % (curr_psi*(180.0/np.pi)))
		print('desired heading : %.2f   [deg] ' % (dsrd_psi*(180.0/np.pi)))
		print('left thrst      : %.2f ' % (left))
		print('right thrst     : %.2f ' % (right))
		print('=================================================')


		# turtlebot
		# self.v_left.data = self.scale*left
		# self.v_right.data = self.scale*right
		# self.pub_left.publish(self.v_left)
		# self.pub_right.publish(self.v_right)
		# kingfisher
		self.v_msg.left = left
		self.v_msg.right = right
		self.pub_thrust(self.v_msg)

	def main(self):
		rospy.spin()

if __name__ == '__main__':
	try:
		rospy.init_node('kf_ppg', anonymous=True)
		my_node = KF_PPG()
		my_node.main()

	except rospy.ROSInterruptException:
		pass
