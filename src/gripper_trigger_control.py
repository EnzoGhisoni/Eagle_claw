#!/usr/bin/env python

import rospy
import math
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, PoseStamped

class JointPub(object):
	def __init__(self):

		self.publishers_array = []
		self.joint_6_pub = rospy.Publisher('/mrm/joint6_position_controller/command', Float64, queue_size=1)
		self.joint_7_pub = rospy.Publisher('/mrm/joint7_position_controller/command', Float64, queue_size=1)
		self.joint_8_pub = rospy.Publisher('/mrm/joint8_position_controller/command', Float64, queue_size=1)
		self.joint_9_pub = rospy.Publisher('/mrm/joint9_position_controller/command', Float64, queue_size=1)
		self.joint_10_pub = rospy.Publisher('/mrm/joint10_position_controller/command', Float64, queue_size=1)
		self.joint_11_pub = rospy.Publisher('/mrm/joint11_position_controller/command', Float64, queue_size=1)

		self.publishers_array.append(self.joint_6_pub)
		self.publishers_array.append(self.joint_7_pub)
		self.publishers_array.append(self.joint_8_pub)
		self.publishers_array.append(self.joint_9_pub)
		self.publishers_array.append(self.joint_10_pub)
		self.publishers_array.append(self.joint_11_pub)
		#self.pose_sub = rospy.Subscriber('/rightControlerPose1', Pose, self.rightControlerPoseCallback)
		self.pose_sub = rospy.Subscriber('/rightControlerTrigger', Float64, self.rightControlerTriggerCallback)
		self.init_pos = [0.0,0.0,0.0,0.0,0.0,0.0]
		self.actual_joint_pose = [0.0,0.0,0.0,0.0,0.0,0.0]
		self.closingFactor = 0.0
		
	def set_init_pose(self):
		"""
		Sets joints to initial position [0,0,0,0,0,0]
		:return:
		"""
		self.check_publishers_connection()
		self.move_joints(self.init_pos)


	def check_publishers_connection(self):
		"""
		Checks that all the publishers are working
		:return:
		"""
		rate = rospy.Rate(10)  # 10hz
		while (self.joint_6_pub.get_num_connections() == 0):
			rospy.logdebug("No susbribers to joint_6_pub yet so we wait and try again")
			try:
				rate.sleep()
			except rospy.ROSInterruptException:
				# This is to avoid error when world is rested, time when backwards.
				pass
		rospy.logdebug("joint_6_pub Publisher Connected")

		while (self.joint_7_pub.get_num_connections() == 0):
			rospy.logdebug("No susbribers to joint_7_pub yet so we wait and try again")
			try:
				rate.sleep()
			except rospy.ROSInterruptException:
				# This is to avoid error when world is rested, time when backwards.
				pass
		rospy.logdebug("joint_7_pub Publisher Connected")

		while (self.joint_8_pub.get_num_connections() == 0):
			rospy.logdebug("No susbribers to joint_8_pub yet so we wait and try again")
			try:
				rate.sleep()
			except rospy.ROSInterruptException:
				# This is to avoid error when world is rested, time when backwards.
				pass
		rospy.logdebug("joint_8_pub Publisher Connected")

		while (self.joint_9_pub.get_num_connections() == 0):
			rospy.logdebug("No susbribers to joint_9_pub yet so we wait and try again")
			try:
				rate.sleep()
			except rospy.ROSInterruptException:
				# This is to avoid error when world is rested, time when backwards.
				pass
		rospy.logdebug("joint_9_pub Publisher Connected")

		while (self.joint_10_pub.get_num_connections() == 0):
			rospy.logdebug("No susbribers to joint_10_pub yet so we wait and try again")
			try:
				rate.sleep()
			except rospy.ROSInterruptException:
				# This is to avoid error when world is rested, time when backwards.
				pass
		rospy.logdebug("joint_10_pub Publisher Connected")

		while (self.joint_11_pub.get_num_connections() == 0):
			rospy.logdebug("No susbribers to joint_11_pub yet so we wait and try again")
			try:
				rate.sleep()
			except rospy.ROSInterruptException:
				# This is to avoid error when world is rested, time when backwards.
				pass
		rospy.logdebug("joint_11_pub Publisher Connected")

		rospy.logdebug("All Publishers READY")

	def joint_mono_des_callback(self, msg):
		rospy.logdebug(str(msg.joint_state.position))

		self.move_joints(msg.joint_state.position)

	def move_joints(self, joints_array):

		i = 0
		for publisher_object in self.publishers_array:
		  joint_value = Float64()
		  joint_value.data = joints_array[i]
		  rospy.logdebug("JointsPos>>"+str(joint_value))
		  publisher_object.publish(joint_value)
		  i += 1

	def closingGripper(self, closingFactor):
		
		"""
		theta6 (bone left), theta7 (talon left), theta8 (bone right),
		theta9 (talon right), theta10 (bone top), theta11 (talon top)
		
		To control the gripper, the pose of every bone and talon will be symetric
		and the closing will be proportionnal to a closing factor (between 0 and 1)
		"""
		theta6 = theta7 = theta8 = theta9 = theta10 = theta11 = closingFactor
		return theta6, theta7, theta8, theta9, theta10, theta11
	

	def rightControlerTriggerCallback(self, ros_msg):
		self.closingFactor = ros_msg.data
		
	
		theta6, theta7, theta8, theta9, theta10, theta11 = self.closingGripper(ros_msg.data)
		print("angles have been calculated")
		self.actual_joint_pose = [theta6, theta7, theta8, theta9, theta10, theta11]
		
		print(str(self.actual_joint_pose))
		self.move_joints(self.actual_joint_pose)
	
if __name__=="__main__":
	rospy.init_node('gripper_joint_publisher_node')
	joint_publisher = JointPub()
	try:
		rospy.spin()
	except rospy.KeyboardInterrupt:
		print("Shutting down")
	
