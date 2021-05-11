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
		self.joint_1_pub = rospy.Publisher('/mrm/joint1_position_controller/command', Float64, queue_size=1)
		self.joint_2_pub = rospy.Publisher('/mrm/joint2_position_controller/command', Float64, queue_size=1)
		self.joint_3_pub = rospy.Publisher('/mrm/joint3_position_controller/command', Float64, queue_size=1)
		self.joint_4_pub = rospy.Publisher('/mrm/joint4_position_controller/command', Float64, queue_size=1)
		self.joint_5_pub = rospy.Publisher('/mrm/joint5_position_controller/command', Float64, queue_size=1)

		self.publishers_array.append(self.joint_1_pub)
		self.publishers_array.append(self.joint_2_pub)
		self.publishers_array.append(self.joint_3_pub)
		self.publishers_array.append(self.joint_4_pub)
		self.publishers_array.append(self.joint_5_pub)
		#self.pose_sub = rospy.Subscriber('/rightControlerPose1', Pose, self.rightControlerPoseCallback)
		self.pose_sub = rospy.Subscriber('/rightControlerPose', PoseStamped, self.rightControlerPoseStampCallback)
		self.init_pos = [0.0,0.0,0.0,0.0]
		self.actual_joint_pose = [0.0,0.0,0.0,0.0]
		self.actual_controler_pose = Pose()
		
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
		while (self.joint_1_pub.get_num_connections() == 0):
			rospy.logdebug("No susbribers to joint_1_pub yet so we wait and try again")
			try:
				rate.sleep()
			except rospy.ROSInterruptException:
				# This is to avoid error when world is rested, time when backwards.
				pass
		rospy.logdebug("joint_1_pub Publisher Connected")

		while (self.joint_2_pub.get_num_connections() == 0):
			rospy.logdebug("No susbribers to joint_2_pub yet so we wait and try again")
			try:
				rate.sleep()
			except rospy.ROSInterruptException:
				# This is to avoid error when world is rested, time when backwards.
				pass
		rospy.logdebug("joint_2_pub Publisher Connected")

		while (self.joint_3_pub.get_num_connections() == 0):
			rospy.logdebug("No susbribers to joint_3_pub yet so we wait and try again")
			try:
				rate.sleep()
			except rospy.ROSInterruptException:
				# This is to avoid error when world is rested, time when backwards.
				pass
		rospy.logdebug("joint_3_pub Publisher Connected")

		while (self.joint_4_pub.get_num_connections() == 0):
			rospy.logdebug("No susbribers to joint_4_pub yet so we wait and try again")
			try:
				rate.sleep()
			except rospy.ROSInterruptException:
				# This is to avoid error when world is rested, time when backwards.
				pass
		rospy.logdebug("joint_4_pub Publisher Connected")

		while (self.joint_5_pub.get_num_connections() == 0):
			rospy.logdebug("No susbribers to joint_5_pub yet so we wait and try again")
			try:
				rate.sleep()
			except rospy.ROSInterruptException:
				# This is to avoid error when world is rested, time when backwards.
				pass
		rospy.logdebug("joint_5_pub Publisher Connected")

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

	"""
		This function performs the computation of the inverse kinematic the control
		each joint value of the arm
	"""
	def inverse_kinematic_from_pose(self, px, py, pz, alpha, theta5):
		b = 0.1
		theta234 = alpha
		d1 = d3 = d4 = 1
		d2 = 1
		
		pz = pz

		theta1 = - math.atan2(py, px)

		c1 = math.cos(theta1)
		s1 = math.sin(theta1)

		c234 = math.cos(theta234)
		s234 = math.sin(theta234)

		#p1 = px * c1 + py * s1 - d4 * c234
		#p2 = pz - d4 * s234

		x_sqr = px**2
		y_sqr = py**2

		theta3 = math.acos((x_sqr + y_sqr + (pz - d1 - b)**2 - d2**2 - d3**2) / (2*d2*d3))
		c3 = math.cos(theta3)
		s3 = math.sin(theta3)


		theta2 = math.atan2((pz - d1 - b), math.sqrt(x_sqr + y_sqr)) - math.atan2((s3 * d3), (d2 + c3 * d3))
		
		theta4 = (theta234-theta2-theta3)
		
		return theta1, - theta2, theta3, - theta4

	"""
		This callback is called at every update of the VR controller pose
	"""
	def rightControlerPoseStampCallback(self, ros_pose):
		self.actual_controler_pose = ros_pose.pose

		px = self.actual_controler_pose.position.x
		py = self.actual_controler_pose.position.y
		pz = self.actual_controler_pose.position.z
		
		# Extract from the controller the values of theta 5 and the approach angle (alpha)
		(theta5, alpha, _) = euler_from_quaternion([self.actual_controler_pose.orientation.x, 
				 self.actual_controler_pose.orientation.y, 
				 self.actual_controler_pose.orientation.z, 
				 self.actual_controler_pose.orientation.w], 
				 'sxyz')
		
		# To adapt in the right frame
		alpha = -alpha + 0.7
		theta5 = theta5
		print("theta5 is " + str(theta5))
		theta1, theta2, theta3, theta4 = self.inverse_kinematic_from_pose(px, py, pz, alpha, theta5)
		print("angles have been calculated")
		self.actual_joint_pose = [theta1, theta2, theta3, theta4, theta5]
		print(str(self.actual_joint_pose) + " and alpha: " + str(alpha))
		self.move_joints(self.actual_joint_pose)
	
if __name__=="__main__":
	rospy.init_node('arm_joint_publisher_node')
	joint_publisher = JointPub()
	try:
		rospy.spin()
	except rospy.KeyboardInterrupt:
		print("Shutting down")

