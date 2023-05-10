#!/usr/bin/python3
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion as efq
from tf.transformations import quaternion_from_euler as qfe
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty, EmptyResponse
from threading import Lock

class ThetaSupervisor():
	def __init__(self, name):
		self.name = name
		rospy.init_node(self.name, anonymous = True)
		rospy.loginfo("[%s] Starting node", self.name)
		self.initParameters()
		self.initSubscribers()
		self.initPublishers()
		self.initServiceClients()
		self.initVariables()
		self.main()

	def initParameters(self):
		self.radio = rospy.get_param("~radio", 2)
		#Topicos
		self.theta_object_topic = rospy.get_param("~theta_object_topic", "/theta_object")
		self.theta_desired_topic = rospy.get_param("~theta_desired_topic", "/theta_desired")
		self.theta_path_topic = rospy.get_param("~theta_path_topic", "/theta_path")
		self.dtilde_topic = rospy.get_param("~dtilde_topic", "/dtilde")
		self.odom_topic = rospy.get_param("~odom_topic", "/odom")
		self.theta_topic_viz = rospy.get_param("~theta_topic_viz", "/theta_desired_viz")
		self.frame_id = rospy.get_param("~frame_id", "odom")
		#Additional
		self.control_rate = rospy.get_param("~control_rate", 10)
		self.update_params_service = self.name + rospy.get_param("~update_params_service", "/update_parameters")
		self.param_lock = Lock()
		return

	def initSubscribers(self):
		self.sub_odom = rospy.Subscriber(self.odom_topic, Odometry, self.callbackOdom)
		self.sub_theta_path = rospy.Subscriber(self.theta_path_topic, Float64, self.callbackThetaPath)
		self.sub_theta_object = rospy.Subscriber(self.theta_object_topic, Float64, self.callbackThetaObject)
		self.sub_dtilde = rospy.Subscriber(self.dtilde_topic, Float64, self.callbackDTilde)
		return

	def initPublishers(self):
		self.pub_theta_desired = rospy.Publisher(self.theta_desired_topic, Float64, queue_size = 10)
		self.pub_theta_viz = rospy.Publisher(self.theta_topic_viz, PoseStamped, queue_size = 10)
		return

	def initServiceClients(self):
		rospy.Service(self.update_params_service, Empty, self.callbackUpdateParams)
		return

	def initVariables(self):
		self.count = 0
		self.change_theta_path = self.change_theta_object = self.change_dtilde = self.change_odom = False
		self.rate = rospy.Rate(self.control_rate)
		return

	def callbackUpdateParams(self):
		with self.param_lock:
			self.initParameters()
			rospy.loginfo("[%s] Parameter update after request", self.name)
		return EmptyResponse()

	def callbackThetaPath(self, msg):
		self.theta_path = msg.data
		self.change_theta_path = True
		return

	def callbackThetaObject(self, msg):
		self.theta_object = msg.data
		self.change_theta_object = True
		return

	def callbackDTilde(self, msg):
		self.dtilde = msg.data
		self.change_dtilde = True
		return

	def callbackOdom(self, msg):
		self.x_bot = msg.pose.pose.position.x
		self.y_bot = msg.pose.pose.position.y
		quat = msg.pose.pose.orientation
		self.theta_bot = efq([quat.x, quat.y, quat.z, quat.w])[2]
		self.change_odom = True
		return

	def vizTheta(self, theta):
		msg = PoseStamped()
		msg.header.seq = self.count
		msg.header.stamp.secs = rospy.get_rostime().secs
		msg.header.stamp.nsecs = rospy.get_rostime().nsecs
		msg.header.frame_id = self.frame_id
		msg.pose.position.x = self.x_bot
		msg.pose.position.y = self.y_bot
		q = qfe(0, 0, theta)
		msg.pose.orientation.x = q[0]
		msg.pose.orientation.y = q[1]
		msg.pose.orientation.z = q[2]
		msg.pose.orientation.w = q[3]
		self.count += 1
		self.pub_theta_viz.publish(msg)
		return

	def supervisor(self):
		if self.dtilde < self.radio:
			self.vizTheta(self.theta_object)
			self.theta_desired = self.theta_bot - self.theta_object
		else:
			self.vizTheta(self.theta_path)
			self.theta_desired = self.theta_bot - self.theta_path
		self.pub_theta_desired.publish(self.theta_desired)
		return

	def main(self):
		rospy.loginfo("[%s] Node OK", self.name)
		while not rospy.is_shutdown():
			if self.change_theta_path and self.change_theta_object and self.change_dtilde and self.change_odom:
				self.supervisor()
				self.change_theta_path = self.change_theta_object = self.change_dtilde = self.change_odom = False
			self.rate.sleep()
		return

if __name__== '__main__':
	try:
		path = ThetaSupervisor('theta_supervisor')
	except rospy.ROSInterruptException:
		print("Error!")
