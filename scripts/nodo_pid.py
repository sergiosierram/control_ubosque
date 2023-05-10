#!/usr/bin/python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from std_srvs.srv import Empty, EmptyResponse
from threading import Lock

class PIDControl():
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
		self.kp = rospy.get_param("~kp", 0)
		self.ki = rospy.get_param("~ki", 0)
		self.kd = rospy.get_param("~kd", 0)
		self.control_rate = rospy.get_param("~control_rate", 10)
		self.v_limit = rospy.get_param("~v_limit", 0.5)
		self.w_limit = rospy.get_param("~w_limit", 0.3)
		self.odom_topic = rospy.get_param("~odom_topic", "/odom")
		self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
		self.setpoint_topic = rospy.get_param("~setpoint_topic", "/setpoint")
		self.update_params_service = self.name + rospy.get_param("~update_params_service", "/update_parameters")
		self.param_lock = Lock()
		return

	def initSubscribers(self):
		self.sub_odom = rospy.Subscriber(self.odom_topic, Odometry, self.callbackOdom)
		self.sub_setpoint = rospy.Subscriber(self.setpoint_topic, Point, self.callbackSetpoint)
		return

	def initPublishers(self):
		self.pub_vel = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size = 10)
		return

	def initServiceClients(self):
		rospy.Service(self.update_params_service, Empty, self.callbackUpdateParams)
		return

	def initVariables(self):
		self.change_odom = self.change_setpoint = False
		self.msg_vel = Twist()
		self.error_prev = 0
		self.rate = rospy.Rate(self.control_rate)
		return
		
	def callbackOdom(self, msg):
		self.pos_x = msg.pose.pose.position.x
		self.change_odom = True
		return

	def callbackSetpoint(self, msg):
		self.set_x = msg.x
		self.change_setpoint = True
		return

	def callbackUpdateParams(self):
		with self.param_lock:
			self.initParameters()
			rospy.loginfo("[%s] Parameter update after request", self.name)
		return EmptyResponse()

	def controller(self):
		self.error = self.set_x - self.pos_x
		self.derror = self.error_prev - self.error
		self.ierror = self.error_prev + self.error
		self.error_prev = self.error
		if abs(self.error) > 0.1:
			self.v = self.kp*self.error + self.ki*self.ierror + self.kd*self.derror
			self.v = self.v_limit*np.tanh(0.5*self.v)
		else:
			self.v = 0
		return

	def makeVelMsg(self):
		self.msg_vel = Twist()
		self.msg_vel.linear.x = self.v
		return

	def main(self):
		rospy.loginfo("[%s] Node OK", self.name)
		while not rospy.is_shutdown():
			if self.change_setpoint and self.change_odom:
				self.controller()
				self.makeVelMsg()
				self.pub_vel.publish(self.msg_vel)
				self.change_odom = self.change_setpoint = False
			self.rate.sleep()
		
if __name__== '__main__':
	try:
		pid = PIDControl('linear_pid')
	except rospy.ROSInterruptException:
		print("Error!")


