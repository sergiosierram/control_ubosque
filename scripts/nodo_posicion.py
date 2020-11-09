#!/usr/bin/python
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion as efq
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64, Bool
from std_srvs.srv import Empty, EmptyResponse
from threading import Lock

class PosControl():
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
		'''
		El simbolo ~ indica que busca el parametro en el servidor global
		1er param = el nombre del parametro en el archivo .yaml y en el servidor
		2do param = el valor por defecto, que no tiene que ser igual al del .yaml
		'''
		self.gamma = rospy.get_param("~gamma", 0.5) 
		self.k = rospy.get_param("~k", 0.7)
		self.odom_topic = rospy.get_param("~odom_topic", "/odom")
		self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
		self.error_topic = rospy.get_param("~error_topic", "/error")
		self.goal_topic = rospy.get_param("~goal_topic", "/move_base/simple_goal")
		self.pos_reached_topic = rospy.get_param("~pos_reached_topic", "/pos_reached")	
		self.control_rate = rospy.get_param("~control_rate", 10)
		self.update_params_service = self.name + rospy.get_param("~update_params_service", "/update_parameters")
		self.param_lock = Lock()
		return

	def initSubscribers(self):
		'''
		1er param = nombre del topico
		2do param = tipo del mensaje 
		3er param = callback que se llama cuando llega un mensaje
		'''
		self.sub_odom = rospy.Subscriber(self.odom_topic, Odometry, self.callbackOdom)
		self.sub_goal = rospy.Subscriber(self.goal_topic, PoseStamped, self.callbackGoal)
		return

	def initPublishers(self):
		'''
		1er param = nombre del topico
		2do param = tipo del mensaje 
		3er param = la pila
		'''
		self.pub_vel = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size = 10)
		self.pub_error = rospy.Publisher(self.error_topic, Float64, queue_size = 10)
		self.pub_pos_reached = rospy.Publisher(self.pos_reached_topic, Bool, queue_size = 10)
		return

	def initServiceClients(self):
		rospy.Service(self.update_params_service, Empty, self.callbackUpdateParams)
		return

	def initVariables(self):
		self.change_odom = self.change_goal = self.active = False
		self.msg_vel = Twist()
		self.rate = rospy.Rate(self.control_rate)
		return

	def callbackUpdateParams(self):
		with self.param_lock:
			self.initParameters()
			rospy.loginfo("[%s] Parameter update after request", self.name)
		return EmptyResponse()

	def callbackOdom(self, msg):
		self.x_bot = msg.pose.pose.position.x
		self.y_bot = msg.pose.pose.position.y
		quat = msg.pose.pose.orientation
		self.theta_bot = efq([quat.x, quat.y, quat.z, quat.w])[2]
		self.change_odom = True
		return

	def callbackGoal(self, msg):
		self.x_goal = msg.pose.position.x
		self.y_goal = msg.pose.position.y
		quat = msg.pose.orientation
		self.theta_goal = efq([quat.x, quat.y, quat.z, quat.w])[2]
		self.change_goal = True

	def getDistance(self, x1, y1, x2, y2):
		d = np.sqrt(np.power(x1-x2, 2) + np.power(y1-y2, 2))
		return d

	def controller(self):
		e = self.getDistance(self.x_bot, self.y_bot, self.x_goal, self.y_goal)
		alpha = np.arctan2(self.y_goal - self.y_bot, self.x_goal - self.x_bot) - self.theta_bot
		alpha2 = self.theta_goal-self.theta_bot
		if e <= 0.1 and alpha <= 0.1 and alpha2 <= 0.1:
			self.active = False
			self.v = self.w = 0
			rospy.loginfo("[%s] Goal reached!", self.name)
			self.pub_pos_reached.publish(True)
		else:
			self.active = True
			self.v = self.gamma*np.tanh(e)*np.cos(alpha)
			self.w = (self.k*alpha) + self.gamma*(np.tanh(e)/e)*np.sin(alpha)*np.cos(alpha) - (0.3*alpha2)
			if self.v < 0:
				self.v = 0
				self.w = self.w + np.sign(self.w)*0.1
		self.pub_error.publish(e)
		return

	def makeVelMsg(self):
		self.msg_vel = Twist()
		self.msg_vel.linear.x = self.v
		self.msg_vel.angular.z = self.w
		return

	def main(self):
		rospy.loginfo("[%s] Node OK", self.name)
		while not rospy.is_shutdown():
			if self.change_odom and (self.change_goal or self.active):
				self.controller()
				self.makeVelMsg()
				self.pub_vel.publish(self.msg_vel)
				self.change_odom = self.change_goal = False
			self.rate.sleep()
		return

if __name__== '__main__':
	try:
		pos = PosControl('position_control')
	except rospy.ROSInterruptException:
		print("Error!")












