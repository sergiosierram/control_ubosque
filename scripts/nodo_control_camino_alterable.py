#!/usr/bin/python
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion as efq
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64, Bool
from std_srvs.srv import Empty, EmptyResponse
from threading import Lock

class PathControl():
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
		#Parametros controlador
		self.v_ref = rospy.get_param("~v_ref",0.5)
		self.ly = rospy.get_param("~ly",3)
		self.lx = rospy.get_param("~lx",3)
		self.ky = rospy.get_param("~ky",0.7)
		self.kx = rospy.get_param("~yx",0.7)
		self.stop_distance = rospy.get_param("~stop_distance",0.1)
		#Topicos
		self.path_topic = rospy.get_param("~path_topic", "/path")
		self.odom_topic = rospy.get_param("~odom_topic", "/odom")
		self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
		self.goal_topic = rospy.get_param("~goal_topic", "/move_base/simple_goal") 
		self.pos_reached_topic = rospy.get_param("~pos_reached_topic", "/pos_reached")
		#Additional
		self.control_rate = rospy.get_param("~control_rate", 10)
		self.update_params_service = self.name + rospy.get_param("~update_params_service", "/update_parameters")
		self.param_lock = Lock()
		return

	def initSubscribers(self):
		self.sub_odom = rospy.Subscriber(self.odom_topic, Odometry, self.callbackOdom)
		self.sub_path = rospy.Subscriber(self.path_topic, Path, self.callbackPath)
		self.sub_goal = rospy.Subscriber(self.goal_topic, PoseStamped, self.callbackGoal)
		self.sub_pos_reached = rospy.Subscriber(self.pos_reached_topic, Bool, self.callbackPosReached)
		return

	def initPublishers(self):
		self.pub_vel = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size = 10)
		return

	def initServiceClients(self):
		rospy.Service(self.update_params_service, Empty, self.callbackUpdateParams)
		return

	def initVariables(self):
		self.change_odom = self.change_path = self.goal_reached = self.change_goal = False		
		self.loc = -1
		self.rate = rospy.Rate(self.control_rate)
		return

	def callbackUpdateParams(self):
		with self.param_lock:
			self.initParameters()
			rospy.loginfo("[%s] Parameter update after request", self.name)
		return EmptyResponse()

	def callbackPosReached(self, msg):
		self.closestPoint()
		self.change_goal = False
		return

	def callbackGoal(self, msg):
		self.change_goal = True
		return

	def callbackOdom(self, msg):
		self.x_bot = msg.pose.pose.position.x
		self.y_bot = msg.pose.pose.position.y
		quat = msg.pose.pose.orientation
		self.theta_bot = efq([quat.x, quat.y, quat.z, quat.w])[2] 
		self.change_odom = True
		return

	def callbackPath(self, msg):
		#Descartamos el header por ahora
		self.x_path = np.array([p.pose.position.x for p in msg.poses])
		self.y_path = np.array([p.pose.position.y for p in msg.poses])
		self.theta_path = []
		try:
			self.theta_path = np.arctan2(np.diff(self.y_path), np.diff(self.x_path))
		except Exception as e:
			print(e)
			#Esto no deberia pasar
			rospy.logerr("[%s] Path cant not be differianted", self.name)
			return
		self.path_size = self.x_path.size
		self.change_path = True
		return

	def makeVelMsg(self):
		self.msg_vel = Twist()
		self.msg_vel.linear.x = self.v
		self.msg_vel.angular.z = self.w
		return

	def closestPoint(self):
		#Estimar la distancia a todos los puntos del camino
		d = np.sqrt(np.power((self.x_path - self.x_bot),2) + np.power((self.y_path - self.y_bot),2))
		#Estimar la posicion de la distancia minima
		loc = np.argmin(d)
		#Extraer x & y deseados
		self.x_desired = self.x_path[loc]
		self.y_desired = self.y_path[loc]
		self.theta_ref = self.theta_path[loc]

		d = self.getDistance(self.x_desired, self.y_desired, self.x_bot, self.y_bot)
		if d <= self.stop_distance and self.loc < self.path_size:
			rospy.loginfo("[%s] Reached pose %d at [%f, %f] - Robot [%f, %f]!", self.name, self.loc, self.x_desired, self.y_desired, self.x_bot, self.y_bot)
			loc += 1
			self.goal_reached = False
			self.x_desired = self.x_path[self.loc]
			self.y_desired = self.y_path[self.loc]
			self.theta_ref = self.theta_path[self.loc]
		if self.path_size - loc == 1:
			rospy.loginfo("[%s] Path ended!", self.name)
			self.goal_reached = True
			return
		else:
			self.theta_ref = self.theta_path[loc]
		return

	def getDistance(self, x1, y1, x2, y2):
		d = np.sqrt(np.power(x1-x2, 2) + np.power(y1-y2, 2))
		return d

	def controller(self):
		self.closestPoint()
		x_error = self.x_desired - self.x_bot
		y_error = self.y_desired - self.y_bot
		x_point = self.v_ref*np.cos(self.theta_ref)+ self.lx*np.tanh((self.kx/self.lx)*x_error)
		y_point = self.v_ref*np.sin(self.theta_ref)+ self.ly*np.tanh((self.ky/self.lx)*y_error)
		c_inv = np.array( [[np.cos(self.theta_bot), np.sin(self.theta_bot)],
						   [-(1/0.1)*np.sin(self.theta_bot), (1/0.1)*np.cos(self.theta_bot)]])
		[self.v, self.w] = np.matmul(c_inv, np.array([[x_point],[y_point]]))
		return

	def main(self):
		rospy.loginfo("[%s] Node OK", self.name)
		while not rospy.is_shutdown():
			if not self.change_goal:
				if self.change_path:
					if not self.goal_reached and self.change_odom:
						self.controller()
						self.makeVelMsg()
						self.pub_vel.publish(self.msg_vel)
						self.change_odom = False
					if self.goal_reached:
						self.change_path = False
			else:
				rospy.logwarn("[%s] Attending another goal!", self.name)
			self.rate.sleep()
		return

if __name__== '__main__':
	try:
		path = PathControl('path_control_alterable')
	except rospy.ROSInterruptException:
		print("Error!")

