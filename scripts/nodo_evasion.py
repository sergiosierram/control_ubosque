#!/usr/bin/python
import rospy
import numpy as np
from tf.transformations import quaternion_from_euler as qfe
from tf.transformations import euler_from_quaternion as efq
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Float64
from std_srvs.srv import Empty, EmptyResponse
from threading import Lock

class Avoidance():
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
		self.c1 = rospy.get_param("~c1", 20)
		self.c2 = rospy.get_param("~c2", 20)
		self.sigma = rospy.get_param("~sigma", 0.3)
		self.sigma2 = rospy.get_param("~sigma2", 1.5)
		self.sigma3 = rospy.get_param("~sigma3", 1)
		#Topicos
		self.odom_topic = rospy.get_param("~odom_topic", "/odom")
		self.obst_topic = rospy.get_param("~obst_topic", "/obstacles")
		self.theta_topic = rospy.get_param("~theta_topic", "/theta_escape")
		self.theta_topic_viz = rospy.get_param("~theta_topic_viz", "/theta_escape_viz")
		self.frame_id = rospy.get_param("~frame_id", "odom")
		#Additional
		self.control_rate = rospy.get_param("~control_rate", 10)
		self.update_params_service = self.name + rospy.get_param("~update_params_service", "/update_parameters")
		self.param_lock = Lock()
		return

	def initSubscribers(self):
		self.sub_odom = rospy.Subscriber(self.odom_topic, Odometry, self.callbackOdom)
		self.sub_obst = rospy.Subscriber(self.obst_topic, PoseArray, self.callbackObst)
		return

	def initPublishers(self):
		self.pub_theta = rospy.Publisher(self.theta_topic, Float64, queue_size = 10)
		self.pub_theta_viz = rospy.Publisher(self.theta_topic_viz, PoseStamped, queue_size = 10)
		return

	def initServiceClients(self):
		rospy.Service(self.update_params_service, Empty, self.callbackUpdateParams)
		return

	def initVariables(self):
		self.change_odom = self.change_obst = False
		self.count = 0
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

	def callbackObst(self, msg):
		self.obstacles = msg.poses
		self.change_obst = True
		return

	def getDistance(self, x1, y1, x2, y2):
		d = np.sqrt(np.power(x2-x1, 2) + np.power(y2-y1, 2))
		return d

	def getAngle(self, x1, y1, x2, y2):
		a = np.arctan2(y2 - y1, x2 - x1)
		return a

	def getEscapeTheta(self):
		#Calcular distancias y betas
		data = np.array([[self.getDistance(0, 0, p.position.x, p.position.y), self.getAngle(0, 0, p.position.x, p.position.y)] for p in self.obstacles])
		distances = data[:,0]
		distances = distances[np.argwhere(distances <= 4)]
		print(distances)
		betas = data[:,1]
		betas = betas[np.argwhere(distances <= 4)]
		#Calcular el peso o contribucion de cada obstaculo
		alphas_d = self.c1*np.exp(-1*np.power(((distances-1.0)/self.sigma), 2))/distances
		#alphas_d = alphas_d/np.sum(alphas_d)
		alphas_t = np.exp(-1*np.power((betas/self.sigma2), 2))
		#alphas_t = alphas_t/np.sum(alphas_t)
		weights = np.multiply(alphas_d, alphas_t)
		betas_w = np.multiply(weights, betas)
		theta_d = (np.pi/2)*np.tanh(np.sum(betas_w)/self.sigma3) + self.theta_bot
		print(theta_d)
		return theta_d

	def makeMsgTheta(self, theta):
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

	def main(self):
		rospy.loginfo("[%s] Node OK", self.name)
		while not rospy.is_shutdown():
			if self.change_odom and self.change_obst:
				theta = self.getEscapeTheta()
				self.pub_theta.publish(theta)
				self.makeMsgTheta(theta)
				self.change_odom = self.change_obst = False
			self.rate.sleep()
		return

if __name__== '__main__':
	try:
		avoid = Avoidance('obstacle_avoidance')
	except rospy.ROSInterruptException:
		print("Error!")

