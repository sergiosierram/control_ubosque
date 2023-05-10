#!/usr/bin/python3
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion as efq
from tf.transformations import quaternion_from_euler as qfe
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty, EmptyResponse
from threading import Lock

class ObstacleDetector():
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
		self.frame_id = rospy.get_param("~frame_id", "odom")
		self.c1 = rospy.get_param("~c1", 20)
		self.c2 = rospy.get_param("~c2", 20)
		self.sigma = rospy.get_param("~sigma", 0.3)
		self.sigma2 = rospy.get_param("~sigma2", 1.5)
		self.sigma3 = rospy.get_param("~sigma3", 1)
		self.radio = rospy.get_param("~radio", 2)
		#Topicos
		self.scan_topic = rospy.get_param("~scan_topic", "/scan")
		self.odom_topic = rospy.get_param("~odom_topic", "/odom")
		self.theta_topic = rospy.get_param("~theta_topic", "/theta_object")
		self.dtilde_topic = rospy.get_param("~dtilde_topic", "/dtilde")
		self.theta_topic_viz = rospy.get_param("~theta_topic_viz", "/theta_escape_viz")
		self.zone_topic = rospy.get_param("~zone_topic", "/zone")
		#Additional
		self.control_rate = rospy.get_param("~control_rate", 10)
		self.update_params_service = self.name + rospy.get_param("~update_params_service", "/update_parameters")
		self.param_lock = Lock()
		return

	def initSubscribers(self):
		self.sub_scan = rospy.Subscriber(self.scan_topic, LaserScan, self.callbackScan)
		self.sub_odom = rospy.Subscriber(self.odom_topic, Odometry, self.callbackOdom)
		return

	def initPublishers(self):
		self.pub_theta = rospy.Publisher(self.theta_topic, Float64, queue_size = 10)
		self.pub_dtilde = rospy.Publisher(self.dtilde_topic, Float64, queue_size = 10)
		self.pub_theta_viz = rospy.Publisher(self.theta_topic_viz, PoseStamped, queue_size = 10)
		self.pub_zone = rospy.Publisher(self.zone_topic, Marker, queue_size = 10)
		return

	def initServiceClients(self):
		rospy.Service(self.update_params_service, Empty, self.callbackUpdateParams)
		return

	def initVariables(self):
		self.change_scan = self.change_odom = False
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

	def callbackScan(self, msg):
		angle_min = msg.angle_min
		angle_inc = msg.angle_increment
		ranges = msg.ranges
		data = []
		aux = angle_min
		if ranges != []:
			for i in range(0, len(ranges)):
				if aux > -np.pi/2 and aux < np.pi/2:
					data.append([ranges[i]*np.cos(aux), ranges[i]*np.sin(aux)])
				aux += angle_inc
			self.data = np.array(data)
			self.change_scan = True
		else:
			self.change_scan = False
		return

	def getDistance(self, x1, y1, x2, y2):
		d = np.sqrt(np.power(x2-x1, 2) + np.power(y2-y1, 2))
		return d

	def getAngle(self, x1, y1, x2, y2):
		a = np.arctan2(y2 - y1, x2 - x1)
		return a

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

	def drawZone(self):
		marker = Marker()
		marker.header.seq = self.count
		marker.header.stamp.secs = rospy.get_rostime().secs
		marker.header.stamp.nsecs = rospy.get_rostime().nsecs
		marker.header.frame_id = self.frame_id
		marker.id = 0 #Identifier
		marker.type = 3 #Cylinder
		marker.action = 0 #Add
		marker.pose.position.x = self.x_bot
		marker.pose.position.y = self.y_bot
		marker.scale.x = 4
		marker.scale.y = 4
		marker.scale.z = 0.1
		marker.color.a = 0.3
		marker.color.r = 0
		marker.color.g = 0
		marker.color.b = 1
		self.pub_zone.publish(marker)
		return


	def detect(self):
		try:
			self.data = self.data[~np.any(np.isnan(self.data), axis=1)]
			self.data = self.data[~np.any(np.isinf(self.data), axis=1)]
		except:
			self.data = np.array([])
		theta_d = self.theta_bot
		dtilde = 10
		if self.data.size > 2:
			#Calcular distancias y betas
			data = np.array([[self.getDistance(0, 0, p[0], p[1]), self.getAngle(0, 0, p[0], p[1])] for p in self.data])
			distances = data[:,0]
			dists = distances[np.argwhere(distances <= self.radio)]
			betas = data[:,1]
			betas = np.array(betas[np.argwhere(distances <= self.radio)])
			if dists.size > 2:
				# Calcular distancias de penetracion en semicirculo
				dtildes = self.radio - dists
				#Calcular el peso o contribucion de cada obstaculo
				alphas = (1 - np.exp(-1*np.power(dtildes/self.sigma, 2)))
				alphas = alphas/np.sum(alphas)
				#weights =np.multiply(alphas, betas)
				#weights_mean = np.sum(weights)
				weights = -1*alphas*(betas - np.sign(betas)*np.pi/2)
				weights_mean = np.sum(weights)
				theta_d = weights_mean
				##theta_d = np.sign(weights_mean)*(np.pi/2 - np.abs(weights_mean)) + self.theta_bot
				# if weights_mean != np.nan:
				# 	if weights_mean < -0.1:
				# 		theta_d = -weights_mean - np.pi/2
				# 	elif weights_mean > 0.1:
				# 		theta_d = -weights_mean + np.pi/2
				# 	else:
				# 		theta_d = self.theta_bot
				loc = np.argmax(dtildes)
				if loc.size > 1:
					dtilde = dtildes[loc]
				else:
					dtilde =  dtildes[0]
		self.vizTheta(theta_d)
		return theta_d, dtilde

	def main(self):
		rospy.loginfo("[%s] Node OK", self.name)
		while not rospy.is_shutdown():
			if self.change_scan and self.change_odom:
				theta_d, dtilde = self.detect()
				self.drawZone()
				self.pub_theta.publish(theta_d)
				self.pub_dtilde.publish(dtilde)
			self.rate.sleep()
		return

if __name__== '__main__':
	try:
		path = ObstacleDetector('obstacle_avoidance')
	except rospy.ROSInterruptException:
		print("Error!")
