#!/usr/bin/python
import rospy
import numpy as np
from sklearn.cluster import DBSCAN
from tf.transformations import euler_from_quaternion as efq
from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import LaserScan
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
		#
		self.eps = rospy.get_param("~eps", 0.05)
		self.min_samples = rospy.get_param("~min_samples", 10)
		self.frame_id = rospy.get_param("~frame_id", "odom")
		#Topicos
		self.scan_topic = rospy.get_param("~scan_topic", "/scan")
		self.obstacles_topic = rospy.get_param("~obstacles_topic", "/obstacles")
		#Additional
		self.control_rate = rospy.get_param("~control_rate", 10)
		self.update_params_service = self.name + rospy.get_param("~update_params_service", "/update_parameters")
		self.param_lock = Lock()
		return

	def initSubscribers(self):
		self.sub_scan = rospy.Subscriber(self.scan_topic, LaserScan, self.callbackScan)
		return

	def initPublishers(self):
		self.pub_obstacles = rospy.Publisher(self.obstacles_topic, PoseArray, queue_size = 10)
		return

	def initServiceClients(self):
		rospy.Service(self.update_params_service, Empty, self.callbackUpdateParams)
		return

	def initVariables(self):
		self.change_scan = False
		self.n_obst = 0
		self.rate = rospy.Rate(self.control_rate)
		return

	def callbackUpdateParams(self):
		with self.param_lock:
			self.initParameters()
			rospy.loginfo("[%s] Parameter update after request", self.name)
		return EmptyResponse()

	def callbackScan(self, msg):
		angle_min = msg.angle_min
		angle_inc = msg.angle_increment
		ranges = msg.ranges
		data = []
		aux = angle_min
		if ranges != []:
			for i in range(0, len(ranges)):
				data.append([ranges[i]*np.cos(aux), ranges[i]*np.sin(aux)])
				aux += angle_inc
			self.data = np.array(data)  
			self.change_scan = True
		else:
			self.change_scan = False
		return

	def detect(self):
		self.data = self.data[~np.any(np.isnan(self.data), axis=1)]
		self.data = self.data[~np.any(np.isinf(self.data), axis=1)]
		if self.data.size > 2:
			try:
				clustering = DBSCAN(eps = self.eps, min_samples = 5).fit(self.data)
				labels = clustering.labels_
				k = np.max(clustering.labels_)
				obstacles_means = []
				for ki in range(k+1):
					points = self.data[labels == ki]
					x_mean = np.mean(points[:, 0])
					y_mean = np.mean(points[:, 1])
					obstacles_means.append([x_mean, y_mean])
				if k+1 != self.n_obst:
					self.n_obst = k+1
					rospy.loginfo("[%s] Detected %d obstacles", self.name,self.n_obst)
				return obstacles_means
			except Exception as e:
				return []
		return []

	def makeMsgList(self, obstacles):
		if obstacles != []:
			obs_list = []
			for obs in obstacles:
				msg = Pose()
				msg.position.x = obs[0]
				msg.position.y = obs[1]
				obs_list.append(msg)
			array = PoseArray()
			array.header.seq = 0
			array.header.stamp.secs = rospy.get_rostime().secs
			array.header.stamp.nsecs = rospy.get_rostime().nsecs
			array.header.frame_id = self.frame_id
			array.poses = obs_list
			self.pub_obstacles.publish(array)
		return

	def main(self):
		rospy.loginfo("[%s] Node OK", self.name)
		while not rospy.is_shutdown():
			if self.change_scan:
				obstacles = self.detect()
				self.makeMsgList(obstacles)
			self.rate.sleep()
		return

if __name__== '__main__':
	try:
		path = ObstacleDetector('obstacle_detector')
	except rospy.ROSInterruptException:
		print("Error!")

