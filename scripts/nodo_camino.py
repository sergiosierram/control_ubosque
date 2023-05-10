#!/usr/bin/python3
import rospy
import numpy as np
import csv
from tf.transformations import euler_from_quaternion as efq
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64
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
		self.csv_file = rospy.get_param("~csv_file","/home/walker/catkin_ws/src/control_ubosque/config/paths/path_points.csv")
		self.csv_header = rospy.get_param("~csv_header", True)
		self.delay = rospy.get_param("~delay", 10)
		self.frame_id = rospy.get_param("~frame_id", "odom")
		self.path_topic = rospy.get_param("~path_topic", "/path")
		self.control_rate = rospy.get_param("~control_rate", 10)
		self.update_params_service = self.name + rospy.get_param("~update_params_service", "/update_parameters")
		self.param_lock = Lock()
		return

	def initSubscribers(self):
		return

	def initPublishers(self):
		self.pub_path = rospy.Publisher(self.path_topic, Path, queue_size = 10)
		return

	def initServiceClients(self):
		rospy.Service(self.update_params_service, Empty, self.callbackUpdateParams)
		return

	def initVariables(self):
		self.rate = rospy.Rate(self.control_rate)
		self.poses = np.array([])
		self.read_flag = False
		return

	def callbackUpdateParams(self):
		with self.param_lock:
			self.initParameters()
			rospy.loginfo("[%s] Parameter update after request", self.name)
		return EmptyResponse()

	def readFile(self):
		try:
			with open(self.csv_file) as csv_file:
				csv_reader = csv.reader(csv_file, delimiter=",")
				for counter, row in enumerate(csv_reader):
					if counter > 0:
						row_float = list(map(float, row))
						row_float = np.reshape(np.array(row_float), (1, 2))
						if counter == 1:
							self.poses = row_float
						else:
							self.poses = np.append(self.poses, row_float, axis=0)
			self.read_flag = True
		except Exception as e:
			rospy.logerr(e)
			self.read_flag = False
		return

	def makePath(self):
		msg = Path()
		msg.header.seq = 0
		msg.header.stamp.secs = rospy.get_rostime().secs
		msg.header.stamp.nsecs = rospy.get_rostime().nsecs
		msg.header.frame_id = self.frame_id
		header = msg.header
		pose_array = []
		for p in self.poses:
			pose = PoseStamped()
			pose.pose.position.x = p[0]
			pose.pose.position.y = p[1]
			pose.header = header
			pose_array.append(pose)
		msg.poses = pose_array
		self.pub_path.publish(msg)
		return


	def main(self):
		rospy.loginfo("[%s] Node OK", self.name)
		if self.delay > 0:
			rospy.loginfo("[%s] This node was asked to wait for %d seconds", self.name, self.delay)
			rospy.sleep(self.delay)
		rospy.loginfo("[%s] Starting ...", self.name)
		self.readFile()
		if self.read_flag:
			rospy.loginfo("[%s] Reading CSV file OK", self.name)
			self.makePath()
			rospy.loginfo("[%s] Nodo exiting with code 0", self.name)
		else:
			rospy.logerr("[%s] Reading CSV failed", self.name)
			rospy.logwarn("[%s] Exiting due to CSV reading error", self.name)
		return

if __name__== '__main__':
	try:
		pos = PosControl('path')
	except rospy.ROSInterruptException:
		print("Error!")












