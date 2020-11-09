#!/usr/bin/python
import rospy
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, EmptyResponse
from threading import Lock

class ForceBridge():
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
		self.vel_topic = rospy.get_param("vel_topic","/aux_cmd_vel")
		self.force_topic = rospy.get_param("force_topic","/force")
		#Additional
		self.control_rate = rospy.get_param("~control_rate", 10)
		self.update_params_service = self.name + rospy.get_param("~update_params_service", "/update_parameters")
		self.param_lock = Lock()
		return

	def initSubscribers(self):
		self.sub_vel = rospy.Subscriber(self.vel_topic, Twist, self.callbackVel)
		return

	def initPublishers(self):
		self.pub_force = rospy.Publisher(self.force_topic, Wrench, queue_size = 10)
		return

	def initServiceClients(self):
		rospy.Service(self.update_params_service, Empty, self.callbackUpdateParams)
		return

	def callbackUpdateParams(self):
		with self.param_lock:
			self.initParameters()
			rospy.loginfo("[%s] Parameter update after request", self.name)
		return EmptyResponse()

	def initVariables(self):
		self.change = False
		self.rate = rospy.Rate(self.control_rate)
		return

	def callbackVel(self, msg):
		#msg = self.vel_format(data)
		self.msg_force = Wrench()
		self.msg_force.force.x = 0
		self.msg_force.force.y = msg.linear.x*8
		self.msg_force.force.z = 0
		self.msg_force.torque.x = 0
		self.msg_force.torque.y = 0
		self.msg_force.torque.z = msg.angular.z*4
		self.change = True
		return

	def main(self):
		rospy.loginfo("[%s] Node OK", self.name)
		while not rospy.is_shutdown():
			if self.change:
				self.pub_force.publish(self.msg_force)
				self.change = False
			self.rate.sleep()

if __name__ == '__main__':
	try:
		sw = ForceBridge("force_bridge")
	except rospy.ROSInterruptException:
		pass
