#!/usr/bin/python3
import rospy
import numpy as np
import message_filters
from std_msgs.msg import Bool
from geometry_msgs.msg import Wrench, Twist
from std_srvs.srv import EmptyResponse, Empty
from sys import stdin
from threading import Lock

class Control():
	def __init__(self, name):
		self.name = name
		self.rospy = rospy
		self.rospy.init_node('Control', anonymous = True)
		self.rospy.loginfo("[%s] Starting Admittance Controller", self.name)
		self.initParameters()
		self.initSubscribers()
		self.initPublishers()
		self.initServiceClients()
		self.initVariables()
		self.mainControl()

	def initParameters(self):
		self.finalVelTopic = self.rospy.get_param("~final_vel_topic","/cmd_vel")
		self.virtualWrenchTopic = self.rospy.get_param("~virtual_wrench_topic","/virtual_wrench")
		self.humanWrenchTopic = self.rospy.get_param("~human_wrench_topic","/human_wrench")
		self.sharedWrenchTopic = self.rospy.get_param("~shared_wrench_topic","/shared_wrench")
		self.falconWrenchTopic = self.rospy.get_param("~falcon_wrench_topic","/falcon_wrench")
		self.controlMode = self.rospy.get_param("~control_parameters/mode","user")
		self.controllerParams = {	"m": self.rospy.get_param("~control_parameters/mass",8),
									"b_l": self.rospy.get_param("~control_parameters/ldaming",10),
									"j": self.rospy.get_param("~control_parameters/inertia",5),
									"b_a": self.rospy.get_param("~control_parameters/adamping",20)}
		self.wLimit = self.rospy.get_param("~angular_vel_limit",0.3)
		self.vLimit = self.rospy.get_param("~linear_vel_limit",0.3)
		self.controlRate = self.rospy.get_param("~control_parameters/rate",100)
		self.updateParamsService = self.name + self.rospy.get_param("~update_params_service", "/update_parameters")
		self.param_lock = Lock()
		return

	def initSubscribers(self):
		if self.controlMode == "assisted":
			self.rospy.loginfo("[%s] Control mode '%s' is set", self.name, self.controlMode)
			self.subTrq = self.rospy.Subscriber(self.virtualWrenchTopic, Wrench, self.callbackTrq)
			self.subFrc = self.rospy.Subscriber(self.humanWrenchTopic, Wrench, self.callbackFrc)
		elif self.controlMode == "shared":
			self.rospy.loginfo("[%s] Control mode '%s' is set", self.name, self.controlMode)
			self.subWrench = self.rospy.Subscriber(self.sharedWrenchTopic, Wrench, self.callbackWrench)
		elif self.controlMode == "assisted_haptic":
			self.rospy.loginfo("[%s] Control mode '%s' is set", self.name, self.controlMode)
			self.subTrq = self.rospy.Subscriber(self.falconWrenchTopic, Wrench, self.callbackTrq)
			self.subFrc = self.rospy.Subscriber(self.humanWrenchTopic, Wrench, self.callbackFrc)
		elif self.controlMode == "user":
			self.rospy.loginfo("[%s] Control mode '%s' is set", self.name, self.controlMode)
			self.subWrench = self.rospy.Subscriber(self.humanWrenchTopic, Wrench, self.callbackWrench)
		else:
			self.rospy.logwarn("[%s] Invalid '%s' control mode. Using default %s mode", self.name, self.controlMode, "user")
			self.subWrench = self.rospy.Subscriber(self.humanWrenchTopic, Wrench, self.callbackWrench)
		return

	def initPublishers(self):
		self.pubFinalVel = self.rospy.Publisher(self.finalVelTopic, Twist, queue_size = 10)
		return

	def initServiceClients(self):
		self.rospy.Service(self.updateParamsService, Empty, self.callbackUpdateParams)
		return

	def initVariables(self):
		self.vPrima = self.vLast = self.vCurrent = 0
		self.wPrima = self.wLast = self.wCurrent = 0
		self.dt = self.lastTime = 0
		self.msgForce = Wrench()
		self.msgVel = Twist()
		self.changeWrench = self.changeTrq = self.changeFrc = False
		self.rate = self.rospy.Rate(self.controlRate)
		return

	def getTime(self):
		time = rospy.get_time()
		try:
			self.dt = time - self.lastTime
			self.lastTime = time
		except:
			self.dt = 0
			self.lastTime = time
		return

	def callbackTrq(self, dataTrq):
		self.trq = dataTrq.torque.z
		self.getTime()
		self.changeTrq = True
		return

	def callbackFrc(self, dataFrc):
		self.frc = dataFrc.force.y
		self.getTime()
		self.changeFrc = True
		return

	def callbackWrench(self, data):
		self.frc = data.force.y
		self.frc_z = data.force.z
		self.trq = data.torque.z
		self.trq_y = data.torque.y
		self.getTime()
		self.changeWrench = True
		return

	def callbackUpdateParams(self, req):
		with self.param_lock:
			self.initParameters()
			self.rospy.loginfo("[%s] Parameter update after request", self.name)
		return EmptyResponse()

	def getAdmittanceResponse(self, input, v_current, v_last, v_prima, m, b, v_limit):
		if input != 0:
			v_current = (input - m*v_prima)/b
		else:
			v_current = 0
		if v_current > v_limit:
			v_current = v_limit
		elif v_current < -v_limit:
			v_current = -v_limit
		v_prima = (v_current - v_last)*self.dt
		v_last = v_current
		return v_current, v_last, v_prima

	def vUpdate(self, v):
		self.vCurrent, self.vLast, self.vPrima = v[0], v[1], v[2]
		return

	def wUpdate(self, w):
		self.wCurrent, self.wLast, self.wPrima = w[0], w[1], w[2]
		return

	def makeVelMsg(self):
		self.msgVel = Twist()
		if self.frc_z > 0.25 or self.frc_z < -0.25:
			self.msgVel.linear.x = self.vCurrent
			#self.msgVel.angular.z = self.wCurrent
		else:
			self.msgVel.linear.x = 0
			#self.msgVel.angular.z = 0
		if self.trq_y > 0.25 or self.trq_y < -0.25:
			#self.msgVel.linear.x = self.vCurrent
			self.msgVel.angular.z = self.wCurrent
		else:
			#self.msgVel.linear.x = 0
			self.msgVel.angular.z = 0
		self.pubFinalVel.publish(self.msgVel)

	def mainControl(self):
		self.rospy.loginfo("[%s] Admittance Controller OK", self.name)
		while not self.rospy.is_shutdown():
			if self.changeWrench or (self.changeFrc and self.changeTrq):
				self.vUpdate(self.getAdmittanceResponse(
									self.frc,
									self.vCurrent,
									self.vLast,
									self.vPrima,
									self.controllerParams["m"],
									self.controllerParams["b_l"],
									self.vLimit))
				self.wUpdate(self.getAdmittanceResponse(
									self.trq,
									self.wCurrent,
									self.wLast,
									self.wPrima,
									self.controllerParams["j"],
									self.controllerParams["b_a"],
									self.wLimit))
				self.makeVelMsg()
				self.changeWrench = self.changeFrc = self.changeTrq = False
			self.rate.sleep()

if __name__ == '__main__':
	try:
		sw = Control('admittance')
	except rospy.ROSInterruptException:
		pass
