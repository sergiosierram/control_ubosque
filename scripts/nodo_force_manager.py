#!/usr/bin/python
import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Wrench
from std_srvs.srv import EmptyResponse, Empty
from threading import Lock

class ForceManager():
    def __init__(self, name):
        self.name = name
        rospy.init_node('ForceManager', anonymous = True)
        rospy.loginfo("[%s] Starting ForceManager", self.name)
        self.initParameters()
        self.initSubscribers()
        self.initPublishers()
        self.initServiceClients()
        self.initVariables()
        self.main()

    def initParameters(self):
        #Topicos
        self.theta_desired_topic = rospy.get_param("~theta_desired_topic","/theta_desired")
        self.force_topic = rospy.get_param("~force_topic", "/force")
        self.virtual_wrench_topic = rospy.get_param("~virtual_wrench_topic","/virtual_wrench")
        self.shared_wrench_topic = rospy.get_param("~shared_wrench_topic", "/shared_wrench")
        #Parametros
        self.d_sensors = rospy.get_param("~frc_sensors_distance", 0.8)
        self.k_virtual = rospy.get_param("~k_virtual_torque", 30)
        #Additional
        self.control_rate = rospy.get_param("~manager_rate",100)
        self.updateParamsService = self.name + rospy.get_param("~update_params_service", "/update_parameters")
        self.param_lock = Lock()
        return

    def initSubscribers(self):
        self.sub_theta_desired = rospy.Subscriber(self.theta_desired_topic, Float64, self.callbackTheta)
        self.sub_force = rospy.Subscriber(self.force_topic, Wrench, self.callbackFrc)
        return

    def initPublishers(self):
        self.pub_virtual_wrench = rospy.Publisher(self.virtual_wrench_topic, Wrench, queue_size = 10)
        self.pub_shared_wrench = rospy.Publisher(self.shared_wrench_topic, Wrench, queue_size = 10)
        return

    def initServiceClients(self):
        rospy.Service(self.updateParamsService, Empty, self.callbackUpdateParams)
        return

    def initVariables(self):
        self.change_theta = self.change_frc = False
        self.rate = rospy.Rate(self.control_rate)
        return

    def callbackUpdateParams(self, req):
        with self.param_lock:
            self.initParameters()
            rospy.loginfo("[%s] Parameter update after request", self.name)
        return EmptyResponse()

    def callbackTheta(self, msg):
        theta = msg.data
        f1 = (1 + np.tanh(theta))*self.k_virtual
        f2 = (1 - np.tanh(theta))*self.k_virtual
        self.virtual_trq = (f2 - f1)*self.d_sensors/2.0
        msg = Wrench()
        msg.torque.z = self.virtual_trq
        self.pub_virtual_wrench.publish(msg)
        self.change_theta = True
        return

    def callbackFrc(self, msg):
        self.force = msg.force.y
        self.torque = -1*msg.torque.z
        self.change_frc = True
        return

    def make_shared_wrench(self):
        msg = Wrench()
        msg.force.y = self.force
        msg.force.z = 0.5 #Constante para simular el usuario
        msg.torque.y = 0.5 #Constante para simular el usuario
        msg.torque.z = self.virtual_trq + self.torque
        self.pub_shared_wrench.publish(msg)
        return

    def main(self):
        rospy.loginfo("[%s] Force Manager OK", self.name)
        while not rospy.is_shutdown():
            if self.change_frc and self.change_theta:
                self.make_shared_wrench()
                self.change_frc = self.change_theta = False
            self.rate.sleep()

if __name__ == '__main__':
	try:
		sw = ForceManager('force_manager')
	except rospy.ROSInterruptException:
		pass
