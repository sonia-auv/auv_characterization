#!/usr/bin/env python

import rospy
import os
from scipy import io
from proc_control.msg import ThrustLogging
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped


class BagToCsv:

    def __init__(self):

        home = os.path.expanduser("~")

        self.csv_file = open(home + "/parse.csv", "w")

        rospy.init_node("proc_control_csv")

        self.msg_list_thrust, self.msg_list_dvl, self.msg_list_speed_imu, self.msg_list_acc_imu = [], [], [], []

        self.subscriber = rospy.Subscriber("/proc_control/thrust_logging", ThrustLogging, self.callback_thruster)
        self.subscriber = rospy.Subscriber("/provider_dvl/dvl_twist", TwistStamped, self.callback_dvl)
        self.subscriber = rospy.Subscriber("/provider_imu/twist", TwistWithCovarianceStamped, self.callback_speed_imu)

        self.loop()

    def loop(self):

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()

        dic = {'Thrust': self.msg_list_thrust, 'LINEAR_SPEED': self.msg_list_dvl, 'ANGULAR_SPEED': self.msg_list_speed_imu}
        io.savemat('yaw_20_1.mat', dic)

    def callback_thruster(self, msg):
        #thruster logging
        self.msg_list_thrust.append(msg.thrust_axe)

    def callback_dvl(self, msg):
        #dvl (speed) logging
        self.msg_list_dvl.append([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])

    def callback_speed_imu(self, msg):
        #imu (speed) logging
        self.msg_list_speed_imu.append([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])








if __name__ == "__main__":
    element = BagToCsv()