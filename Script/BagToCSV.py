#!/usr/bin/env python

import rospy
import os
from scipy import io
from interface_rs485.msg import SendRS485Msg
from proc_control.msg import ThrustLogging
from geometry_msgs.msg import TwistStamped
from matplotlib import pyplot as plt


class BagToCsv:

    def __init__(self):

        home = os.path.expanduser("~")

        self.csv_file = open(home + "/parse.csv", "w")

        rospy.init_node("proc_control_csv")

        self.msg_list = []

        self.subscriber = rospy.Subscriber("/proc_control/thrust_logging", ThrustLogging, self.callback)

        self.loop()

    def loop(self):

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()

        dic = {'SPEED': self.msg_list}
        io.savemat('thrust_y_25.mat', dic)

    def callback(self, msg):

        self.msg_list.append(msg.thrust_axe)




if __name__ == "__main__":
    element = BagToCsv()