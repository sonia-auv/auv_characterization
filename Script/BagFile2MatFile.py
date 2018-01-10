#!/usr/bin/env python

import struct
import rosbag
import argparse
import numpy as np

from scipy import io




def parse_arguments():

    parser = argparse.ArgumentParser()
    parser.add_argument('--BagName', dest='bag_name')
    parser.add_argument('--command', dest='command')
    parser.add_argument('--outName', dest='output_name')
    parser.add_argument('--axis', dest='axis')

    return parser.parse_args()


args = parse_arguments()
bag_name, command, output_name, axis = args.bag_name, int(args.command), args.output_name, int(args.axis)
data_thrust, data_dvl, data_imu = [], [], []

begin_time = 0
finish_time = 0
start = False
for topic, msg, t in rosbag.Bag(bag_name).read_messages():
    if topic == '/interface_rs485/dataRx' and msg.slave == 16 and not start:
        data = list(struct.unpack("{}B".format(len(msg.data)), msg.data))
        if data[axis] - 100 == command:
            begin_time = t
            start = True
        if start and data[axis] - 100 != command:
            finish_time = t
            start = False

    if start:
        if topic == '/interface_rs485/dataRx' and msg.slave == 16:
            data_thrust.append([float(list(struct.unpack("{}B".format(len(msg.data)), msg.data))[axis] - 100)])
        if topic == '/provider_dvl/dvl_twist':
            data_dvl.append([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        if topic == '/provider_imu/twist':
            data_imu.append([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])

data_thrust = []
for i in data_dvl:
    data_thrust.append([10.0])


dic = {'Thrust': data_thrust, 'LINEAR_SPEED': data_dvl, 'ANGULAR_SPEED': data_imu, 'T': (t - begin_time).secs}
io.savemat(output_name, dic)
