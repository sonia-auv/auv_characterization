#!/usr/bin/env python

import rosbag
import argparse

from scipy import io




def parse_arguments():

    parser = argparse.ArgumentParser()
    parser.add_argument('--BagName', dest='bag_name')
    parser.add_argument('--outName', dest='output_name')
    parser.add_argument('--topic', dest='topic_name')
    parser.add_argument('--varName', dest='var_name')

    return parser.parse_args()


args = parse_arguments()
bag_name, output_name, topic_name, var_name = args.bag_name, args.output_name, args.topic_name, args.var_name
data = []

for topic, msg, t in rosbag.Bag(bag_name).read_messages():
    if topic == topic_name:
        data.append(msg.Z)

print var_name
dic = {var_name: data}
io.savemat(output_name, dic)
