#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag.
"""

import os
import argparse

import cv2
from scipy.io import savemat
from matplotlib import pyplot as plt
from skimage.transform import resize

import rosbag
from cv_bridge import CvBridge


def main():
    """Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("--bag_file", help="Input ROS bag.")
    parser.add_argument("--image_topic", help="Image topic.")
    parser.add_argument("--output_file", help="Out put file")
    parser.add_argument("--label", help="Out put file")

    args = parser.parse_args()

    bag_file_list = args.bag_file.split()
    label_list = args.label.split()

    #exec ('bag_file_list, label_list = {}'.format(args.bag_file))

    count, skip_image = 0, 0
    img_list, lab_list = [], []
    for bag_file, label in zip(bag_file_list, label_list):
        print "Extract images from %s on topic %s" % (bag_file, args.image_topic)
        bag = rosbag.Bag(bag_file, "r")
        bridge = CvBridge()
        for topic, msg, t in bag.read_messages(topics=[args.image_topic]):
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            if skip_image == 2:
                cv_img = resize(cv_img, (150, 150, 3))
                skip_image = 0
                img_list.append(cv_img.flatten())
                lab_list.append(int(label))
                print "Wrote image %i" % count
                count += 1

            skip_image += 1
        bag.close()
    dic = {'X': img_list, 'Y': lab_list}
    savemat(args.output_file, dic)


    return

if __name__ == '__main__':
    main()