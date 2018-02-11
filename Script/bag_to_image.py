#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag.
"""

import argparse
import uuid
import cv2
import rosbag
from cv_bridge import CvBridge


def generate_uuid1_name():
    return str(uuid.uuid1())


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

    for bag_file in bag_file_list:
        msg = 'Extract images from {} on topic {}'.\
              format(bag_file=bag_file,topic=args.image_topic)
        print(msg)
        
        bag = rosbag.Bag(bag_file, "r")
        bridge = CvBridge()
        for topic, msg, t in bag.read_messages(topics=[args.image_topic]):
            pic_name = "frame" + generate_uuid1_name() + ".jpg"
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            cv2.imwrite(pic_name, cv_img)
    return


if __name__ == '__main__':
    main()
