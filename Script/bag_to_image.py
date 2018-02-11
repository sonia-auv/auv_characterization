#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag.
"""

import argparse
import uuid
import os
import logging
import sys
import tqdm
import cv2
import rosbag
from cv_bridge import CvBridge

#TODO: CREATE DOC FOR THIS MODULES (COMMAND EXAMPLE)

IMAGE_TOPIC_DEFAULT = '/provider_vision/Front_GigE/compressed'
OUTPUT_FOLDER_DEFAULT = os.path.join(os.getcwd(), 'extraction')


def set_logger(log_level=logging.INFO):
    """Configure the logger with log_level."""
    logging.basicConfig(
        format='%(asctime)s - %(levelname)s - %(name)s - %(message)s',
        level=log_level,
        stream=sys.stderr)
    logger = logging.getLogger("bag_to_image")
    logging.getLogger('requests').setLevel(logging.WARNING)

    return logger



def parse_args():
    """
    Parse script arguments.
    :return: parsed args
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")

    parser.add_argument('-s', '--source_bag_dir', required=True,
                        help="Directory containing ROS bag.")
    parser.add_argument('-t', '--image_topic', default=IMAGE_TOPIC_DEFAULT,
                        help="Image topic.")
    parser.add_argument('-o', '--output_dir', default=OUTPUT_FOLDER_DEFAULT,
                        help="Output directory")

    return parser.parse_args()

def generate_uuid1_name():
    """
    Generate a unique identifier for each generated images.
    :return: uuid 1 identifier
    """
    return str(uuid.uuid1())


def main(logger):
    """
    Extract a folder of images from a rosbag.
    """

    args = parse_args()

    bag_file_list = os.listdir(args.source_bag_dir)

    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)

    bridge = CvBridge()

    for bag_file in bag_file_list:


        template_msg = 'Extract images from {} on topic {}'
        msg = template_msg.format(bag_file, args.image_topic)

        logger.info(msg)

        bag_path = os.path.join(args.source_bag_dir, bag_file)

        with rosbag.Bag(bag_path, "r") as bag:
            for topic, msg, _ in bag.read_messages(topics=[args.image_topic]):

                img_name = "frame_{}.jpg".format(generate_uuid1_name())
                extraction_path = os.path.join(args.output_dir, img_name)

                cv_img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
                cv2.imwrite(extraction_path, cv_img)
                logger.info('Extracted image {} to {}'.format(img_name, extraction_path))


if __name__ == '__main__':
    logger = set_logger()
    main(logger)
    logger.info("Extraction of all bags complete with success")
