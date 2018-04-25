#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Extract images from a .mp4 video
"""

import argparse
import uuid
import os
import logging
import sys
import tqdm
import cv2

from glob import glob


def set_logger(log_level=logging.INFO):
    """Configure the logger with log_level."""
    logging.basicConfig(
        format='%(asctime)s - %(levelname)s - %(name)s - %(message)s',
        level=log_level,
        stream=sys.stderr)
    logger = logging.getLogger("bag_to_image")
    logging.getLogger('requests').setLevel(logging.WARNING)

    return logger


OUTPUT_FOLDER_DEFAULT = os.path.join(os.getcwd(), 'extraction')


def parse_args():
    """
    Parse script arguments.
    :return: parsed args
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")

    parser.add_argument('-s', '--source_video_dir', required=True,
                        help="Directory containing .avi or .mp3 movie file")
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

    types = ('*.mp4', '*.MP4', '*.avi', '*.AVI')  # the tuple of file types
    video_files = []
    for files in types:
        video_files.extend(glob(files))

    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)

    for video_file in video_files:

        template_msg = 'Extract images from {}'
        msg = template_msg.format(video_file)

        logger.info(msg)

        vidcap = cv2.VideoCapture(video_file)
        success, image = vidcap.read()
        count = 0
        while success:
            success, image = vidcap.read()

            img_name = "frame_{}.jpg".format(generate_uuid1_name())
            extraction_path = os.path.join(args.output_dir, img_name)

            cv2.imwrite(extraction_path, image)     # save frame as JPEG file
            if cv2.waitKey(10) == 27:               # exit if Escape is hit
                break
            count += 1
            logger.info('Extracted image {} to {}'.format(img_name, extraction_path))


if __name__ == '__main__':
    logger = set_logger()
    main(logger)
    logger.info("Extraction of all bags complete with success")
