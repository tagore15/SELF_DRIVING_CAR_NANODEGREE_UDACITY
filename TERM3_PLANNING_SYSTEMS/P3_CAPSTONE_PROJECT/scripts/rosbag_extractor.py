"""
Simple script for extracting images from rosbags to a directory
"""

import os
import shutil

import rosbag
import cv_bridge
import cv2
import tqdm


def main():

    bag_path = "/home/student/Downloads/traffic_light_bag_files/loop_with_traffic_light.bag"

    output_dir_path = "/tmp/images/"

    shutil.rmtree(output_dir_path, ignore_errors=True)

    if not os.path.exists(output_dir_path):
        os.makedirs(output_dir_path)

    bridge = cv_bridge.CvBridge()

    with rosbag.Bag(bag_path) as bag:

        messages = list(bag.read_messages(topics="/image_raw"))

        padding = len(str(len(messages))) + 1

        for index, message in enumerate(tqdm.tqdm(messages)):

            data = bridge.imgmsg_to_cv2(message.message, "bgr8")

            file_name = str(index).zfill(padding) + ".jpg"
            path = os.path.join(output_dir_path, file_name)
            cv2.imwrite(path, data)


if __name__ == "__main__":

    main()