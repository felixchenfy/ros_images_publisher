#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header

import numpy as np
import cv2
import open3d
import time
import argparse
import os
import sys
import queue
import simplejson
import glob
from abc import ABCMeta, abstractmethod  # Abstract class.

''' ============================= Settings ============================= '''
ROOT = os.path.dirname(os.path.abspath(__file__))+'/'

FRAME_ID = "head_camera"  # Name of the camera frame.

IMAGE_SUFFIXES = ["png", "jpg"]  # What kind of image to read from folder.

APPEND_RANDOM_NUMBER_TO_NODE_NAME = True  # Make node name unique.


def get_camera_info_topic_name(image_topic_name):
    if "/" not in image_topic_name:
        camera_info_topic_name = "camera_info"
    else:
        idx = image_topic_name.rfind("/")
        camera_info_topic_name = image_topic_name[:idx+1] + "camera_info"
    return camera_info_topic_name


def parse_command_line_arguments():

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description="Publish color or depth images from a folder to ROS topic.")
    parser.add_argument("-i", "--images_folder", required=True, type=str,
                        default=ROOT+"data/color/",
                        help="A data folder which contains .jpg or .png images for publishing.")
    parser.add_argument("-c", "--camera_info_file", required=False, type=str,
                        default=ROOT+"data/cam_params_realsense.json",
                        help="File path of camera parameters. No distortion.")
    parser.add_argument("-t", "--topic_name", required=False, type=str,
                        default="test_data/image_raw",
                        help="ROS topic name to publish image.")
    parser.add_argument("-r", "--publish_rate", required=False, type=float,
                        default=1.0,
                        help="How many images to publish per second.")
    parser.add_argument("-f", "--format", required=False, type=str,
                        choices=["color", "depth"],
                        default="color",
                        help="Format of image: color (uint8, 3 channels), or depth (uint16, 1 channel).")
    args = parser.parse_args(rospy.myargv()[1:])
    return args


''' ============================= Helper functions ============================= '''


def read_json_file(file_path):
    with open(file_path, 'r') as f:
        data = simplejson.load(f)
    return data


def create_header(frame_id):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    return header


def is_int(num):
    ''' Is floating number very close to a int. '''
    # print(is_int(0.0000001)) # False
    # print(is_int(0.00000001)) # True
    return np.isclose(np.round(num), num)


def get_filenames(folder, suffixes=IMAGE_SUFFIXES):
    filenames = []
    for suffix in suffixes:
        filenames.extend(glob.glob(folder + "/*." + suffix))
    return sorted(filenames)


''' ============================= Publishers ============================= '''


class AbstractImagePublisher(object):
    __metaclass__ = ABCMeta

    def __init__(self, image_topic,
                 queue_size=10):
        self._pub = rospy.Publisher(image_topic, Image, queue_size=queue_size)
        self._cv_bridge = CvBridge()

    def publish(self, image, frame_id="head_camera"):
        ros_image = self._to_ros_image(image)
        ros_image.header = create_header(frame_id)
        self._pub.publish(ros_image)

    @abstractmethod
    def _to_ros_image(self, image):
        pass


class ColorImagePublisher(AbstractImagePublisher):

    def __init__(self, topic_name, queue_size=10):
        super(ColorImagePublisher, self).__init__(
            topic_name, queue_size)

    def _to_ros_image(self, cv2_uint8_image, img_format="bgr"):

        # -- Check input.
        shape = cv2_uint8_image.shape  # (row, col, depth=3)
        assert(len(shape) == 3 and shape[2] == 3)

        # -- Convert image to bgr format.
        if img_format == "rgb":  # If rgb, convert to bgr.
            bgr_image = cv2.cvtColor(cv2_uint8_image, cv2.COLOR_RGB2BGR)
        elif img_format == "bgr":
            bgr_image = cv2_uint8_image
        else:
            raise RuntimeError("Wrong image format: " + img_format)

        # -- Convert to ROS format.
        ros_image = self._cv_bridge.cv2_to_imgmsg(bgr_image, "bgr8")
        return ros_image


class DepthImagePublisher(AbstractImagePublisher):

    def __init__(self, topic_name, queue_size=10):
        super(DepthImagePublisher, self).__init__(
            topic_name, queue_size)

    def _to_ros_image(self, cv2_uint16_image):

         # -- Check input.
        shape = cv2_uint16_image.shape  # (row, col)
        assert(len(shape) == 2)
        assert(type(cv2_uint16_image[0, 0] == np.uint16))

        # -- Convert to ROS format.
        ros_image = self._cv_bridge.cv2_to_imgmsg(cv2_uint16_image, "16UC1")
        return ros_image


class MyCameraInfo():

    def __init__(self, camera_info_json_file_path):
        data = read_json_file(camera_info_json_file_path)
        self._width = int(data["width"])  # int.
        self._height = int(data["height"])  # int.
        self._intrinsic_matrix = data["intrinsic_matrix"]  # list of float.
        # The list extracted from the matrix **column by column** !!!.
        # If the intrinsic matrix is:
        # [fx,  0, cx],
        # [ 0, fy, cy],
        # [ 0,  0,  1],
        # Then, self._intrinsic_matrix = [fx, 0, 0, 0, fy, 0, cx, cy, 1]

    def resize(self, ratio):
        r0, c0 = self._height, self._width
        if not (is_int(r0*ratio) and is_int(c0*ratio)):
            raise RuntimeError(
                "Only support resizing image to an interger size.")
        self._width = int(ratio * self._width)
        self._height = int(ratio * self._height)
        self._intrinsic_matrix[:-1] = [x*ratio
                                       for x in self._intrinsic_matrix[:-1]]

    def width(self):
        return self._width

    def height(self):
        return self._height

    def intrinsic_matrix(self):
        return self._intrinsic_matrix

    def get_cam_params(self):
        ''' Get all camera parameters.
        Notes: intrinsic_matrix:
            [0]: fx, [3]   0, [6]:  cx
            [1]:  0, [4]: fy, [7]:  cy
            [2]:  0, [5]   0, [8]:   1
        '''
        im = self._intrinsic_matrix
        row, col = self._height, self._width
        fx, fy, cx, cy = im[0], im[4], im[6], im[7]
        return row, col, fx, fy, cx, cy


class CameraInfoPublisher():
    ''' Publish image size and camera instrinsics to ROS CameraInfo topic.
        The distortion is not considered.
    '''

    def __init__(self, topic_name):
        # Set publisher.
        self._pub = rospy.Publisher(topic_name, CameraInfo, queue_size=5)

        # Create default camera info:
        camera_info = CameraInfo()
        camera_info.distortion_model = "plumb_bob"
        camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._default_camera_info = camera_info

    def publish(self, width, height, intrinsic_matrix, frame_id="head_camera"):
        ''' Publish camera_info constructed by:
                (1) width
                (2) height
                (3) intrinsic_matrix
        Arguments:
            intrinsic_matrix {1D list or 2D list}:
                If 1D list, the data order is: column1, column2, column3.
        '''
        # -- Set camera info.
        camera_info = self._default_camera_info
        camera_info.header = create_header(frame_id)
        self._set_size_and_intrinsics(
            camera_info, width, height, intrinsic_matrix)

        # -- Publish.
        self._pub.publish(camera_info)

    def _2d_array_to_list(self, intrinsic_matrix):
        res = list()
        for i in range(3):
            for j in range(3):
                res.append(intrinsic_matrix[i][j])
        return res

    def _set_size_and_intrinsics(self, camera_info, width, height, intrinsic_matrix):
        camera_info.height = height
        camera_info.width = width
        if isinstance(intrinsic_matrix, list):
            K = intrinsic_matrix
        else:
            K = self._2d_array_to_list(intrinsic_matrix)
        camera_info.K = K

        camera_info.P = [
            K[0], K[1], K[2], 0.0,
            K[3], K[4], K[5], 0.0,
            K[6], K[7], K[8], 0.0,
        ]


''' ============================= Subscribers (Not used) ============================= '''


class AbstractImageSubscriber(object):
    __metaclass__ = ABCMeta

    def __init__(self, topic_name, queue_size=2):
        self._cv_bridge = CvBridge()
        self._sub = rospy.Subscriber(
            topic_name, Image, self._callback_of_image_subscriber)
        self._imgs_queue = queue.Queue(maxsize=queue_size)

    @abstractmethod
    def _convert_ros_image_to_desired_image_format(self, ros_image):
        pass

    def get_image(self):
        ''' Get the next image subscribed from ROS topic,
            convert to desired opencv format, and then return. '''
        if not self.has_image():
            raise RuntimeError("Failed to get_image().")
        ros_image = self._imgs_queue.get(timeout=0.05)
        dst_image = self._convert_ros_image_to_desired_image_format(ros_image)
        return dst_image

    def has_image(self):
        return self._imgs_queue.qsize() > 0

    def _callback_of_image_subscriber(self, ros_image):
        ''' Save the received image into queue.
        '''
        if self._imgs_queue.full():  # If queue is full, pop one.
            img_to_discard = self._imgs_queue.get(timeout=0.001)
        self._imgs_queue.put(ros_image, timeout=0.001)  # Push image to queue


class ColorImageSubscriber(AbstractImageSubscriber):
    ''' RGB image subscriber. '''

    def __init__(self, topic_name, queue_size=2):
        super(ColorImageSubscriber, self).__init__(topic_name, queue_size)

    def _convert_ros_image_to_desired_image_format(self, ros_image):
        ''' To np.ndarray np.uint8 BGR format. '''
        return self._cv_bridge.imgmsg_to_cv2(ros_image, "bgr8")


class DepthImageSubscriber(AbstractImageSubscriber):
    ''' Depth image subscriber. '''

    def __init__(self, topic_name, queue_size=2):
        super(DepthImageSubscriber, self).__init__(topic_name, queue_size)

    def _convert_ros_image_to_desired_image_format(self, ros_image):
        ''' To np.ndarray np.uint16 format. '''
        return self._cv_bridge.imgmsg_to_cv2(ros_image, "16UC1")  # not 32FC1


class CameraInfoSubscriber(object):

    def __init__(self, topic_name):
        self._camera_info = None
        self._sub = rospy.Subscriber(
            topic_name, CameraInfo, self._callback)

    def get_camera_info(self):
        if self._camera_info is None:
            raise RuntimeError("Failed to get_camera_info().")
        camera_info = self._camera_info
        self._camera_info = None  # Reset data.
        return camera_info

    def has_camera_info(self):
        return self._camera_info != None

    def _callback(self, camera_info):
        self._camera_info = camera_info


''' ============================= Main ============================= '''


def main(args):

    # -- Read images filenames.
    images_filenames = get_filenames(
        args.images_folder, suffixes=IMAGE_SUFFIXES)
    if len(images_filenames) == 0:
        raise RuntimeError("Image folder has no image. "
                           "Check this folder again: " + args.images_folder)
    rospy.loginfo("There are {} images in: {}".format(
        len(images_filenames), args.images_folder))

    # -- Create image publisher.
    img_topic_name = args.topic_name
    if args.format == "color":
        img_publisher = ColorImagePublisher(img_topic_name)
    else:  # "depth"
        img_publisher = DepthImagePublisher(img_topic_name)
    rospy.loginfo("Publish {} image to: {}".format(
        args.format, img_topic_name))

    # -- Create camera_info publisher.
    if not args.camera_info_file:
        rospy.logwarn("Camera info file is empty. Not publish it.")
        camera_info = None
    elif not os.path.exists(args.camera_info_file):
        rospy.logwarn("Camera info file is empty. Not publish it.")
        camera_info = None
    else:
        camera_info = MyCameraInfo(args.camera_info_file)
        rospy.loginfo("Load camera info from: " + args.camera_info_file)

        camera_info_topic_name = get_camera_info_topic_name(img_topic_name)
        rospy.loginfo("Publish camera info to: " + camera_info_topic_name)

        camera_info_publisher = CameraInfoPublisher(camera_info_topic_name)

    # -- Loop and publish.
    loop_rate = rospy.Rate(args.publish_rate)

    num_total_imgs = len(images_filenames)
    cnt_imgs = 0
    ith_img = 0

    while not rospy.is_shutdown():

        # -- Read next image.
        if ith_img == num_total_imgs:
            ith_img = 0
        image = cv2.imread(images_filenames[ith_img], cv2.IMREAD_UNCHANGED)
        cnt_imgs += 1
        ith_img += 1

        # -- Publish data.
        rospy.loginfo("=================================================")
        rospy.loginfo("Publish {}/{}th data; {} published in total.".format(
            ith_img, num_total_imgs, cnt_imgs))

        img_publisher.publish(image, FRAME_ID)
        if camera_info is not None:
            camera_info_publisher.publish(
                camera_info.width(),
                camera_info.height(),
                camera_info.intrinsic_matrix(),
                FRAME_ID)

        # -- Wait for publish.
        loop_rate.sleep()


if __name__ == '__main__':
    node_name = "publish_images"
    if APPEND_RANDOM_NUMBER_TO_NODE_NAME:
        node_name += "_" + str(np.random.randint(low=0, high=99999999999))
    rospy.init_node(node_name)
    args = parse_command_line_arguments()
    main(args)
    rospy.logwarn("Node `{}` stops.".format(node_name))
