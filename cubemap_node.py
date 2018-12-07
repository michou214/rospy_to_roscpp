#!/usr/bin/env python

import sys

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo


class CubemapNode:

    def __init__(self):

        # Initialize node
        rospy.init_node('image_node', argv=sys.argv)
        rospy.loginfo('Image node initialized')

        self.bridge = CvBridge()

        camera_names = ['left', 'front', 'right', 'back', 'top', 'bottom']
        self.image_subs = []
        for i, name in enumerate(camera_names):
            topic = 'camera_{}/image_raw'.format(name)
            image_sub = rospy.Subscriber(topic, Image,
                                         callback=self.image_callback,
                                         callback_args=i,
                                         queue_size=1)
            self.image_subs.append(image_sub)

        self.camera_subs = []
        for i, name in enumerate(camera_names):
            topic = 'camera_{}/camera_info'.format(name)
            camera_sub = rospy.Subscriber(topic, CameraInfo,
                                          callback=self.camera_callback,
                                          callback_args=i,
                                          queue_size=1)
            self.camera_subs.append(camera_sub)

        self.images = [None for _ in camera_names]
        self.cameras = [None for _ in camera_names]

        img_pub_topic = 'camera_concat/image_raw'
        self.img_pub = rospy.Publisher(img_pub_topic, Image, queue_size=1)

        self.hz = 10
        self.rate = rospy.Rate(self.hz)

        self.step = 0

        rospy.spin()

    def image_callback(self, img_msg, index):
        bgr_img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        self.images[index] = bgr_img

        if index == 0 and self.step > len(self.camera_subs):
            self.publish_image()

        self.step += 1

    def camera_callback(self, cam_msg, index):
        self.cameras[index] = cam_msg
        self.camera_subs[index].unregister()

    def publish_image(self):
        concat_image = np.concatenate(self.images, axis=1)
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(concat_image, 'bgr8'))
        self.rate.sleep()


def main():
    CubemapNode()


if __name__ == '__main__':
    main()
