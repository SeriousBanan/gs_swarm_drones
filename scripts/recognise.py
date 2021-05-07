#!/usr/bin/env python3

"""Module that creates servise for recognising tanks using camera."""

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# pylint: disable=import-error, no-name-in-module
from gs_swarm_drones.srv import Recognise, RecogniseResponse  # type: ignore
# pylint: enable=import-error, no-name-in-module

import os

try:
    from .detector.detector import Detector
except ImportError:
    from detector.detector import Detector  # type: ignore

detector = Detector(0.5)
img = []
bridge = CvBridge()
count_missed = 0


def img_callback(data):
    """Get image from camera."""
    global img
    global count_missed
    try:
        img = bridge.imgmsg_to_cv2(data)
        count_missed = 0
    except BaseException:
        img = []
        count_missed += 1
    with open(f"{os.getenv('PKG_PATH')}/___missed_counter.txt", "a") as f:
        print(count_missed, file=f)


def detect_tank(_):
    """Detect tank using camera."""
    with open(f"{os.getenv('PKG_PATH')}/___type_of_image.txt", "a") as f:
        print(type(img), file=f)

    flag, path = detector.detect(img)
    return RecogniseResponse(flag, path)


rospy.init_node("gs_swarm_drones_recognise_node")

rospy.Subscriber("pioneer_max_camera/image_raw", Image, img_callback)

recognise_service = rospy.Service("gs_swarm_drones/recognise", Recognise, detect_tank)

rospy.spin()
