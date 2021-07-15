#!/usr/bin/env python3

"""Module that creates servise for recognising tanks using camera."""

import cv2 as _
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from gs_swarm_drones.srv import Recognise, RecogniseResponse

from scripts.detector.detector import Detector

detector = Detector(0.5)
img = []
bridge = CvBridge()


def img_callback(data):
    """Get image from camera."""

    global img
    try:
        img = bridge.imgmsg_to_cv2(data)
    except BaseException:
        img = []


def detect_tank(_):
    """Detect tank using camera."""

    flag, path = detector.detect(img)
    return RecogniseResponse(flag, path)


rospy.init_node("recognise_node")

rospy.Subscriber("pioneer_max_camera/image_raw", Image, img_callback)

recognise_service = rospy.Service("gs_swarm_drones/recognise", Recognise, detect_tank)

rospy.spin()
