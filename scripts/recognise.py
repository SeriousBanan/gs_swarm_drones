#!/usr/bin/env python3

"""Module that creates servise for recognising tanks using camera."""

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# pylint: disable=import-error, no-name-in-module
from gs_swarm_drones.srv import Recognise, RecogniseResponse  # type: ignore
# pylint: enable=import-error, no-name-in-module


try:
    from .detector.detector import Detector
except ImportError:
    from detector.detector import Detector  # type: ignore

detector = Detector(0.3)
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


rospy.init_node("gs_swarm_drones_recognise_node")

rospy.Subscriber("pioneer_max_camera/image_raw", Image, img_callback)

recognise_service = rospy.Service("gs_swarm_drones/recognise", Recognise, detect_tank)

rospy.spin()
