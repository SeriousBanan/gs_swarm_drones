#!/usr/bin/env python3

"""Module for communications with drones."""

from time import sleep
import subprocess
import json
import os

import rospy

# pylint: disable=import-error, no-name-in-module
from gs_swarm_drones.srv import TransferData, TransferDataResponse  # type: ignore
# pylint: enable=import-error, no-name-in-module

PKG_PATH = os.getenv("PKG_PATH")
if PKG_PATH:
    os.chdir(PKG_PATH)

def send_image(data) -> bool:
    """Send image to the operator
    Specify path to image and operator MAC address
    """

    try:
        subprocess.run(["scp", data.data, f"{CONFIGURATION['operator username']}@{CONFIGURATION['operator ip']}:~/swarm_drones_image/"])
    except BaseException:
        return TransferDataResponse(False)

    return TransferDataResponse(True)


if __name__ == '__main__':
    rospy.init_node("gs_swarm_drones_communication_node")

    _configuration_path = rospy.get_param(rospy.search_param("configuration_path"))

    with open(_configuration_path) as configuration_file:
        CONFIGURATION = json.load(configuration_file)

    image_service = rospy.Service("gs_swarm_drones/send_image", TransferData, send_image)

    rospy.spin()
