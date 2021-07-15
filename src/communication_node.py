#!/usr/bin/env python3

"""Module for communications with drones."""

import json
import os
import socket
from typing import NoReturn

import rospy

# pylint: disable=import-error, no-name-in-module
from gs_swarm_drones.srv import TransferData, TransferDataResponse  # type: ignore
# pylint: enable=import-error, no-name-in-module

PKG_PATH = os.getenv("PKG_PATH")
if PKG_PATH:
    os.chdir(PKG_PATH)

BUFFER_SIZE = 4096


def send_message(data: TransferData) -> TransferDataResponse:
    """Send message to all other devices in the network using TCP/IP."""

    try:
        # Iterate all devices in network and send them message.
        for id_ in CONFIGURATION["addresses"]:
            if id_ == str(CONFIGURATION["drone id"]):
                continue

            # Creating socket
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                server_address = (CONFIGURATION["addresses"][id_]["ip"],
                                  CONFIGURATION["addresses"][id_]["port"])
                sock.connect(server_address)

                bytes_ = data.data.encode()

                message_info = {
                    "type": "text",
                    "size": len(bytes_),
                }

                # Sending info about message until device got it.
                answer = b""
                while answer != b"got message info":
                    sock.sendall(json.dumps(message_info).encode())

                    answer = sock.recv(BUFFER_SIZE)

                # Sending message until device got it.
                answer = b""
                while answer != b"got message":
                    sock.sendall(bytes_)
                    sock.sendall(b"$finished$")

                    answer = sock.recv(BUFFER_SIZE)

                sock.sendall(b"$close$")

    except socket.error:
        return TransferDataResponse(False)

    return TransferDataResponse(True)


def send_image(data: TransferData) -> TransferDataResponse:
    """Send image to the operator
    Specify path to image and operator MAC address
    """

    try:
        # Creating socket
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            server_address = (CONFIGURATION["addresses"]["operator"]["ip"],
                              CONFIGURATION["addresses"]["operator"]["port"])
            sock.connect(server_address)

            with open(data.data, "rb") as image_file:
                bytes_ = image_file.read()

            message_info = {
                "type": "image",
                "size": len(bytes_),
            }

            # Sending info about message until device got it.
            answer = b""
            while answer != b"got message info":
                sock.sendall(json.dumps(message_info).encode())

                answer = sock.recv(BUFFER_SIZE)

            # Sending message until device got it.
            answer = b""
            while answer != b"got message":
                sock.sendall(bytes_)
                sock.sendall(b"$finished$")

                answer = sock.recv(BUFFER_SIZE)

            sock.sendall(b"$close$")

    except socket.error:
        return TransferDataResponse(False)

    return TransferDataResponse(True)


def listening() -> NoReturn:
    """Create listener on socket and receive data."""

    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    drone_id = str(CONFIGURATION["drone id"])
    server_address = (CONFIGURATION["addresses"][drone_id]["ip"],
                      CONFIGURATION["addresses"][drone_id]["port"])

    server_sock.bind(server_address)

    # Listening to incoming connections.
    server_sock.listen(10)

    while True:
        try:
            # Waiting for connection.
            sock, _ = server_sock.accept()
            message_info = {}  # type: ignore
            message = b""
            finished = False

            with sock:
                # Receiving data.
                while True:
                    data = sock.recv(BUFFER_SIZE)

                    # Firstly get info about message.
                    if not message_info:
                        message_info = json.loads(data.decode())
                        sock.sendall(b"got message info")

                    # Close connection after receiving whole message or if connection broken.
                    elif data == b"$close$" or not data:
                        sock.close()
                        break

                    elif not finished:
                        if data[-10:] == b"$finished$":
                            data = data[:-10]
                            finished = True

                        message += data

                    # If not all data received send error.
                    if finished and len(message) != message_info["size"]:
                        sock.sendall(b"error while receiving message")
                        message = b""
                        finished = False

                    # Send to device message that all received.
                    elif finished:
                        sock.sendall(b"got message")

            # Processing text
            if message_info["type"] == "text":
                responce = False
                while not responce:
                    responce = send_received_data(message.decode()).status

            # Processing image
            elif message_info["type"] == "image":
                with open(CONFIGURATION["image file path"], "wb") as image_file:
                    image_file.write(message)
        except socket.error:
            server_sock.close()
            server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_sock.bind(server_address)

            # Listening to incoming connections.
            server_sock.listen(10)


if __name__ == '__main__':
    rospy.init_node("communication_node")

    _configuration_path = rospy.get_param(rospy.search_param("configuration_path"))

    with open(_configuration_path) as configuration_file:
        CONFIGURATION = json.load(configuration_file)

    message_service = rospy.Service("gs_swarm_drones/send_message", TransferData, send_message)
    image_service = rospy.Service("gs_swarm_drones/send_image", TransferData, send_image)

    rospy.wait_for_service("gs_swarm_drones/receive_message")
    send_received_data = rospy.ServiceProxy("gs_swarm_drones/receive_message", TransferData)

    listening()
