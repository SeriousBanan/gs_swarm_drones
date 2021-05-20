#!/usr/bin/env python3

"""Module for communications with drones."""

import socket
import json
import os
from typing import Callable
from threading import Thread

_BUFFER_SIZE = 4096

with open("../configuration.json") as _configuration_file:
    _CONFIGURATION = json.load(_configuration_file)


def send_message_to_all(data: str) -> bool:
    """Send message to all other devices in the network using TCP/IP."""

    try:
        # Iterate all devices in network and send them message.
        for id_ in _CONFIGURATION["addresses"]:
            if id_ == "operator":
                continue

            # Creating socket
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                server_address = (_CONFIGURATION["addresses"][id_]["ip"],
                                  _CONFIGURATION["addresses"][id_]["port"])
                sock.connect(server_address)

                bytes_ = data.encode()

                message_info = {
                    "type": "text",
                    "size": len(bytes_),
                }

                # Sending info about message until device got it.
                answer = b""
                while answer != b"got message info":
                    sock.sendall(json.dumps(message_info).encode())

                    answer = sock.recv(_BUFFER_SIZE)

                # Sending message until device got it.
                answer = b""
                while answer != b"got message":
                    sock.sendall(bytes_)
                    sock.sendall(b"$finished$")

                    answer = sock.recv(_BUFFER_SIZE)

                sock.sendall(b"$close$")

    except socket.error:
        return False

    return True


def send_message_to_drone(data: str, drone_id: int) -> bool:
    """Send message to drone in the network using TCP/IP."""

    try:
        # Creating socket
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            server_address = (_CONFIGURATION["addresses"][str(drone_id)]["ip"],
                              _CONFIGURATION["addresses"][str(drone_id)]["port"])
            sock.connect(server_address)

            bytes_ = data.encode()

            message_info = {
                "type": "text",
                "size": len(bytes_),
            }

            # Sending info about message until device got it.
            answer = b""
            while answer != b"got message info":
                sock.sendall(json.dumps(message_info).encode())

                answer = sock.recv(_BUFFER_SIZE)

            # Sending message until device got it.
            answer = b""
            while answer != b"got message":
                sock.sendall(bytes_)
                sock.sendall(b"$finished$")

                answer = sock.recv(_BUFFER_SIZE)

            sock.sendall(b"$close$")

    except socket.error:
        return False

    return True


def create_listener(callback: Callable[[str, str], None]) -> None:
    """
    Create listener on socket and receive data.

    callback must have two arguments:
    1. type of receiving message ("text" or "image_path").
    2. received message or path to received image.
    """

    if getattr(create_listener, "listener_created", True):
        return
    setattr(create_listener, "listener_created", False)
    setattr(create_listener, "tread_stopped", False)

    def listener() -> None:
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        server_address = (_CONFIGURATION["addresses"]["operator"]["ip"],
                          _CONFIGURATION["addresses"]["operator"]["port"])

        server_sock.bind(server_address)

        # Listening to incoming connections.
        server_sock.listen(10)

        while not getattr(create_listener, "tread_stopped", True):
            # Waiting for connection.
            sock, _ = server_sock.accept()
            message_info = {}  # type: ignore
            message = b""
            finished = False

            with sock:
                # Receiving data.
                while True:
                    data = sock.recv(_BUFFER_SIZE)

                    # Close connection after receiving whole message or if connection broken.
                    if data == b"$close$" or not data:
                        sock.close()
                        break

                    # Firstly get info about message.
                    elif not message_info:
                        message_info = json.loads(data.decode())
                        sock.sendall(b"got message info")

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

            if not message_info:
                break

            # Processing text
            if message_info["type"] == "text":
                callback("text", message.decode())

            # Processing image
            elif message_info["type"] == "image":
                with open(_CONFIGURATION["image file path"], "wb") as image_file:
                    image_file.write(message)
                callback("image_path", os.path.abspath(_CONFIGURATION["image file path"]))

    thread = Thread(target=listener)

    thread.start()


setattr(create_listener, "listener_created", False)


def stop_listener() -> None:
    """Stop listener on socket."""

    setattr(create_listener, "tread_stopped", True)

    if getattr(create_listener, "listener_created", False):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            server_address = (_CONFIGURATION["addresses"]["operator"]["ip"],
                              _CONFIGURATION["addresses"]["operator"]["port"])
            sock.connect(server_address)

        setattr(create_listener, "listener_created", False)
