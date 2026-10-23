"""Receive the latest complete camera frame through Bullet TCP/SHM transport."""

import cv2
import numpy as np
import zmq
from bullet_msgs.shm import FrameReader, MAX_MESSAGE_BYTES, topic_prefix


class ImageReceiver:
    """One exact topic per receiver. poll() returns metadata and owned BGR pixels."""

    def __init__(self, endpoint, topic):
        self.reader = FrameReader(topic)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        try:
            self.socket.setsockopt(zmq.LINGER, 0)
            self.socket.setsockopt(zmq.CONFLATE, 1)
            self.socket.setsockopt(zmq.MAXMSGSIZE, MAX_MESSAGE_BYTES)
            self.socket.setsockopt(zmq.SUBSCRIBE, topic_prefix(topic))
            self.socket.connect(endpoint)
        except BaseException:
            self.close()
            raise

    def poll(self):
        """Return (descriptor, BGR image), or None when no new complete frame exists."""
        try:
            message = self.socket.recv(zmq.NOBLOCK)
        except zmq.Again:
            return None
        if self.socket.getsockopt(zmq.RCVMORE):
            raise ValueError("expected one image descriptor message")
        result = self.reader.read(message)
        if result is None:
            return None
        metadata, pixels = result
        rows = np.frombuffer(pixels, dtype=np.uint8).reshape(metadata["height"], metadata["step"])
        image = rows[:, :metadata["width"] * 3].reshape(metadata["height"], metadata["width"], 3)
        if metadata["encoding"] == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        else:
            image = image.copy()
        return metadata, image

    def close(self):
        """Release SHM mappings, socket, and the receiver's private ZMQ context."""
        self.reader.close()
        self.socket.close(linger=0)
        self.context.term()
