#!/usr/bin/env python3
import argparse

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class Capture(Node):
    def __init__(self, topic: str, output: str, compressed: bool):
        super().__init__('one_shot_image_capture')
        self.output = output
        message_type = CompressedImage if compressed else Image
        self.subscription = self.create_subscription(
            message_type,
            topic,
            self.callback,
            qos_profile_sensor_data,
        )

    def callback(self, message):
        if isinstance(message, CompressedImage):
            image = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
        else:
            row_bytes = int(message.width) * 3
            step = int(message.step) or row_bytes
            rows = np.frombuffer(message.data, np.uint8).reshape(int(message.height), step)
            image = rows[:, :row_bytes].reshape(int(message.height), int(message.width), 3)
        if image is None or not cv2.imwrite(self.output, image):
            raise RuntimeError(f'Unable to save {self.output}')
        self.get_logger().info(f'Saved {self.output}')
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('topic')
    parser.add_argument('output')
    parser.add_argument('--compressed', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = Capture(args.topic, args.output, args.compressed)
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


if __name__ == '__main__':
    main()
