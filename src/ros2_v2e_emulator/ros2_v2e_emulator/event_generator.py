#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from event_camera_msgs.msg import EventPacket
from cv_bridge import CvBridge
import numpy as np
import cv2

class EventGenerator(Node):
    def __init__(self):
        super().__init__('event_generator')
        self.bridge = CvBridge()
        self.prev_gray = None

        # Subscribe to the raw camera frames
        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        # Publisher for EventPacket
        self.pub = self.create_publisher(EventPacket, '/dvs/events', 10)

        # Canvas for visualization
        self.canvas = None
        cv2.namedWindow('DVS View', cv2.WINDOW_NORMAL)

        # Timer to refresh the OpenCV window at 30 Hz
        self.create_timer(1.0/30.0, self.timer_callback)

    def image_callback(self, msg: Image):
        # Convert ROS Image → OpenCV grayscale
        try:
            gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'CVBridge error: {e}')
            return

        # Initialize on first frame
        if self.prev_gray is None:
            self.prev_gray = gray
            h, w = gray.shape
            self.canvas = np.zeros((h, w), dtype=np.uint8)
            return

        # Compute absolute difference and threshold
        diff = cv2.absdiff(gray, self.prev_gray)
        _, mask = cv2.threshold(diff, 30, 255, cv2.THRESH_BINARY)

        # Build the EventPacket
        pkt = EventPacket()
        # Use ROS time for both header.stamp and time_base
        now = self.get_clock().now()
        pkt.header.stamp = now.to_msg()
        pkt.time_base = now.nanoseconds
        pkt.encoding = 'mono'  # 64-bit words: [pol y x dt]

        # Pack every changed pixel into a 64-bit word, little-endian
        buf = bytearray()
        ys, xs = np.nonzero(mask)
        for y_np, x_np in zip(ys, xs):
            y = int(y_np)
            x = int(x_np)
            # build 64-bit: polarity=1 (ON), y in bits 48-62, x in 32-47, dt=0
            word = (1 << 63) | (y << 48) | (x << 32)
            buf.extend(word.to_bytes(8, 'little'))
            # draw on canvas
            self.canvas[y, x] = 255

        # **The raw‐bytes field on EventPacket is called `events` in Python**
        # It’s a variable-length array of uint8, so we assign it from our byte buffer:
        pkt.events = list(buf)

        # Publish the packet
        self.pub.publish(pkt)

        # Update previous frame
        self.prev_gray = gray

    def timer_callback(self):
        # Show & clear the canvas
        if self.canvas is not None:
            cv2.imshow('DVS View', self.canvas)
            cv2.waitKey(1)
            self.canvas[:] = 0

def main(args=None):
    rclpy.init(args=args)
    node = EventGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
