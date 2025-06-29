import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from webrtc_bridge.streamer import WebRTCStreamer
from webrtc_bridge.web.server import run_web


class WebRTCBridgeLocalNode(Node):
    def __init__(self):
        super().__init__("webrtc_bridge_local_node")

        # Declare parameters with standard default values
        self.declare_parameter("image_topic", "image_raw")
        self.declare_parameter("framerate", 30)
        self.declare_parameter("camera_info_topic", "camera_info")

        # Get parameter values
        self.image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.camera_info_topic = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )

        # Create subscriptions
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 1
        )

        self.streamer = WebRTCStreamer(self.get_logger())

    def image_callback(self, msg: Image) -> None:
        self.streamer.set_frame(msg)


def main():
    rclpy.init()
    node = WebRTCBridgeLocalNode()

    webcam_thread = threading.Thread(target=run_web, args=(node.streamer,), daemon=True)
    webcam_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
