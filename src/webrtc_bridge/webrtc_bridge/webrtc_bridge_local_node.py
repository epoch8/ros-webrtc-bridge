import rclpy
from rclpy.node import Node


class WebRTCBridgeLocalNode(Node):
    def __init__(self):
        super().__init__("webrtc_bridge_local_node")
        self.get_logger().info("Hello World")


def main():
    rclpy.init()
    node = WebRTCBridgeLocalNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
