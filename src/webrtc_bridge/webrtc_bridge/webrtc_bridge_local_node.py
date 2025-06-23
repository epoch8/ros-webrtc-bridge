import threading
import rclpy
from rclpy.node import Node
from webrtc_bridge.webcam import main as webcam_main


class WebRTCBridgeLocalNode(Node):
    def __init__(self):
        super().__init__("webrtc_bridge_local_node")
        self.get_logger().info("Hello World")


def main():
    rclpy.init()
    node = WebRTCBridgeLocalNode()

    webcam_thread = threading.Thread(target=webcam_main, daemon=True)
    webcam_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":

    main()
