import threading

import rclpy
from aiortc import RTCSessionDescription
from rclpy.node import Node
from sensor_msgs.msg import Image

from webrtc_bridge.streamer import WebRTCStreamer
from webrtc_bridge.web.server import run_web
from webrtc_bridge_srv.srv import WebRTCOffer, WebRTCOffer_Request, WebRTCOffer_Response


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

        # Create service server
        self.offer_service = self.create_service(
            WebRTCOffer, f"{self.get_name()}/offer", self.handle_offer_request
        )

        self.streamer = WebRTCStreamer(self.get_logger())

    def image_callback(self, msg: Image) -> None:
        self.streamer.set_frame(msg)

    def handle_offer_request(
        self, request: WebRTCOffer_Request, response: WebRTCOffer_Response
    ) -> WebRTCOffer_Response:
        """Handle WebRTC offer request and return answer"""
        self.get_logger().info(
            f"Received WebRTC offer: type={request.type}, sdp length={len(request.sdp)}"
        )

        # Create RTCSessionDescription from the request
        offer = RTCSessionDescription(sdp=request.sdp, type=request.type)

        # Call the async offer method from the main thread
        answer = self.streamer.offer_sync(offer)

        # Set the response
        response.type = answer.type
        response.sdp = answer.sdp

        return response


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
