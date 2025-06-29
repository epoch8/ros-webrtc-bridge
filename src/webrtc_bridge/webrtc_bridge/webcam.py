import asyncio
import importlib.resources
import json

from aiohttp import web
from aiortc import (
    MediaStreamTrack,
    RTCPeerConnection,
    RTCRtpSender,
    RTCSessionDescription,
    VideoStreamTrack,
)
from aiortc.contrib.media import MediaRelay
from av import VideoFrame
import av
import numpy as np
import rclpy.logging
from sensor_msgs.msg import Image


class ProxiedVideoStreamTrack(VideoStreamTrack):
    def __init__(self) -> None:
        super().__init__()
        self.current_frame: VideoFrame | None = None

    def set_frame(self, frame: VideoFrame) -> None:
        self.current_frame = frame

    async def recv(self) -> VideoFrame:
        pts, time_base = await self.next_timestamp()

        if self.current_frame is not None:
            frame = self.current_frame
        else:
            frame = VideoFrame(width=640, height=480)
            for p in frame.planes:
                p.update(bytes(p.buffer_size))

        frame.pts = pts
        frame.time_base = time_base
        return frame


class WebRTCStreamer:
    def __init__(self) -> None:
        self.pcs: set[RTCPeerConnection] = set()
        self.relay = MediaRelay()
        self.video_stream = ProxiedVideoStreamTrack()

    def set_frame(self, image_msg: Image) -> None:
        if image_msg.encoding == "yuv422_yuy2":
            np_array = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(
                image_msg.height, image_msg.width, -1
            )
            frame = av.VideoFrame.from_ndarray(np_array, format="yuyv422")

            self.video_stream.set_frame(frame)
        else:
            rclpy.logging.get_logger("WebRTCStreamer").log(
                f"Unsupported image encoding: {image_msg.encoding}. Expected 'yuv422_yuy2'.",
                rclpy.logging.LoggingSeverity.WARN,
            )

    def create_local_tracks(self) -> MediaStreamTrack:
        return self.relay.subscribe(self.video_stream)

    def force_codec(
        self, pc: RTCPeerConnection, sender: RTCRtpSender, forced_codec: str
    ) -> None:
        kind = forced_codec.split("/")[0]
        codecs = RTCRtpSender.getCapabilities(kind).codecs
        transceiver = next(t for t in pc.getTransceivers() if t.sender == sender)
        transceiver.setCodecPreferences(
            [codec for codec in codecs if codec.mimeType == forced_codec]
        )

    def force_h264(self, pc: RTCPeerConnection, sender: RTCRtpSender) -> None:
        """Force H.264 codec on the sender."""
        self.force_codec(pc, sender, "video/H264")

    async def offer(self, request: web.Request) -> web.Response:
        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

        pc = RTCPeerConnection()
        self.pcs.add(pc)

        @pc.on("connectionstatechange")
        async def on_connectionstatechange() -> None:
            print("Connection state is %s" % pc.connectionState)
            print("")
            if pc.connectionState == "failed":
                await pc.close()
                self.pcs.discard(pc)

        # open media source
        video = self.create_local_tracks()

        video_sender = pc.addTrack(video)
        assert video_sender is not None
        # force_h264(pc, video_sender)

        await pc.setRemoteDescription(offer)

        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        return web.Response(
            content_type="application/json",
            text=json.dumps(
                {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
            ),
        )

    async def on_shutdown(self, app: web.Application) -> None:
        # Close peer connections.
        coros = [pc.close() for pc in self.pcs]
        await asyncio.gather(*coros)
        self.pcs.clear()


streamer = WebRTCStreamer()


async def index(request: web.Request) -> web.Response:
    with importlib.resources.path("webrtc_bridge", "index.html") as path:
        with open(path, "r") as f:
            content = f.read()
    return web.Response(content_type="text/html", text=content)


async def javascript(request: web.Request) -> web.Response:
    with importlib.resources.path("webrtc_bridge", "client.js") as path:
        with open(path, "r") as f:
            content = f.read()
    return web.Response(content_type="application/javascript", text=content)


def main():
    # logging.basicConfig(level=logging.DEBUG)

    app = web.Application()
    app.on_shutdown.append(streamer.on_shutdown)
    app.router.add_get("/", index)
    app.router.add_get("/client.js", javascript)
    app.router.add_post("/offer", streamer.offer)
    web.run_app(
        app,
        host="0.0.0.0",
        port=8080,
        handle_signals=False,
    )


if __name__ == "__main__":
    main()
