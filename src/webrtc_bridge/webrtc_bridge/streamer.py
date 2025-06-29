import asyncio

import av
import numpy as np
import rclpy.logging
from aiortc import (
    MediaStreamTrack,
    RTCPeerConnection,
    RTCRtpSender,
    RTCSessionDescription,
    VideoStreamTrack,
)
from aiortc.contrib.media import MediaRelay
from av import VideoFrame
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
    def __init__(self, logger, loop: asyncio.AbstractEventLoop | None = None) -> None:
        self.pcs: set[RTCPeerConnection] = set()
        self.relay = MediaRelay()
        self.video_stream = ProxiedVideoStreamTrack()
        self.logger = logger

        if loop is None:
            loop = asyncio.new_event_loop()

        self.loop = loop

    def set_frame(self, image_msg: Image) -> None:
        if image_msg.encoding == "yuv422_yuy2":
            np_array = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(
                image_msg.height, image_msg.width, -1
            )
            frame = av.VideoFrame.from_ndarray(np_array, format="yuyv422")

            self.video_stream.set_frame(frame)
        else:
            self.logger.log(
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

    async def offer(self, offer: RTCSessionDescription) -> RTCSessionDescription:
        pc = RTCPeerConnection()
        self.pcs.add(pc)

        self.logger.info(f"Starting new peer connection: {id(pc)}")
        self.logger.debug(offer.sdp)

        @pc.on("connectionstatechange")
        async def on_connectionstatechange() -> None:
            self.logger.info(f"Connection {id(pc)} state is {pc.connectionState}")
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

        return RTCSessionDescription(
            sdp=pc.localDescription.sdp, type=pc.localDescription.type
        )

    def offer_sync(self, offer: RTCSessionDescription) -> RTCSessionDescription:
        if self.loop is None:
            raise RuntimeError(
                "Event loop not set. Make sure the web server is running."
            )

        if not self.loop.is_running():
            raise RuntimeError("Event loop is not running.")

        future = asyncio.run_coroutine_threadsafe(self.offer(offer), self.loop)
        return future.result()

    async def shutdown(self) -> None:
        self.logger.info("Shutting down WebRTCStreamer")
        # Close all peer connections.
        coros = [pc.close() for pc in self.pcs]
        await asyncio.gather(*coros)
        self.pcs.clear()
        self.relay.close()
