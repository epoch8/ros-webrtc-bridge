import argparse
import asyncio
import json
import logging
import os
import platform
import ssl
from typing import Optional

from aiohttp import web
from aiortc import (
    MediaStreamTrack,
    RTCPeerConnection,
    RTCRtpSender,
    RTCSessionDescription,
)
from aiortc.contrib.media import MediaPlayer, MediaRelay
import importlib.resources

ROOT = os.path.dirname(__file__)

pcs = set()
relay = None
webcam = None


def create_local_tracks() -> tuple[
    Optional[MediaStreamTrack], Optional[MediaStreamTrack]
]:
    global relay, webcam

    # Otherwise, play from the system's default webcam.
    #
    # In order to serve the same webcam to multiple users we make use of
    # a `MediaRelay`. The webcam will stay open, so it is our responsability
    # to stop the webcam when the application shuts down in `on_shutdown`.
    options = {"framerate": "15", "video_size": "640x480"}
    if relay is None:
        if platform.system() == "Darwin":
            webcam = MediaPlayer("default:none", format="avfoundation", options=options)
        elif platform.system() == "Windows":
            webcam = MediaPlayer(
                "video=Integrated Camera", format="dshow", options=options
            )
        else:
            webcam = MediaPlayer("/dev/video0", format="v4l2", options=options)
        relay = MediaRelay()
    return None, relay.subscribe(webcam.video)


def force_codec(pc: RTCPeerConnection, sender: RTCRtpSender, forced_codec: str) -> None:
    kind = forced_codec.split("/")[0]
    codecs = RTCRtpSender.getCapabilities(kind).codecs
    transceiver = next(t for t in pc.getTransceivers() if t.sender == sender)
    transceiver.setCodecPreferences(
        [codec for codec in codecs if codec.mimeType == forced_codec]
    )


def force_h264(pc: RTCPeerConnection, sender: RTCRtpSender) -> None:
    """Force H.264 codec on the sender."""
    force_codec(pc, sender, "video/H264")


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


async def offer(request: web.Request) -> web.Response:
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    pc = RTCPeerConnection()
    pcs.add(pc)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange() -> None:
        print("Connection state is %s" % pc.connectionState)
        print("")
        if pc.connectionState == "failed":
            await pc.close()
            pcs.discard(pc)

    # open media source
    audio, video = create_local_tracks()

    video_sender = pc.addTrack(video)
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


async def on_shutdown(app: web.Application) -> None:
    # Close peer connections.
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()

    # If a shared webcam was opened, stop it.
    if webcam is not None:
        webcam.video.stop()


def main():
    # logging.basicConfig(level=logging.DEBUG)

    app = web.Application()
    app.on_shutdown.append(on_shutdown)
    app.router.add_get("/", index)
    app.router.add_get("/client.js", javascript)
    app.router.add_post("/offer", offer)
    web.run_app(
        app,
        host="0.0.0.0",
        port=8080,
        handle_signals=False,
    )


if __name__ == "__main__":
    main()
