import importlib.resources
import json

from aiohttp import web
from aiortc import (
    RTCSessionDescription,
)

from webrtc_bridge.streamer import WebRTCStreamer


async def index(request: web.Request) -> web.Response:
    with importlib.resources.path("webrtc_bridge.web", "index.html") as path:
        with open(path, "r") as f:
            content = f.read()
    return web.Response(content_type="text/html", text=content)


async def javascript(request: web.Request) -> web.Response:
    with importlib.resources.path("webrtc_bridge.web", "client.js") as path:
        with open(path, "r") as f:
            content = f.read()
    return web.Response(content_type="application/javascript", text=content)


def run_web(streamer: WebRTCStreamer) -> None:
    async def offer(request: web.Request) -> web.Response:
        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

        res = await streamer.offer(offer)

        return web.Response(
            content_type="application/json",
            text=json.dumps({"sdp": res.sdp, "type": res.type}),
        )

    async def on_shutdown(self, app: web.Application) -> None:
        # Close peer connections.
        await streamer.shutdown()

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
