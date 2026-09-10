"""TeleImager's UniRobot-compatible config REQ and raw JPEG SUB protocol.

Each socket belongs to the receiver thread. Images carry local receipt times;
the upstream JPEG protocol supplies neither exposure timestamps nor depth.
"""

from __future__ import annotations

import hashlib
import io
import threading
import time
import uuid
from collections import OrderedDict


class CameraUnavailableError(ValueError):
    pass


class TeleImagerCameras:
    def __init__(self, config, *, clock=time.monotonic):
        self.config, self.clock = config, clock
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._frames = {}
        self._streams = {}
        self._artifacts = OrderedDict()
        self.error = "connecting"
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._run, name="g1d-camera", daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        with self._condition:
            self._condition.notify_all()
        if self._thread:
            self._thread.join(timeout=3)

    def state(self):
        with self._condition:
            now = self.clock()
            return {
                "cameras": {name: {
                    "stream_key": key,
                    "ready": name in self._frames and now - self._frames[name][1] <= .5,
                    "receive_age_ms": ((now - self._frames[name][1]) * 1000
                                       if name in self._frames else None),
                    "binocular": bool(self._streams.get(key, {}).get("binocular", False)),
                } for name, key in self.config["stream_bindings"].items()},
                "error": self.error,
                "spatial_targeting_ready": False,
                "spatial_targeting_reason": "RGB only; no verified intrinsics/depth/robot extrinsics",
            }

    def observe(self, camera="head", **_):
        if camera not in self.config["stream_bindings"]:
            raise CameraUnavailableError("unknown camera")
        requested = self.clock()
        with self._condition:
            # Require a frame received AFTER this query; never relabel a cached image as live.
            while not self._stop.is_set():
                frame = self._frames.get(camera)
                if frame is not None and frame[1] > requested:
                    break
                remaining = 1.2 - (self.clock() - requested)
                if remaining <= 0:
                    raise CameraUnavailableError(f"{camera}: no fresh JPEG from TeleImager; {self.error or 'stream silent'}")
                self._condition.wait(min(remaining, .1))
            else:
                raise CameraUnavailableError("camera subscriber closed")
        raw, received, wall = frame
        from PIL import Image

        try:
            with Image.open(io.BytesIO(raw)) as decoded:
                if decoded.format != "JPEG" or decoded.width * decoded.height > 8_000_000:
                    raise ValueError("unsupported camera image")
                decoded.load()
                width, height = decoded.size
        except (OSError, ValueError) as exc:
            raise CameraUnavailableError(f"invalid JPEG: {exc}") from exc
        if self.clock() - received > .5:
            raise CameraUnavailableError("camera frame became stale during decoding")
        frame_id = uuid.uuid4().hex
        with self._condition:
            self._artifacts[frame_id] = (raw, self.clock())
            while len(self._artifacts) > 24:
                self._artifacts.popitem(last=False)
        return {
            "camera": camera, "frame_id": frame_id,
            "image": {"path": f"/g1d/camera/frames/{frame_id}.jpg",
                      "mime_type": "image/jpeg", "sha256": hashlib.sha256(raw).hexdigest()},
            "width": width, "height": height,
            "received_at_unix_s": wall, "receive_age_ms": (self.clock() - received) * 1000,
            "timestamp_basis": "local_receive_only", "capture_timestamp_available": False,
            "binocular": self.state()["cameras"][camera]["binocular"],
            "spatial_targeting_ready": False,
        }

    def image(self, frame_id):
        with self._condition:
            item = self._artifacts.get(frame_id)
            if item is None or self.clock() - item[1] > 60:
                raise CameraUnavailableError("image expired; request a new observation")
            return item[0]

    def _run(self):
        context = None
        sockets = {}
        try:
            import zmq

            context = zmq.Context()
            next_config = 0.0
            while not self._stop.is_set():
                if self.clock() >= next_config:
                    request = context.socket(zmq.REQ)
                    request.setsockopt(zmq.LINGER, 0)
                    try:
                        request.connect(f"tcp://{self.config['host']}:{self.config['request_port']}")
                        request.send(b"get_cam_config")
                        if not request.poll(300):
                            raise CameraUnavailableError("TeleImager config request timed out")
                        streams = request.recv_json()
                        if not isinstance(streams, dict):
                            raise CameraUnavailableError("invalid TeleImager configuration")
                        for name, key in self.config["stream_bindings"].items():
                            stream = streams.get(key, {})
                            port = stream.get("zmq_port") if stream.get("enable_zmq") else None
                            if port is not None and (not isinstance(port, int) or not 1 <= port <= 65535):
                                raise CameraUnavailableError("invalid TeleImager stream port")
                            if name in sockets and sockets[name][0] != port:
                                sockets.pop(name)[1].close()
                                with self._condition:
                                    self._frames.pop(name, None)
                            if port is not None and name not in sockets:
                                sub = context.socket(zmq.SUB)
                                sub.setsockopt(zmq.LINGER, 0)
                                sub.setsockopt(zmq.CONFLATE, 1)
                                sub.setsockopt(zmq.MAXMSGSIZE, 4_000_000)
                                sub.setsockopt(zmq.SUBSCRIBE, b"")
                                sub.connect(f"tcp://{self.config['host']}:{port}")
                                sockets[name] = (port, sub)
                        with self._condition:
                            self._streams, self.error = streams, None
                    except (ValueError, zmq.ZMQError) as exc:
                        self.error = str(exc)
                    finally:
                        request.close()
                    next_config = self.clock() + 3
                poller = zmq.Poller()
                for _, sub in sockets.values():
                    poller.register(sub, zmq.POLLIN)
                ready = dict(poller.poll(20)) if sockets else {}
                for name, (_, sub) in sockets.items():
                    if sub not in ready:
                        continue
                    raw = sub.recv()
                    if len(raw) > 4_000_000 or not raw.startswith(b"\xff\xd8"):
                        self.error = f"{name}: invalid JPEG message"
                        continue
                    with self._condition:
                        self._frames[name] = (raw, self.clock(), time.time())
                        self._condition.notify_all()
                if not sockets:
                    self._stop.wait(.02)
        except Exception as exc:
            self.error = f"camera subscriber unavailable: {exc}"
        finally:
            for _, sub in sockets.values():
                sub.close()
            if context is not None:
                context.term()
