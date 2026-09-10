import asyncio
import base64
import hashlib
import io
import json
import threading
from types import SimpleNamespace

import httpx
import pytest
import zmq
from PIL import Image

from PhyAgentOS.skill_runtime.g1d_camera import CameraUnavailableError, TeleImagerCameras


@pytest.fixture
def camera_server():
    stop, ready, publish = threading.Event(), threading.Event(), threading.Event()
    publish.set()
    ports = []
    raw = io.BytesIO()
    Image.new("RGB", (128, 48), "red").save(raw, "JPEG")
    jpeg = raw.getvalue()

    def serve():
        with zmq.Context() as context:
            rep, pub = context.socket(zmq.REP), context.socket(zmq.PUB)
            rep.linger = pub.linger = 0
            ports.extend([rep.bind_to_random_port("tcp://127.0.0.1"), pub.bind_to_random_port("tcp://127.0.0.1")])
            ready.set()
            while not stop.is_set():
                if rep.poll(10):
                    rep.recv()
                    rep.send_json({"head_camera": {"enable_zmq": True, "zmq_port": ports[1], "binocular": True}})
                if publish.is_set():
                    pub.send(jpeg)
            rep.close()
            pub.close()

    thread = threading.Thread(target=serve)
    thread.start()
    assert ready.wait(2)
    cameras = TeleImagerCameras({"host": "127.0.0.1", "request_port": ports[0],
        "stream_bindings": {"head": "head_camera", "right_wrist": "missing"}})
    cameras.start()
    try:
        yield cameras, publish, jpeg
    finally:
        cameras.close()
        stop.set()
        thread.join(2)
        assert not thread.is_alive()
        assert not cameras._thread.is_alive()


def test_unirobot_wire_snapshot_and_no_stale_replay(camera_server):
    cameras, publish, jpeg = camera_server
    first = cameras.observe("head")
    assert cameras.image(first["frame_id"]) == jpeg
    assert first["width"] == 128 and first["binocular"]
    assert first["image"]["sha256"] == hashlib.sha256(jpeg).hexdigest()
    assert first["capture_timestamp_available"] is False
    assert first["spatial_targeting_ready"] is False
    assert not cameras.state()["cameras"]["right_wrist"]["ready"]
    publish.clear()
    # A queued final transport frame is allowed once; repeated queries may not
    # pass off the saved image as a fresh exposure.
    try:
        cameras.observe("head")
    except CameraUnavailableError:
        pass
    with pytest.raises(CameraUnavailableError, match="fresh JPEG"):
        cameras.observe("head")
    assert not cameras.state()["cameras"]["head"]["ready"]
    publish.set()
    second = cameras.observe("head")
    assert second["frame_id"] != first["frame_id"]
    assert second["received_at_unix_s"] > first["received_at_unix_s"]
    # Bounded immutable cache and explicit expiry.
    raw, created = cameras._artifacts[first["frame_id"]]
    cameras._artifacts[first["frame_id"]] = (raw, created - 61)
    with pytest.raises(CameraUnavailableError, match="expired"):
        cameras.image(first["frame_id"])


@pytest.mark.asyncio
async def test_forge_observation_sends_pixels_without_base64_in_agent_history(camera_server):
    from PhyAgentOS.agent.tools.forge_tool_api import ForgeToolQueryTool
    from PhyAgentOS.forge.tool_client import ForgeToolClient

    cameras, _, jpeg = camera_server
    observed = await asyncio.to_thread(cameras.observe)
    async def handler(request):
        assert request.url.path == observed["image"]["path"]
        return httpx.Response(200, content=jpeg, headers={"content-type": "image/jpeg"})

    class Provider:
        async def chat_with_retry(self, *, messages):
            url = messages[-1]["content"][-1]["image_url"]["url"]
            assert base64.b64decode(url.split(",", 1)[1]) == jpeg
            return SimpleNamespace(content="A red scene.", finish_reason="stop")

    class Coordinator:
        async def invoke_query(self, task, tool, args, **kw):
            assert task == "bound-task"
            return {"ok": True, "data": {"response": {"result": {"status": "succeeded", "outputs": observed}}}}

    async with ForgeToolClient("http://gateway", transport=httpx.MockTransport(handler)) as client:
        tool = ForgeToolQueryTool(client, Coordinator(), Provider())
        result = await tool.execute("g1d.dual_arm.camera_observe", {}, task_id="bound-task")
    assert "base64" not in result
    assert json.loads(result)["data"]["response"]["result"]["outputs"]["vision"]["description"] == "A red scene."


@pytest.mark.asyncio
async def test_camera_artifact_cannot_redirect_or_load_an_untrusted_url():
    from PhyAgentOS.forge.tool_client import ForgeToolAPIError, ForgeToolClient

    async def handler(request):
        return httpx.Response(302, headers={"location": "http://other/secret"})
    async with ForgeToolClient("http://gateway", transport=httpx.MockTransport(handler)) as client:
        with pytest.raises(ForgeToolAPIError, match="path"):
            await client.read_camera_image({"path": "http://other/secret"})
        with pytest.raises(ForgeToolAPIError, match="unavailable"):
            await client.read_camera_image({"path": "/g1d/camera/frames/" + "a" * 32 + ".jpg"})


@pytest.mark.asyncio
async def test_provider_failure_is_not_a_visual_observation():
    from PhyAgentOS.agent.tools.forge_tool_api import ForgeToolQueryTool

    class Client:
        async def read_camera_image(self, image):
            return b"\xff\xd8image"

    class Provider:
        async def chat_with_retry(self, **kwargs):
            return SimpleNamespace(finish_reason="error", content="401 Invalid token")

    observed = {"image": {}, "camera": "head", "frame_id": "sample"}
    await ForgeToolQueryTool(Client(), None, Provider())._see(observed, None)
    assert observed["vision"]["status"] == "failed"
    assert "401" in observed["vision"]["error"]
    assert "description" not in observed["vision"]
