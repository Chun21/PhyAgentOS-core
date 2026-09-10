# G1_D cameras and Dex1

On the robot, activate `phyagent`, enter `~/PhyAgentOS-core` and run
`paos agent --physical`. The managed runtime subscribes to the existing TeleImager
and Dex1 serial service. No extra `g1d_control.sh` process is needed.

Examples in the TUI:

- “看一下头部相机，告诉我桌上有什么。”
- “看一下左手腕相机，夹爪前面有没有障碍物？”
- “把左夹爪张开到 80%，双臂和右夹爪保持不动。”
- “把右夹爪闭合到 30%，然后看右腕相机确认位置。”

PAOS registers camera queries and gripper planning in the same Skill and Forge
Gateway as the arms. `plan_gripper({left: 0.8})` holds the measured arm joints,
returns a five-second plan, and executes via the existing task-bound
`execute_pose` Action. Feedback, invocation status and evidence stay in the normal
PAOS execution chain. Opening 0 is closed; 1 is open (Dex1 q=5.4 rad, kp=5, kd=.05,
matching UniRobot's external Dex1 command profile). The controller ramps the
command, rather than immediately sending the endpoint.

The independent session writer refreshes requested gripper targets at 50 Hz
between Actions. Stop/failure freezes the last emitted opening. Feedback loss or
a visible competing command writer suspends that side until another explicitly
admitted operation. Session exit stops publication; the external service's 1 s
timeout switches to BRAKE. Holding a payload across TUI shutdown is not guaranteed.
The service's DDS `lost` field is not independent proof of fresh serial feedback:
the current C++ service does not expose every serial transport failure.

The camera subscriber follows UniRobot's TeleImager wire protocol: a ZMQ REQ on
60000 discovers enabled streams; a SUB per stream receives a single raw JPEG.
The robot's keys are `head_camera`, `left_wrist_camera`, `right_wrist_camera` on
55555/55556/55557. Configuration lives in the bundle's
`profiles/real-g1d/camera.json`. PAOS subscribes without starting a second camera
capture process. The pinned Python dependencies are pyzmq 26.4.0 and Pillow 11.3.0;
DDS stays at 0.10.2.

`camera_observe` requires a newly received JPEG, validates it, and returns an
immutable image reference plus receipt timestamp. The Agent wrapper downloads
that image from its configured Gateway, verifies the hash, and sends real pixels
to PAOS's multimodal provider (or the current provider when mode routing is
disabled). The conversation stores the visual description and frame ID, not
base64 text. A model without image support returns a vision error. Camera streams
recover independently and an unavailable wrist camera does not disable the arms.

Visual observations are not metric grasp targets. No verified camera calibration
was found in the deployed TeleImager service. The stereo RGB images contain no
depth or exposure timestamps. Autonomous object-to-grasp planning still needs
camera intrinsics/distortion, stereo depth or another metric depth source, and
camera-to-robot transforms (wrist cameras need transforms through live FK).
Those transforms must account for the physical waist/base rather than treating
the fixed UniRobot IK model frame as a world frame. Contact-aware closure and
object retention also need measured hardware validation. Until then, PAOS can
observe, execute explicit arm goals and operate grippers, but must not invent
three-dimensional object positions or report a secure grasp from closure alone.

## Deployment validation, 2026-09-10

Skill and node 0.3.3 were installed and SHA-256 verified on `192.168.123.164`.
The managed read-only Gateway exposed all seven tools. Fresh head (1280×480)
and left/right wrist (640×480) JPEGs were retrieved through the Forge queries
and their image hashes verified. Both Dex1 sides supplied fresh state and matched
command receivers, with no competing command writer observed. No gripper or arm
motion was commanded during these hardware checks.

The regression suite covered 131 tests for planning, execution, session lifecycle,
DDS, cameras, grippers, packaging, and installed Gateway/AgentTask arm execution.
Four further tests covered current Forge context and task serialization. Physical
gripper opening/contact/retention remain unverified on hardware.

The real `paos agent` image request was blocked by the configured model provider's
`401 Invalid token`. The robot and development host have matching model, provider
endpoint and key; this is not a source synchronization mismatch. Replace the
invalid credential in `~/.PhyAgentOS/config.json` before expecting natural-language
requests or image interpretation to work. Pixel delivery and failure reporting are
tested with a fake provider; actual image interpretation has not passed a live
model test. No camera-to-robot spatial calibration or autonomous grasp acceptance
is claimed.
