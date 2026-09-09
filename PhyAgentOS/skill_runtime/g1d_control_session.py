"""Operator-started Unitree control session with independent state/hold watchdogs.

One session owns the lowcmd writer. Discovery detects visible competing writers;
the operator must also stop other controllers before starting this session.
"""

from __future__ import annotations

import fcntl
import math
import threading
import time
from dataclasses import replace
from pathlib import Path

from PhyAgentOS.skill_runtime.g1d_adapter import LowCmdFrame, MotorCommand, SafetyFaultError
from PhyAgentOS.skill_runtime.g1d_executor import StreamPhase, StreamSample


class LowCmdDiscovery:
    def __init__(self, domain_id):
        from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsPublication
        from cyclonedds.domain import DomainParticipant

        self.participant = DomainParticipant(domain_id)
        self.reader = BuiltinDataReader(self.participant, BuiltinTopicDcpsPublication)
        self.writers = set()

    def poll(self):
        for item in self.reader.take(1024):
            key = str(item.key)
            if not item.sample_info.valid_data:
                self.writers.discard(key)
            elif item.topic_name == "rt/lowcmd":
                self.writers.add(key)
        return set(self.writers)


def validate_control_profile(config):
    gains, modes = config["gains"], config["modes"]
    if len(gains) != 35 or len(modes) != 35:
        raise ValueError("35 gain/mode entries required")
    if any(len(pair) != 2 or any(not math.isfinite(x) or x < 0 for x in pair) for pair in gains):
        raise ValueError("finite nonnegative gains required")
    if any(mode not in (0, 1) for mode in modes):
        raise ValueError("motor modes must be 0 or 1")
    if any(modes[i] != 1 or min(gains[i]) <= 0 for i in range(12, 29)):
        raise ValueError("waist hold and both arms need position gains")
    if not 1 <= config["approved_mode"] <= 255:
        raise ValueError("mode_machine must be explicitly verified")
    if not 0 < config["max_arm_excursion_rad"] <= 1.5:
        raise ValueError("session excursion bound must be within (0, 1.5] rad")


class ControlSession:
    """Explicit handoff, persistent hold and bounded recovery around the executor."""

    def __init__(self, *, config, source, sink, motion, discovery,
                 lock_path: Path, duration_s=300.0, clock=time.monotonic):
        validate_control_profile(config)
        if not 10 <= duration_s <= 1800:
            raise ValueError("operator session duration must be 10..1800 seconds")
        self.config, self.source, self.sink = config, source, sink
        self.motion, self.discovery = motion, discovery
        self.lock_path, self.duration_s, self.clock = lock_path, duration_s, clock
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._writer_thread = None
        self._monitor_thread = None
        self._file = None
        self._released = False
        self._ready = False
        self.error = None
        self.recovery = "not_needed"
        self._latest = None
        self._received = 0.0
        self._last_tick = None
        self._monitor_at = 0.0
        self._last_frame = None
        self._last_update = 0.0
        self._moving = False
        self._initial = None
        self._expires = 0.0
        self.sent = 0
        self.max_gap_s = 0.0
        self._last_sent = None

    def _read(self):
        item = self.source.read()
        if item is not None:
            frame, received = item
            if self._last_tick is None or 0 < (frame.tick - self._last_tick) % (2**32) < 2**31:
                self._latest, self._received, self._last_tick = frame, received, frame.tick
        if self._latest is None or self.clock() - self._received > .1:
            raise SafetyFaultError("session state unavailable or stale")
        if not self._latest.safety_ok or self._latest.mode_machine != self.config["approved_mode"]:
            raise SafetyFaultError("session state fault or machine mode changed")
        if self._initial is not None:
            if any(abs(self._latest.positions[i] - self._initial[i]) > .05 for i in range(12, 15)):
                raise SafetyFaultError("waist moved outside session hold tolerance")
            if any(abs(self._latest.positions[i] - self._initial[i]) > self.config["max_arm_excursion_rad"] + .05
                   for i in range(15, 29)):
                raise SafetyFaultError("measured arm excursion exceeded")
        return self._latest

    def start(self):
        self.lock_path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.lock_path.open("a")
        try:
            fcntl.flock(self._file, fcntl.LOCK_EX | fcntl.LOCK_NB)
            deadline = self.clock() + 5
            while True:
                self.discovery.poll()
                try:
                    self._read()
                    break
                except SafetyFaultError:
                    if self.clock() >= deadline:
                        raise
                    time.sleep(.01)
            # Discovery is allowed to settle before changing the original mode.
            until = self.clock() + 1
            while self.clock() < until:
                self.discovery.poll()
                self._read()
                time.sleep(.01)
            if self.motion.check() != "ai":
                raise SafetyFaultError("expected factory ai mode; another controller may own lowcmd")
            if len(self.discovery.poll() - {str(self.sink.writer_guid)}) > 1:
                raise SafetyFaultError("multiple lowcmd writers before handoff")
            # A timeout may mean release happened. Recovery must reconcile it.
            self._released = True
            self.motion.release()
            deadline = self.clock() + 5
            while True:
                state = self._read()
                other = self.discovery.poll() - {str(self.sink.writer_guid)}
                if self.motion.check() == "" and not other and self.sink.matched:
                    break
                if self.clock() >= deadline:
                    raise SafetyFaultError(f"handoff incomplete: other writers={sorted(other)}")
                time.sleep(.05)
            self._initial = tuple(state.positions)
            self._last_frame = LowCmdFrame(mode_pr=0, mode_machine=state.mode_machine,
                motor_cmd=tuple(MotorCommand(mode=mode, q=q, kp=gain[0], kd=gain[1])
                    for q, mode, gain in zip(state.positions, self.config["modes"], self.config["gains"])))
            self._expires = self.clock() + self.duration_s
            self._monitor_at = self.clock()
            self._ready = True
            self._writer_thread = threading.Thread(target=self._run_writer, name="g1d-session-hold", daemon=True)
            self._monitor_thread = threading.Thread(target=self._run_monitor, name="g1d-session-mode", daemon=True)
            self._writer_thread.start()
            self._monitor_thread.start()
        except BaseException:
            self.close()
            raise

    def require(self):
        if not self._ready or self._stop.is_set() or self.error:
            raise SafetyFaultError(self.error or "control session inactive")
        if self.clock() >= self._expires or self.clock() - self._monitor_at > .5:
            raise SafetyFaultError("operator session or mode monitor expired")

    def validate_plan(self, plan):
        self.require()
        if self._initial is None:
            raise SafetyFaultError("no initial hold")
        path = getattr(plan, "joint_path", None)
        points = (path.points if path is not None else
                  [(*plan.joint_solution.left_q, *plan.joint_solution.right_q)])
        for q in points:
            if any(abs(value - self._initial[15+i]) > self.config["max_arm_excursion_rad"]
                   for i, value in enumerate(q)):
                raise SafetyFaultError("plan exceeds operator session excursion bound")

    def write(self, sample):
        if not isinstance(sample, StreamSample):
            raise SafetyFaultError("arm session does not accept gripper commands")
        with self._lock:
            self.require()
            self._read()
            frame = sample.frame
            if len(frame.motor_cmd) != 35 or frame.mode_machine != self.config["approved_mode"]:
                raise SafetyFaultError("invalid command shape or machine mode")
            if not self.sink.matched:
                raise SafetyFaultError("DDS command receiver lost")
            commands = list(frame.motor_cmd)
            for i in range(35):
                if i not in range(15, 29):
                    commands[i] = self._last_frame.motor_cmd[i]
                else:
                    command = commands[i]
                    if any(not math.isfinite(value) for value in
                           (command.q, command.dq, command.tau, command.kp, command.kd)):
                        raise SafetyFaultError("nonfinite arm command")
                    if abs(command.q - self._initial[i]) > self.config["max_arm_excursion_rad"]:
                        raise SafetyFaultError("command exceeds operator excursion bound")
                    commands[i] = replace(command, mode=self.config["modes"][i],
                        kp=self.config["gains"][i][0], kd=self.config["gains"][i][1])
            frame = replace(frame, mode_pr=0, motor_cmd=tuple(commands)).with_crc()
            self.sink.write(replace(sample, frame=frame))
            self._record_send()
            self._last_frame = frame
            self._last_update = self.clock()
            self._moving = any(abs(m.dq) > 1e-6 for m in frame.motor_cmd[15:29])

    def _record_send(self):
        now = self.clock()
        if self._last_sent is not None:
            self.max_gap_s = max(self.max_gap_s, now - self._last_sent)
        self._last_sent = now
        self.sent += 1

    def _run_writer(self):
        try:
            while not self._stop.wait(.002):
                with self._lock:
                    self.require()
                    self._read()
                    if self._moving:
                        if self.clock() - self._last_update > .02:
                            raise SafetyFaultError("trajectory writer stalled")
                        continue
                    if self._last_sent is not None and self.clock() - self._last_sent < .002:
                        continue
                    self.sink.write(StreamSample(self.sent, self.clock(), self._last_frame.with_crc(), StreamPhase.HOLD))
                    self._record_send()
        except BaseException as error:
            self.error = str(error)
            self._ready = False
            self._stop.set()

    def _run_monitor(self):
        try:
            while not self._stop.wait(.1):
                other = self.discovery.poll() - {str(self.sink.writer_guid)}
                if other or not self.sink.matched:
                    raise SafetyFaultError(f"DDS ownership/receiver lost: {sorted(other)}")
                if self.motion.check() != "":
                    raise SafetyFaultError("factory control mode changed")
                self._monitor_at = self.clock()
        except BaseException as error:
            self.error = str(error)
            self._ready = False
            self._stop.set()
        finally:
            self._recover()

    def _recover(self):
        self._ready = False
        self._stop.set()
        # Wait for an in-flight executor write before restoring another controller.
        with self._lock:
            pass
        if self._writer_thread is not None:
            self._writer_thread.join(timeout=3)
            if self._writer_thread.is_alive():
                self.recovery = "failed_writer_still_running"
                return
        if not self._released:
            return
        try:
            mode = self.motion.check()
            if mode == "":
                if self.discovery.poll() - {str(self.sink.writer_guid)}:
                    raise SafetyFaultError("another writer visible; refusing to override it")
                try:
                    self.motion.restore()
                except (TimeoutError, RuntimeError):
                    # SelectMode may finish after its RPC deadline. Reconcile
                    # the observed mode instead of issuing a duplicate command.
                    if self.motion.check() != "ai":
                        raise
            elif mode != "ai":
                raise SafetyFaultError(f"unexpected mode {mode!r}; refusing to override it")
            deadline = self.clock() + 5
            while self.motion.check() != "ai":
                if self.clock() >= deadline:
                    raise TimeoutError("factory ai restore unconfirmed")
                time.sleep(.05)
            self.recovery = "ai_confirmed"
            self._released = False
        except BaseException as error:
            self.recovery = f"unconfirmed: {error}"

    def close(self):
        self._ready = False
        self._stop.set()
        if self._monitor_thread is not None:
            self._monitor_thread.join(timeout=10)
        else:
            self._recover()
        if self._monitor_thread is not None and self._monitor_thread.is_alive():
            raise RuntimeError("control recovery still running")
        if self._file is not None:
            self._file.close()
            self._file = None

    def status(self):
        return {"ready": self._ready and not self._stop.is_set(), "error": self.error,
                "recovery": self.recovery, "frames_sent": self.sent,
                "max_writer_gap_ms": self.max_gap_s * 1000,
                "remaining_s": max(0.0, self._expires - self.clock())}
