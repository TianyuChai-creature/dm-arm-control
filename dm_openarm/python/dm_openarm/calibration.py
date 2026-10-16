"""Dual-arm calibration using firmware MIT position control, not host torque PD."""
from __future__ import annotations

from collections import deque
from copy import deepcopy
from dataclasses import dataclass, field
import math
from statistics import median
import threading
import time

from . import _core

# Station defaults; contact thresholds require hardware commissioning.
_RATE, _TOUCH_RATE, _LEAD = 2.0, 0.2, 0.05
_ACCELERATION, _APPROACH, _FEEDBACK_LAG = 6.0, 0.2, 0.15
_JOINT_TIME_TARGET, _TOTAL_TIME_TARGET = 12.0, 120.0
_WINDOW, _STILL, _TOL = 0.25, 0.002, math.pi / 180
_RANGES = (
    (-3.4907, 1.3963), (-3.3161, 0.17453), (-1.5708, 1.5708),
    (0.0, 2.4435), (-1.5708, 1.5708),
    (-1.3963, 3.4907), (-0.17453, 3.3161), (-1.5708, 1.5708),
    (0.0, 2.4435), (-1.5708, 1.5708),
)


def _zero_position(measured_lower, measured_upper, ideal_lower, ideal_upper):
    values = (measured_lower,measured_upper,ideal_lower,ideal_upper)
    if (not all(math.isfinite(v) for v in values) or measured_upper<=measured_lower or
            ideal_upper<=ideal_lower or not ideal_lower<=0<=ideal_upper):
        raise ValueError("invalid endpoint pair for zero interpolation")
    # Interpolate the zero once; subsequent SDK angles remain encoder radians.
    fraction = -ideal_lower/(ideal_upper-ideal_lower)
    return measured_lower+fraction*(measured_upper-measured_lower)


@dataclass
class JointZeroResult:
    can_id: int
    measured_lower: float | None = None
    measured_upper: float | None = None
    repeat_error: float = 0.0
    span_error: float | None = None
    zero_raw: float | None = None
    final_error: float | None = None
    kp: float = 160.0
    kd: float = 5.0
    elapsed_s: float = 0.0
    phase: str = "WAITING"
    save_state: str = "NOT_ATTEMPTED"
    time_target_exceeded: bool = False


@dataclass
class CalibrationReport:
    success: bool = False
    phase: str = "IDLE"
    error: str = ""
    elapsed_s: float = 0.0
    joints: list[JointZeroResult] = field(default_factory=list)
    time_target_exceeded: bool = False


class CalibrationError(RuntimeError):
    def __init__(self, message, report):
        super().__init__(message)
        self.report = deepcopy(report)


def _endpoint_hints(previous, states):
    # Hints change approach timing only; both endpoints are measured again.
    if any(r.save_state=="UNKNOWN" for r in previous.joints):
        return {}
    hints = {}
    for r in previous.joints:
        shift = r.zero_raw if r.save_state=="SAVED" else 0
        pair = {}
        for name in ("lower","upper"):
            measured = getattr(r,"measured_"+name)
            if measured is not None:
                pair[name] = measured-shift
        if pair:
            hints[r.can_id] = pair
    return hints


class _Contact:
    def __init__(self):
        self.samples = deque()
        self.sequence = None

    def update(self, state, q, direction, now, threshold):
        # Count observed reception batches, never repeated cached snapshots.
        if state.rx_sequence == self.sequence:
            return None
        self.sequence = state.rx_sequence
        received = now - state.last_rx_age_s
        if self.samples and received - self.samples[-1][0] < 0.02:
            return None
        self.samples.append((received, state.position, direction * state.torque, direction * q))
        while len(self.samples) > 1 and self.samples[1][0] <= received - 1.2:
            self.samples.popleft()
        history = list(self.samples)
        # Keep the preceding load rise: delayed feedback can first show a stop
        # only after bounded MIT effort has already reached a steady plateau.
        start = received-_WINDOW
        first = next((i for i, sample in enumerate(history) if sample[0] >= start),len(history)-1)
        if first > 0:
            first -= 1
        samples = history[first:]
        if len(samples) < 3 or samples[-1][0]-samples[0][0] < _WINDOW:
            return None
        positions = [s[1] for s in samples]
        if max(positions)-min(positions) > _STILL:
            return None
        prior = history[:first+1]
        if len(prior) < 2 or samples[-1][3]-min(s[3] for s in prior) < 0.003:
            return None
        baseline = min(median([s[2] for s in prior[i:i+3]]) for i in range(len(prior)-1))
        tail = [s for s in samples if s[0] >= received-0.15]
        if (len(tail) < 2 or tail[-1][0]-tail[0][0] < 0.1-1e-9 or
                any(s[2] < baseline+threshold for s in tail)):
            return None
        return median(positions)



class _JointScan:
    def __init__(self, result, position, now, hints=None):
        self.result = result
        self.lower, self.upper = _RANGES[result.can_id]
        hints = hints or {}
        self.upper_hint_measured = hints.get("upper") is not None
        self.hint_lower = hints.get("lower")
        self.hint_upper = hints.get("upper")
        if self.hint_lower is None and self.hint_upper is not None:
            self.hint_lower = self.hint_upper-(self.upper-self.lower)
        if self.hint_upper is None and self.hint_lower is not None:
            self.hint_upper = self.hint_lower+(self.upper-self.lower)
        if self.hint_lower is None:
            self.hint_lower,self.hint_upper = self.lower,self.upper
        self.q = position
        self.started = now
        self.first = None
        self.initial_kp = result.kp
        self.contact_lead = _LEAD if result.can_id % 5 < 2 else _LEAD/2
        self._enter("LOWER", position, now)

    def _enter(self, phase, position, now, target=None):
        self.result.phase = phase
        self.began = now
        self.last_raise = now
        self.origin = position
        self.target = target
        self.contact = _Contact()
        self.budget_at = None
        self.settled_at = None
        self.velocity = 0.0
        self.feedback_velocity = 0.0

    def _lead(self, speed):
        if abs(speed) <= _TOUCH_RATE + 1e-9:
            return self.contact_lead + (self.result.kd/self.result.kp + _FEEDBACK_LAG)*abs(self.feedback_velocity)
        # Allow damping error plus the measured feedback transport lag in transit.
        return self.contact_lead + (self.result.kd/self.result.kp + _FEEDBACK_LAG)*abs(speed)

    def _velocity(self, wanted, dt):
        change = max(-_ACCELERATION*dt, min(_ACCELERATION*dt, wanted-self.velocity))
        self.velocity += change
        return self.velocity

    def _ramp(self, position, target, dt):
        remaining = target-self.q
        speed = min(_RATE, math.sqrt(2*_ACCELERATION*abs(remaining)))
        velocity = self._velocity(math.copysign(speed, remaining), dt)
        step = min(abs(velocity)*dt, abs(remaining))
        candidate = self.q + math.copysign(step, remaining)
        lead = self._lead(velocity)
        self.q = max(position-lead, min(position+lead, candidate))

    def step(self, state, now, dt):
        r = self.result
        if r.phase == "DONE":
            return self.q
        r.elapsed_s = now-self.started
        r.time_target_exceeded = r.elapsed_s > _JOINT_TIME_TARGET
        self.feedback_velocity = state.velocity
        if self.target is not None:
            self._ramp(state.position, self.target, dt)
            if r.phase == "MOVE_ZERO":
                near = abs(state.position-self.target) < _TOL
                if not near and now-self.last_raise >= 2:
                    r.kp = min(r.kp+self.initial_kp*0.25,self.initial_kp*2,500)
                    self.last_raise = now
            else:
                direction = 1 if r.phase == "BACKOFF_LOWER" else -1
                clearance = direction*(state.position-self.first)
                near = clearance >= 0.015
                if not near and abs(self.q-self.target)<1e-8:
                    distance = min(0.06,0.03+abs(self.q-state.position))
                    adjusted = self.first+direction*distance
                    if direction*(adjusted-self.target)>0:
                        self.target = adjusted
            if near and abs(state.velocity) <= 0.1 and abs(self.q-self.target) < 1e-8:
                self.settled_at = now if self.settled_at is None else self.settled_at
                if now-self.settled_at >= 0.12:
                    if r.phase == "MOVE_ZERO":
                        r.phase = "DONE"
                    else:
                        phase = "REPEAT_LOWER" if r.phase == "BACKOFF_LOWER" else "REPEAT_UPPER"
                        self._enter(phase, state.position, now)
            else:
                self.settled_at = None
            return self.q
        direction = -1 if "LOWER" in r.phase else 1
        endpoint_hint = self.hint_lower if direction < 0 else self.hint_upper
        distance = direction*(endpoint_hint-state.position)
        if r.phase.startswith("REPEAT"):
            wanted = _TOUCH_RATE
        else:
            margin = _APPROACH + _FEEDBACK_LAG*max(abs(self.velocity),abs(state.velocity))
            wanted = max(_TOUCH_RATE, min(_RATE, math.sqrt(2*_ACCELERATION*max(0.0,distance-margin))))
        velocity = self._velocity(direction*wanted,dt)
        lead_limit = self._lead(velocity)
        lead = direction*(self.q-state.position)
        if abs(self.q-state.position) > self._lead(_RATE)+0.1:
            raise RuntimeError(f"CAN {r.can_id}: feedback jump during search")
        if lead > lead_limit:
            self.q -= direction*min(_TOUCH_RATE*dt,lead-lead_limit)
        else:
            self.q += direction*min(abs(velocity)*dt,lead_limit-lead)
        if direction*(self.q-state.position) >= lead_limit-1e-8:
            self.budget_at = now if self.budget_at is None else self.budget_at
        else:
            self.budget_at = None
        threshold = 0.15 if r.can_id % 5 < 2 else 0.10
        endpoint = self.contact.update(state, self.q, direction, now, threshold)
        if endpoint is not None:
            if not r.phase.startswith("REPEAT"):
                self.first = endpoint
                phase = "BACKOFF_LOWER" if direction < 0 else "BACKOFF_UPPER"
                self._enter(phase, state.position, now, endpoint-direction*0.03)
            else:
                error = abs(endpoint-self.first)
                r.repeat_error = max(r.repeat_error, error)
                if direction < 0:
                    r.measured_lower = endpoint
                    self.hint_lower = endpoint
                    if not getattr(self,"upper_hint_measured",False):
                        self.hint_upper = endpoint+(self.upper-self.lower)
                    self._enter("UPPER", state.position, now)
                else:
                    r.measured_upper = endpoint
                    r.span_error = (r.measured_upper-r.measured_lower)-(self.upper-self.lower)
                    r.zero_raw = _zero_position(r.measured_lower,r.measured_upper,self.lower,self.upper)
                    if not r.measured_lower <= r.zero_raw <= r.measured_upper:
                        raise RuntimeError(f"CAN {r.can_id}: zero lies outside measured stops")
                    self._enter("MOVE_ZERO", state.position, now, r.zero_raw)
        elif self.budget_at is not None and now-self.budget_at >= 2:
            raise RuntimeError(f"CAN {r.can_id}: lead budget reached without confirmed contact")
        return self.q


class _Calibration:
    def __init__(self, arm):
        self.arm = arm
        self.lock = threading.Lock()
        self.run_lock = threading.Lock()
        previous = getattr(arm,"_calibration",None)
        self.report = deepcopy(previous.report) if previous is not None else CalibrationReport()
        self.started = None

    def status(self):
        with self.lock:
            report = deepcopy(self.report)
        if report.phase == "COMMITTING":
            saves = self.arm._loop._calibration_saves()
            by_id = dict(zip((int(m.can_id) for m in self.arm._config.motors), saves))
            for r in report.joints:
                r.save_state = by_id.get(r.can_id, r.save_state)
        return report

    def _publish(self, report):
        report.elapsed_s = time.monotonic()-self.started
        report.time_target_exceeded = report.elapsed_s > _TOTAL_TIME_TARGET
        with self.lock:
            self.report = deepcopy(report)

    def _parameters(self, kp, kd, gains):
        motors = list(self.arm._config.motors)
        if len(motors) != 10 or {int(m.can_id) for m in motors} != set(range(10)):
            raise ValueError("calibration requires all ten station joints")
        for m in motors:
            expected = _core.MotorModel.DM8009 if int(m.can_id)%5 < 2 else _core.MotorModel.DM4340P
            if m.model != expected:
                raise ValueError(f"CAN {m.can_id}: incorrect calibration motor model")
        if gains is not None:
            if kp is not None or kd is not None:
                raise ValueError("gains cannot be combined with kp/kd")
            if set(gains) != set(range(10)):
                raise ValueError("gains must specify all ten CAN IDs")
            values = {cid: tuple(gains[cid]) for cid in range(10)}
        else:
            values = {cid: (kp if kp is not None else 120.0 if cid in (0, 1, 5, 6) else 60.0,
                            5.0 if kd is None else kd) for cid in range(10)}
        for cid, pair in values.items():
            if len(pair) != 2 or not all(math.isfinite(float(v)) for v in pair):
                raise ValueError(f"CAN {cid}: invalid gains")
            p, d = map(float, pair)
            if not 0 < p <= 500 or not 0 < d <= 5:
                raise ValueError(f"CAN {cid}: require 0<kp<=500 and 0<kd<=5")
            values[cid] = (p, d)
        return motors, values

    def _states(self):
        loop = self.arm._loop
        if loop._calibration_cancelled():
            raise RuntimeError("zero calibration cancelled")
        if loop.safety_state() not in ("HOLD", "ACTIVE"):
            raise RuntimeError(f"calibration control failed: {loop.fault()}")
        states = {int(s.can_id): s for s in self.arm.states()}
        if set(states) != set(range(10)):
            raise RuntimeError("calibration feedback count mismatch")
        for cid, s in states.items():
            if (not s.feedback_fresh(0.3) or not s.enabled_confirmed or s.raw_status != 1 or
                    s.error_code != 0 or not all(math.isfinite(v) for v in (s.position,s.velocity,s.torque))):
                raise RuntimeError(f"CAN {cid}: feedback stale, invalid or faulted")
        return states

    def run(self, *, kp=None, kd=None, gains=None):
        if not self.run_lock.acquire(blocking=False):
            raise RuntimeError("zero calibration already executing")
        owned = False
        report = None
        loop = self.arm._loop
        try:
            motors, values = self._parameters(kp, kd, gains)
            previous_report = self.status()
            running = loop.running() if callable(loop.running) else loop.running
            resume = (running and loop.safety_state()=="FAULT_HOLD" and len(previous_report.joints)==10 and
                      all(r.save_state=="NOT_ATTEMPTED" for r in previous_report.joints))
            self.started = time.monotonic()
            report = CalibrationReport(phase="STARTING", joints=[
                JointZeroResult(cid, kp=values[cid][0], kd=values[cid][1]) for cid in range(10)])
            if resume:
                for old in previous_report.joints:
                    if old.measured_lower is not None:
                        report.joints[old.can_id] = deepcopy(old)
                        initial = values[old.can_id][0]
                        report.joints[old.can_id].kp = max(initial,min(old.kp,initial*2,500))
                        report.joints[old.can_id].kd = values[old.can_id][1]
            commands = [_core.MitCommand(kp=report.joints[int(m.can_id)].kp,
                                         kd=report.joints[int(m.can_id)].kd) for m in motors]
            # Own cleanup before crossing the GIL-releasing native startup call:
            # KeyboardInterrupt can arrive immediately when that call returns.
            owned = True
            self._publish(report)
            loop._begin_calibration(commands)
            states = self._states()
            hints = _endpoint_hints(previous_report,states)
            targets = {cid: s.position for cid, s in states.items()}
            now = time.monotonic()
            for r in report.joints:
                if r.phase=="DONE":
                    targets[r.can_id] = r.zero_raw
                    if abs(states[r.can_id].position-r.zero_raw)>=_TOL:
                        r.phase="MOVE_ZERO"
            def next_scan(side, first):
                for cid in range(first,side+5):
                    r=report.joints[cid]
                    if r.phase=="DONE":
                        continue
                    scan=_JointScan(r,states[cid].position,now,hints.get(cid))
                    if resume and r.zero_raw is not None:
                        scan._enter("MOVE_ZERO",states[cid].position,now,target=r.zero_raw)
                    elif resume and r.measured_lower is not None:
                        scan._enter("UPPER",states[cid].position,now)
                    return scan
                return None
            scans = {side:scan for side in (0,5) if (scan:=next_scan(side,side)) is not None}
            report.phase = "SCANNING"
            previous = now
            while scans:
                now = time.monotonic()
                dt = min(0.02, max(0.0, now-previous))
                previous = now
                states = self._states()
                for side, scan in list(scans.items()):
                    cid = scan.result.can_id
                    targets[cid] = scan.step(states[cid], now, dt)
                    if scan.result.phase == "DONE":
                        targets[cid] = scan.result.zero_raw
                        following=next_scan(side,cid+1)
                        if following is None:
                            del scans[side]
                        else:
                            scans[side]=following
                self._send(motors, targets, report)
                self._publish(report)
                time.sleep(0.004)
            report.phase = "CORRECTING"
            targets = {r.can_id: r.zero_raw for r in report.joints}
            began = time.monotonic()
            stable = None
            last_raise = {cid: began for cid in range(10)}
            while True:
                now = time.monotonic()
                states = self._states()
                all_near = True
                for r in report.joints:
                    r.final_error = abs(states[r.can_id].position-r.zero_raw)
                    if r.final_error >= _TOL:
                        all_near = False
                        if now-last_raise[r.can_id] >= 2:
                            initial = values[r.can_id][0]
                            r.kp = min(r.kp+initial*0.25, initial*2, 500)
                            last_raise[r.can_id] = now
                stable = (now if stable is None else stable) if all_near else None
                self._send(motors, targets, report)
                self._publish(report)
                if stable is not None and now-stable >= 1:
                    break
                if now-began >= 10:
                    raise RuntimeError("zero correction failed to converge within ten seconds")
                time.sleep(0.004)
            report.phase = "COMMITTING"
            self._publish(report)
            loop._calibration_commit([targets[int(m.can_id)] for m in motors])
            self._saves(report, motors)
            report.success = True
            report.phase = "DONE"
            self._publish(report)
            return deepcopy(report)
        except BaseException as exc:
            if owned:
                report.error = str(exc) or type(exc).__name__
                report.phase = "CANCELLED" if isinstance(exc, KeyboardInterrupt) or loop._calibration_cancelled() else "FAILED"
                try:
                    loop._calibration_fail(report.error)
                except BaseException as cleanup:
                    report.error += f"; hold failed: {cleanup}"
                    try:
                        loop.stop()
                    except BaseException as stop_error:
                        report.error += f"; stop failed: {stop_error}"
                self._saves(report, motors)
                self._publish(report)
                raise CalibrationError(report.error, report) from exc
            raise
        finally:
            try:
                if owned:
                    loop._finish_calibration()
            finally:
                self.run_lock.release()

    def _send(self, motors, targets, report):
        loop = self.arm._loop
        loop._calibration_update([
            _core.MitCommand(kp=report.joints[int(m.can_id)].kp,
                             kd=report.joints[int(m.can_id)].kd,
                             q=targets[int(m.can_id)], dq=0, tau=0) for m in motors])

    def _saves(self, report, motors):
        for m, saved in zip(motors, self.arm._loop._calibration_saves()):
            report.joints[int(m.can_id)].save_state = saved
