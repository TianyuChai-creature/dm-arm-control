"""Deterministic calibration tests; no hardware and no real-time waiting."""
from types import SimpleNamespace
import math
import pytest

from dm_openarm import Arm, CalibrationError, _core
from dm_openarm import calibration as cal


class Clock:
    def __init__(self):
        self.now = 10.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += 0.02


def state(q=0.0, torque=0.0, sequence=1):
    return SimpleNamespace(position=q, velocity=0.0, torque=torque, rx_sequence=sequence,
                           last_rx_age_s=0.0, raw_status=1, enabled_confirmed=True,
                           error_code=0, feedback_fresh=lambda age: True)


def config():
    c = _core.ArmConfig()
    motors = []
    # Exercise the station's non-sorted vector order.
    for cid in (4,2,3,1,0,9,8,7,6,5):
        m = _core.MotorConfig()
        m.can_id, m.mst_id = cid, cid+16
        m.model = _core.MotorModel.DM8009 if cid%5 < 2 else _core.MotorModel.DM4340P
        motors.append(m)
    c.motors = motors
    left, right = _core.LimbSpec(), _core.LimbSpec()
    left.count = right.count = 5
    right.begin = 5
    c.left, c.right = left, right
    return c


class Plant:
    def __init__(self, cfg, clock):
        self.ids = [int(m.can_id) for m in cfg.motors]
        self.clock = clock
        self.feedback = {cid: state(0.0) for cid in self.ids}
        for cid, s in self.feedback.items():
            s.can_id = cid
        self.commands = None
        self.cancelled = False
        self.running = False
        self.safety = "DISABLED"
        self.saved = ["NOT_ATTEMPTED"] * 10
        self.begin_count = 0
        self.commit_count = 0
        self.hold_reason = ""
        self.history = []
        self.stop_ids = None
        self.partial_failure = False
        self.stale = False
        self.cancel_at = None

    def _begin_calibration(self, commands):
        self.begin_count += 1
        self.running = True
        self.safety = "HOLD"
        self.commands = commands

    def safety_state(self):
        return self.safety

    def fault(self):
        return ""

    def _calibration_cancelled(self):
        return self.cancelled or (self.cancel_at is not None and self.clock.now > self.cancel_at)

    def _calibration_update(self, commands):
        assert self.running
        for cid, cmd in zip(self.ids, commands):
            assert cmd.dq == 0 and cmd.tau == 0 and cmd.kp > 0 and cmd.kd > 0
            lower, upper = cal._RANGES[cid]
            q = max(lower+0.01, min(upper+0.01, cmd.q))
            if self.stop_ids is not None and cid in self.stop_ids:
                q = self.stop_ids[cid]
            s = self.feedback[cid]
            s.position = q
            s.torque = cmd.kp*(cmd.q-q)
        self.commands = commands
        self.history.append(tuple(c.q for c in commands))

    def states(self):
        for s in self.feedback.values():
            s.rx_sequence += 1
            s.feedback_fresh = lambda age: not self.stale
        return list(self.feedback.values())

    def _calibration_fail(self, reason):
        self.hold_reason = reason
        self.safety = "FAULT_HOLD"

    def stop(self):
        self.running = False

    def _finish_calibration(self):
        pass

    def _calibration_commit(self, zeros):
        self.commit_count += 1
        assert all(abs(self.feedback[cid].position-q) < cal._TOL for cid,q in zip(self.ids,zeros))
        if self.partial_failure:
            self.saved[0], self.saved[1] = "SAVED", "UNKNOWN"
            self.running = False
            raise RuntimeError("save unconfirmed")
        self.saved = ["SAVED"] * 10
        self.running = False
        self.commands = [_core.MitCommand() for _ in self.ids]
        self.safety = "DISABLED"

    def _calibration_saves(self):
        return self.saved


def runner(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(cal, "time", clock)
    cfg = config()
    plant = Plant(cfg, clock)
    arm = SimpleNamespace(_config=cfg, _loop=plant, states=plant.states)
    return cal._Calibration(arm), plant, clock


@pytest.mark.parametrize("kwargs", [
    {"kp":0}, {"kd":0}, {"kp":501}, {"kd":6}, {"kp":math.nan},
    {"gains":{0:(160,5)}}, {"kp":160,"gains":{cid:(160,5) for cid in range(10)}},
])
def test_invalid_gains_before_enable(monkeypatch, kwargs):
    run, plant, _ = runner(monkeypatch)
    with pytest.raises(ValueError):
        run.run(**kwargs)
    assert plant.begin_count == 0 and run.status().phase == "IDLE"


def test_defaults_and_complete_parallel_scan(monkeypatch):
    run, plant, clock = runner(monkeypatch)
    began = clock.now
    report = run.run()
    assert clock.now-began <= 120
    assert all(0 < r.elapsed_s <= 12 for r in report.joints)
    assert report.success and report.phase == "DONE" and plant.commit_count == 1
    for r in report.joints:
        assert abs(r.zero_raw-0.01) < 0.002
        assert abs(r.span_error) < 0.005
        assert r.final_error < cal._TOL
        assert r.kp == (120 if r.can_id in (0,1,5,6) else 60) and r.kd == 5 and r.save_state == "SAVED"
    # First cycle commands both J1s while all other axes retain their initial targets.
    first = next(row for row in plant.history if any(abs(q)>1e-8 for q in row))
    by_id = dict(zip(plant.ids,first))
    assert by_id[0] < 0 and by_id[5] < 0
    assert all(by_id[cid] == 0 for cid in range(10) if cid not in (0,5))
    assert not plant.running and plant.safety == "DISABLED"
    report.joints[0].save_state = "edited"
    assert run.status().joints[0].save_state == "SAVED"


def test_contact_rejects_cache_spike_and_motion():
    for moving, spike, cached in ((False,False,True),(False,True,False),(True,False,False)):
        contact = cal._Contact()
        result = None
        for i in range(25):
            t = i*0.05
            torque = 1.0 if spike and i == 16 else 0.0 if spike else t
            s = state(t*0.02 if moving else 0, torque, 1 if cached else i+1)
            result = contact.update(s,t*0.02,1,t,0.1)
            assert result is None


def test_contact_requires_both_conditions():
    contact = cal._Contact()
    for i in range(30):
        t = i*0.05
        found = contact.update(state(0,t,i+1),t*0.02,1,t,0.1)
        if found is not None:
            assert found == 0
            break
    else:
        pytest.fail("steady contact was not detected")


def test_cancel_holds_without_commit(monkeypatch):
    run, plant, clock = runner(monkeypatch)
    plant.cancel_at = clock.now+1
    with pytest.raises(CalibrationError) as error:
        run.run(kp=120,kd=4)
    assert error.value.report.phase == "CANCELLED"
    assert plant.safety == "FAULT_HOLD" and plant.running and plant.commit_count == 0
    assert all(r.kp == 120 and r.kd == 4 for r in run.status().joints)


def test_different_measured_span_is_interpolated_without_span_gate(monkeypatch):
    run, plant, _ = runner(monkeypatch)
    original = plant._calibration_update
    def obstruct(commands):
        original(commands)
        s = plant.feedback[0]
        s.position = max(-0.3,s.position)
        cmd = commands[plant.ids.index(0)]
        s.torque = cmd.kp*(cmd.q-s.position)
    plant._calibration_update = obstruct
    report=run.run()
    r=report.joints[0]
    assert report.success
    assert abs(r.measured_lower+.3)<.002
    expected=cal._zero_position(r.measured_lower,r.measured_upper,*cal._RANGES[0])
    assert abs(r.zero_raw-expected)<1e-9


def test_partial_save_report(monkeypatch):
    run, plant, _ = runner(monkeypatch)
    plant.partial_failure = True
    with pytest.raises(CalibrationError) as error:
        run.run()
    states = {r.can_id:r.save_state for r in error.value.report.joints}
    assert states[4] == "SAVED" and states[2] == "UNKNOWN" and states[0] == "NOT_ATTEMPTED"
    assert plant.commit_count == 1


def test_lead_budget_without_load_stops(monkeypatch):
    run, plant, _ = runner(monkeypatch)
    def stuck(commands):
        plant.commands = commands
        assert all(abs(c.q) <= 0.413 for c in commands)
    plant._calibration_update = stuck
    with pytest.raises(CalibrationError,match="without confirmed contact"):
        run.run(kp=160, kd=5)
    assert plant.commit_count == 0


def test_stale_feedback_prevents_commands(monkeypatch):
    run, plant, _ = runner(monkeypatch)
    plant.stale = True
    with pytest.raises(CalibrationError,match="feedback stale"):
        run.run()
    assert plant.commit_count == 0 and not plant.history


def test_python_surface():
    assert callable(Arm.calibrate_zero)
    assert callable(Arm.calibration_status)
    assert callable(Arm.cancel_zero_calibration)


def finish_scans_immediately(monkeypatch):
    class Scan:
        def __init__(self, result, position, now, hints=None):
            self.result = result
        def step(self, state, now, dt):
            self.result.zero_raw = 0.01
            self.result.phase = "DONE"
            return 0.01
    monkeypatch.setattr(cal,"_JointScan",Scan)


@pytest.mark.parametrize("initial,load,expected", [(160,4,240),(300,8,500)])
def test_gain_correction_converges_with_bounded_kp(monkeypatch, initial, load, expected):
    finish_scans_immediately(monkeypatch)
    run, plant, _ = runner(monkeypatch)
    original = plant._calibration_update
    def loaded(commands):
        original(commands)
        for cid,c in zip(plant.ids,commands):
            plant.feedback[cid].position = c.q-load/c.kp
    plant._calibration_update = loaded
    report = run.run(kp=initial,kd=4)
    assert report.success and all(r.kp == expected and r.kd == 4 for r in report.joints)
    assert all(r.kp <= min(initial*2,500) and r.final_error < cal._TOL for r in report.joints)


def test_correction_timeout_never_writes(monkeypatch):
    finish_scans_immediately(monkeypatch)
    run, plant, _ = runner(monkeypatch)
    original = plant._calibration_update
    def stuck(commands):
        original(commands)
        for cid in plant.ids:
            plant.feedback[cid].position = -0.1
    plant._calibration_update = stuck
    with pytest.raises(CalibrationError,match="correction failed") as error:
        run.run(kp=300,kd=5)
    assert all(r.kp == 500 for r in error.value.report.joints)
    assert plant.commit_count == 0


def test_startup_failure_carries_report_and_unlocks(monkeypatch):
    run, plant, _ = runner(monkeypatch)
    def fail(commands):
        raise RuntimeError("startup failed")
    plant._begin_calibration = fail
    with pytest.raises(CalibrationError,match="startup failed") as error:
        run.run()
    assert error.value.report.phase == "FAILED" and not run.run_lock.locked()


def test_reentrant_call_is_rejected(monkeypatch):
    run, plant, _ = runner(monkeypatch)
    run.run_lock.acquire()
    try:
        with pytest.raises(RuntimeError,match="already executing"):
            run.run()
        assert plant.begin_count == 0
    finally:
        run.run_lock.release()


def test_per_axis_gains_and_model_validation(monkeypatch):
    run, plant, _ = runner(monkeypatch)
    gains = {cid:(100+cid, 3+cid/10) for cid in range(10)}
    motors, values = run._parameters(None, None, gains)
    assert values == gains and [int(m.can_id) for m in motors] == plant.ids
    invalid = run.arm._config.motors
    invalid[0].model = _core.MotorModel.DM4310
    run.arm._config.motors = invalid
    with pytest.raises(ValueError,match="incorrect calibration motor model"):
        run.run()
    assert plant.begin_count == 0


def test_joint_timing_target_does_not_stop_scan_at_phase_change():
    result = cal.JointZeroResult(0)
    scan = cal._JointScan(result,0,0)
    scan._enter("UPPER",0,11)
    scan.step(state(),12.01,0.004)
    assert result.phase == "UPPER" and result.time_target_exceeded
    assert result.elapsed_s == pytest.approx(12.01)


def test_total_timing_target_reports_without_blocking_feedback(monkeypatch):
    run, plant, clock = runner(monkeypatch)
    run.started = clock.now-121
    plant.safety = "HOLD"
    assert len(run._states()) == 10
    report = cal.CalibrationReport()
    run._publish(report)
    assert run.status().time_target_exceeded
    assert run.status().elapsed_s == pytest.approx(121)


def test_search_is_not_gated_by_ideal_endpoint_proximity():
    r=cal.JointZeroResult(3)
    scan=cal._JointScan(r,-.3,0)
    scan.contact=SimpleNamespace(update=lambda *args:-.3)
    scan.step(state(-.3),.01,.01)
    assert r.phase=="BACKOFF_LOWER"


def test_fast_scan_with_100ms_cached_feedback(monkeypatch):
    run, plant, clock = runner(monkeypatch)
    original = plant.states
    received = None
    cached = None
    def delayed():
        nonlocal received, cached
        now = clock.now
        if received is None or now-received >= 0.1-1e-9:
            previous = {s.can_id:s.position for s in cached} if cached else None
            cached = [SimpleNamespace(**vars(s)) for s in original()]
            if previous is not None:
                for s in cached:
                    s.velocity = (s.position-previous[s.can_id])/(now-received)
            received = now
        for s in cached:
            s.last_rx_age_s = now-received
        return cached
    run.arm.states = delayed
    report = run.run()
    assert report.success and report.elapsed_s <= 120
    assert all(r.elapsed_s <= 12 and abs(r.zero_raw-0.01) < 0.005 for r in report.joints)


def test_contact_detects_load_plateau_after_delayed_stall_feedback():
    contact = cal._Contact()
    found = None
    for i in range(11):
        t = i*0.1
        q = min(0.0,-0.04+t*0.2)
        # Load rises at the stop, then stays flat while the command is capped.
        torque = 0.2 if i<2 else 8.0
        command = q if i<2 else 0.05
        found = contact.update(state(q,torque,i+1),command,1,t,0.15)
        if found is not None:
            assert abs(found) < 1e-9
            assert t <= 0.8
            break
    else:
        pytest.fail("plateau after sustained load rise must confirm the stop")


def test_constant_preexisting_load_is_not_contact():
    contact = cal._Contact()
    for i in range(20):
        assert contact.update(state(0,8,i+1),min(0.05,i*.01),1,i*.1,.15) is None


def test_backoff_uses_actual_clearance_not_zero_load_tracking_error():
    r=cal.JointZeroResult(0)
    scan=cal._JointScan(r,0,0)
    scan.first=0
    scan._enter("BACKOFF_LOWER",0,0,target=.03)
    scan.q=.03
    for i in range(1,12):
        scan.step(state(.022),i*.02,.02)
        if r.phase=="REPEAT_LOWER":
            break
    assert r.phase=="REPEAT_LOWER"


def test_backoff_cannot_finish_without_physical_clearance():
    r=cal.JointZeroResult(0)
    scan=cal._JointScan(r,0,0)
    scan.first=0
    scan._enter("BACKOFF_LOWER",0,0,target=.03)
    scan.q=.03
    for i in range(1,20):
        scan.step(state(.005),i*.02,.02)
    assert r.phase=="BACKOFF_LOWER"


def test_move_zero_accepts_one_degree_and_boosts_when_needed():
    r=cal.JointZeroResult(0)
    scan=cal._JointScan(r,0,0)
    scan._enter("MOVE_ZERO",0,0,target=0)
    scan.step(state(-.02),2.01,.02)
    assert r.kp==200 and r.phase=="MOVE_ZERO"
    for i in range(1,12):
        scan.step(state(-.016),2.01+i*.02,.02)
        if r.phase=="DONE":break
    assert r.phase=="DONE"


@pytest.mark.parametrize("extra_degrees",[4.99,5.01,12.0])
def test_measured_span_difference_does_not_reject_zero(extra_degrees):
    lower,upper=cal._RANGES[0]
    upper+=math.radians(extra_degrees)
    r=cal.JointZeroResult(0,measured_lower=lower)
    scan=cal._JointScan(r,upper,0)
    scan.first=upper
    scan._enter("REPEAT_UPPER",upper,0)
    scan.contact=SimpleNamespace(update=lambda *args:upper)
    scan.step(state(upper),.01,.01)
    assert r.phase=="MOVE_ZERO"
    assert r.zero_raw==pytest.approx(cal._zero_position(lower,upper,*cal._RANGES[0]))


def test_small_motor_touch_budget_is_lower_without_reducing_gains():
    big=cal._JointScan(cal.JointZeroResult(0),0,0)
    small=cal._JointScan(cal.JointZeroResult(2),0,0)
    assert big._lead(cal._TOUCH_RATE)==.05
    assert small._lead(cal._TOUCH_RATE)==.025
    assert small.result.kp==160 and small.result.kd==5


def test_shrinking_touch_budget_releases_excess_reference_pressure():
    r=cal.JointZeroResult(2)
    scan=cal._JointScan(r,cal._RANGES[2][0],0)
    scan.q=scan.lower-.04
    previous=scan.q
    scan.step(state(scan.lower),.02,.02)
    assert scan.q>previous


def test_asymmetric_zero_is_interpolated_not_average_offset():
    # Ideal -2..1 and measured 10..16: zero is 2/3 of the measured span.
    assert cal._zero_position(10,16,-2,1)==14
    assert cal._zero_position(10,16,-2,1)!=(10+2+16-1)/2
    assert cal._zero_position(10,16,0,2)==10
    assert cal._zero_position(10,16,-1,1)==13


@pytest.mark.parametrize("lower,upper",[(1,1),(2,1),(math.nan,3),(1,math.inf)])
def test_invalid_measured_endpoints_still_fail(lower,upper):
    with pytest.raises(ValueError,match="endpoint pair"):
        cal._zero_position(lower,upper,-1,1)


def test_retry_hints_use_measured_stops_and_account_for_flash_zero():
    lower=cal.JointZeroResult(9,measured_lower=-2.56,phase="REPEAT_UPPER")
    saved=cal.JointZeroResult(0,measured_lower=-3.5,measured_upper=1.4,zero_raw=-.05,save_state="SAVED")
    previous=cal.CalibrationReport(joints=[lower,saved])
    hints=cal._endpoint_hints(previous,{9:state(.69)})
    assert hints[9]=={"lower":-2.56}
    assert hints[0]["lower"]==pytest.approx(-3.45)
    scan=cal._JointScan(cal.JointZeroResult(9),.69,0,hints[9])
    assert scan.hint_lower==-2.56 and scan.hint_upper==pytest.approx(-2.56+cal._RANGES[9][1]-cal._RANGES[9][0])
    assert scan.lower==cal._RANGES[9][0]


def test_low_speed_motion_allows_damping_and_transport_lag_without_high_stop_pressure():
    scan=cal._JointScan(cal.JointZeroResult(2),0,0)
    stopped=scan._lead(.2)
    scan.feedback_velocity=.2
    assert scan._lead(.2)>stopped
    scan.feedback_velocity=0
    assert scan._lead(.2)==.025


def test_stalled_motor_in_both_directions_cannot_be_committed_as_a_stop(monkeypatch):
    run,plant,clock=runner(monkeypatch)
    plant.feedback[4].position=-1.33
    plant.stop_ids={4:-1.33}
    plant.cancel_at=clock.now+130
    with pytest.raises(CalibrationError,match="cancelled") as error:
        run.run()
    assert error.value.report.joints[4].zero_raw is None
    assert plant.commit_count==0 and plant.safety=="FAULT_HOLD"


@pytest.mark.parametrize("phase,first,endpoint,next_phase", [
    ("REPEAT_LOWER", -1.0, -1.0164, "UPPER"),
    ("REPEAT_UPPER", 1.0, 1.0164, "MOVE_ZERO"),
])
def test_repeat_error_is_recorded_without_stopping(phase, first, endpoint, next_phase):
    result = cal.JointZeroResult(2, measured_lower=-1.4)
    scan = cal._JointScan(result, first, 0)
    scan.first = first
    scan._enter(phase, first, 0)
    scan.contact = SimpleNamespace(update=lambda *args: endpoint)
    scan.step(state(endpoint), .1, .004)
    assert result.phase == next_phase
    assert result.repeat_error == pytest.approx(.0164)
    assert (result.measured_lower if phase == "REPEAT_LOWER" else result.measured_upper) == endpoint


def test_unconfirmed_stop_is_not_reused_as_a_trusted_motion_hint():
    r=cal.JointZeroResult(4,phase="BACKOFF_LOWER")
    assert cal._endpoint_hints(cal.CalibrationReport(joints=[r]),{4:state(-1.33)})=={}


def test_already_near_zero_can_finish_after_timing_target():
    r=cal.JointZeroResult(0)
    scan=cal._JointScan(r,0,0)
    scan._enter("MOVE_ZERO",0,11,target=0)
    scan.step(state(.001),12.01,.01)
    scan.step(state(.001),12.15,.01)
    assert r.phase=="DONE"


def test_move_zero_continues_after_joint_and_total_timing_targets():
    for position,now in ((.1,12.01),(.001,121)):
        result=cal.JointZeroResult(0)
        scan=cal._JointScan(result,0,0)
        scan._enter("MOVE_ZERO",0,11,target=0)
        scan.step(state(position),now,.01)
        assert result.phase == "MOVE_ZERO" and result.time_target_exceeded


def test_retry_retains_completed_axes_in_same_unwritten_hold(monkeypatch):
    run,plant,clock=runner(monkeypatch)
    previous=cal.CalibrationReport(phase="FAILED",joints=[cal.JointZeroResult(cid) for cid in range(10)])
    for r in previous.joints:
        if r.can_id%5<4:
            lower,upper=cal._RANGES[r.can_id]
            r.measured_lower=lower+.01
            r.measured_upper=upper+.01
            r.zero_raw=.01
            r.phase="DONE"
            plant.feedback[r.can_id].position=.01
    run.report=previous
    plant.running=True
    plant.safety="FAULT_HOLD"
    report=run.run()
    assert report.success and report.elapsed_s<15
    for row in plant.history:
        assert all(abs(q-.01)<1e-9 for cid,q in zip(plant.ids,row) if cid%5<4)


def test_backoff_compensates_static_tracking_error_with_bounded_extra_distance():
    r=cal.JointZeroResult(4)
    scan=cal._JointScan(r,0,0)
    scan.first=0
    scan._enter("BACKOFF_LOWER",0,0,target=.03)
    scan.q=.03
    scan.step(state(.013),.01,.01)
    assert scan.target==pytest.approx(.047)
    assert .03<=scan.target<=.06 and r.phase=="BACKOFF_LOWER"
