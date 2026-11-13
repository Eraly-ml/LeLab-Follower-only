"""Single follower control: small, explicit joint steps from the current position."""
import logging
import math
import threading
import time

from pydantic import BaseModel
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from .utils.config import setup_follower_calibration_file
from .utils.devices import follower_lock, safe_disconnect_device
from . import teleoperate

class TargetRequest(BaseModel):
    joint: str
    angle: float

class CartesianMoveRequest(BaseModel):
    delta: list[float]

class ReleaseRequest(BaseModel):
    joint: str

class ManualRequest(BaseModel):
    follower_port: str
    follower_config: str

_lock = threading.RLock()
_robot = None
_last_seen = 0.0
_stop = threading.Event()
_last_command: dict[str, float] = {}
_enabled_motors: set[str] = set()
_ramp: dict[str, tuple[float, float]] = {}
_cartesian_solver = None
_cartesian_anchor = None
_cartesian_motors: set[str] = set()
_cartesian_last = 0.0
_last_reason = "Управление отключено"
logger = logging.getLogger(__name__)
_JOINTS = {"Rotation": "shoulder_pan", "Pitch": "shoulder_lift", "Elbow": "elbow_flex", "Wrist_Pitch": "wrist_flex", "Wrist_Roll": "wrist_roll", "Jaw": "gripper"}

def start(request: ManualRequest):
    global _robot, _last_seen, _last_reason
    from . import record, replay, rollout
    from .calibrate import calibration_manager
    with follower_lock, _lock:
        if calibration_manager.get_status().calibration_active:
            return {"success": False, "message": "Сначала завершите калибровку."}
        if teleoperate.teleoperation_active or record.recording_active or replay.replay_active or rollout.inference_active:
            return {"success": False, "message": "Рука уже используется другим процессом LeLab."}
        teleoperate.teleoperation_active = True
        robot = None
        try:
            name = setup_follower_calibration_file(request.follower_config)
            robot = SO101Follower(SO101FollowerConfig(port=request.follower_port, id=name, max_relative_target=12.0))
            robot.bus.connect()
            if not robot.calibration or not robot.is_calibrated:
                raise RuntimeError("Калибровка файла не совпадает с моторами. Сначала откалибруйте Robot (Follower).")
            # Start with torque disabled; a control click is required to move.
            # configure() enables all motors on exit, so do not call it here.
            modes = robot.bus.sync_read("Operating_Mode", normalize=False)
            if any(mode != 0 for mode in modes.values()):
                raise RuntimeError("Моторы должны быть в позиционном режиме; повторите калибровку follower")
            _enabled_motors.clear()
            _ramp.clear()
            _last_command.clear()
            _robot = robot
            _last_reason = "Управление подключено"
            _last_seen = time.monotonic()
            _stop.clear()
            threading.Thread(target=_watchdog, daemon=True).start()
            return {"success": True, "positions": robot.get_observation()}
        except Exception as exc:
            safe_disconnect_device(robot, __import__('logging').getLogger(__name__))
            _robot = None
            teleoperate.teleoperation_active = False
            return {"success": False, "message": str(exc)}

def _watchdog():
    while not _stop.wait(1):
        with _lock:
            if _robot is None:
                return
            if time.monotonic() - _last_seen > 5:
                stop("Связь со страницей потеряна: нет подтверждения более 5 секунд")
                return

def is_active():
    with _lock:
        return _robot is not None

def status():
    global _last_seen
    with _lock:
        if _robot is None:
            return {"success": False, "message": _last_reason}
        try:
            _last_seen = time.monotonic()
            return {"success": True}
        except Exception as exc:
            reason = f"Не удалось прочитать положение: {exc}"
            stop(reason)
            return {"success": False, "message": reason}

def move_target(request: TargetRequest):
    global _last_seen
    with _lock:
        if _robot is None:
            return {"success": False, "message": _last_reason}
        joint = _JOINTS.get(request.joint)
        if not joint or not math.isfinite(request.angle):
            return {"success": False, "message": "Некорректный сустав или угол"}
        now = time.monotonic()
        if now - _last_command.get(joint, 0.0) < 0.08:
            return {"success": True, "limited": True}
        try:
            robot = _robot
            current = float(robot.bus.read("Present_Position", joint))
            if not math.isfinite(current):
                raise RuntimeError("Не удалось прочитать положение сустава")
            calibration = robot.calibration[joint]
            if calibration.range_max - calibration.range_min < 100:
                raise RuntimeError("Некорректный диапазон калибровки")
            # The bundled visualizer maps gripper percentages to degrees.
            wanted = math.degrees(request.angle)
            if joint == "gripper":
                low, high = 0.0, 98.0
            else:
                half_range = (calibration.range_max - calibration.range_min) * 180.0 / 4095.0
                low, high = -half_range + 2.0, half_range - 2.0
            wanted = min(high, max(low, wanted))
            # Limit the goal relative to the measured position for force and
            # calibration safety, then ramp the commanded goal in time.
            bounded = max(current - 12.0, min(current + 12.0, wanted))
            previous, timestamp = _ramp.get(joint, (current, now - 0.08))
            if now - timestamp > 0.4:
                previous, timestamp = current, now - 0.08
            previous = max(current - 12.0, min(current + 12.0, previous))
            # Do not keep driving the old direction after the user reverses.
            if (wanted - current) * (previous - current) < 0:
                previous = current
            step = 30.0 * min(0.15, max(0.0, now - timestamp))
            target = max(previous - step, min(previous + step, bounded))
            if current < low and target <= current or current > high and target >= current:
                return {"success": False, "message": "Сустав у предела: двигайте его к середине диапазона"}
            if abs(target - current) < 0.05:
                _ramp[joint] = (current, now)
                return {"success": True}
            # Establish a present-position goal before enabling this motor.
            key = joint + ".pos"
            if joint not in _enabled_motors:
                robot.send_action({key: current})
                robot.bus.enable_torque(joint)
                _enabled_motors.add(joint)
            sent = robot.send_action({key: target})
            _ramp[joint] = (float(sent[key]), now)
            _last_seen = now
            _last_command[joint] = now
            return {"success": True, "target": target}
        except Exception as exc:
            logger.exception("Virtual control failed on %s", joint)
            reason = f"Ошибка управления суставом {joint}: {exc}"
            stop(reason)
            return {"success": False, "message": reason}

def release_target(request: ReleaseRequest):
    """Hold the measured position when dragging stops."""
    with _lock:
        joint = _JOINTS.get(request.joint)
        if _robot is None or not joint:
            return {"success": False, "message": _last_reason}
        try:
            _ramp.pop(joint, None)
            if joint in _enabled_motors:
                current = float(_robot.bus.read("Present_Position", joint))
                if not math.isfinite(current):
                    raise RuntimeError("Не удалось прочитать положение сустава")
                _robot.send_action({joint + ".pos": current})
            return {"success": True}
        except Exception as exc:
            logger.exception("Unable to hold joint after drag")
            reason = f"Ошибка остановки сустава {joint}: {exc}"
            stop(reason)
            return {"success": False, "message": reason}

def cartesian_begin():
    """Capture the actual pose without enabling torque or sending a target."""
    global _cartesian_solver, _cartesian_anchor, _cartesian_last
    with _lock:
        if _robot is None:
            return {"success": False, "message": _last_reason}
        try:
            from .cartesian_control import CartesianSolver, MOTORS
            if _cartesian_solver is None:
                _cartesian_solver = CartesianSolver()
            obs = _robot.get_observation()
            q = [float(obs[m + ".pos"]) for m in MOTORS]
            if not all(math.isfinite(x) for x in q):
                raise ValueError("Не удалось прочитать все суставы")
            _cartesian_anchor = _cartesian_solver.anchor(q)
            _cartesian_motors.clear()
            _cartesian_last = 0.0
            return {"success": True}
        except Exception as exc:
            logger.exception("Cartesian initialization failed")
            return {"success": False, "message": f"Не удалось включить режим всей руки: {exc}"}

def cartesian_move(request: CartesianMoveRequest):
    global _cartesian_last, _last_seen
    with _lock:
        if _robot is None or _cartesian_anchor is None:
            return {"success": False, "message": "Режим всей руки не запущен"}
        now = time.monotonic()
        if now - _cartesian_last < 0.10:
            return {"success": True, "limited": True}
        try:
            from .cartesian_control import MOTORS, MODEL_BOUNDS
            robot = _robot
            obs = robot.get_observation()
            q = [float(obs[m + ".pos"]) for m in MOTORS]
            bounds = []
            for m, (urdf_low, urdf_high) in zip(MOTORS, MODEL_BOUNDS, strict=True):
                cal = robot.calibration[m]
                half = (cal.range_max - cal.range_min) * 180.0 / 4095.0
                if half < 5:
                    raise ValueError("Некорректная калибровка сустава")
                bounds.append((max(-half + 2.0, urdf_low), min(half - 2.0, urdf_high)))
            solution = _cartesian_solver.solve(q, _cartesian_anchor, request.delta, bounds)
            commands = {}
            seeds = {}
            new_motors = []
            for motor, current, wanted in zip(MOTORS, q, solution, strict=True):
                if abs(wanted-current) < 0.1:
                    continue
                prev, stamp = _ramp.get(motor, (current, now - 0.1))
                if now - stamp > 0.4:
                    prev, stamp = current, now - 0.1
                prev = max(current - 12.0, min(current + 12.0, prev))
                if (wanted-current)*(prev-current) < 0:
                    prev = current
                bounded = max(current - 12.0, min(current + 12.0, wanted))
                step = 25.0 * min(0.15, max(0.0, now-stamp))
                target = max(prev-step, min(prev+step, bounded))
                commands[motor+'.pos'] = target
                if motor not in _enabled_motors:
                    seeds[motor+'.pos'] = current
                    new_motors.append(motor)
            if new_motors:
                robot.send_action(seeds)
                robot.bus.enable_torque(new_motors)
                _enabled_motors.update(new_motors)
            if commands:
                sent = robot.send_action(commands)
                for motor in MOTORS:
                    key=motor+'.pos'
                    if key in sent:
                        _ramp[motor] = (float(sent[key]), now)
                        _cartesian_motors.add(motor)
            _last_seen = now
            _cartesian_last = now
            return {"success": True}
        except ValueError as exc:
            # An unreachable target is an ordinary limit, not a serial fault.
            return {"success": False, "message": str(exc), "recoverable": True}
        except Exception as exc:
            logger.exception("Cartesian control failed")
            reason = f"Ошибка управления всей рукой: {exc}"
            stop(reason)
            return {"success": False, "message": reason}

def cartesian_release():
    global _cartesian_anchor
    with _lock:
        if _robot is None:
            return {"success": False, "message": _last_reason}
        try:
            obs = _robot.get_observation()
            hold = {m+'.pos': float(obs[m+'.pos']) for m in _cartesian_motors}
            if hold and not all(math.isfinite(x) for x in hold.values()):
                raise ValueError("Не удалось прочитать положение суставов")
            if hold:
                _robot.send_action(hold)
            for motor in _cartesian_motors:
                _ramp.pop(motor, None)
            _cartesian_motors.clear()
            _cartesian_anchor = None
            return {"success": True}
        except Exception as exc:
            logger.exception("Unable to stop Cartesian movement")
            stop(f"Ошибка остановки: {exc}")
            return {"success": False, "message": str(exc)}

def stop(reason="Управление отключено"):
    global _robot, _last_reason, _cartesian_anchor
    with _lock:
        if _robot is None:
            return {"success": True}
        _cartesian_anchor = None
        _cartesian_motors.clear()
        _last_reason = reason
        logger.warning("Virtual session stopped: %s", reason)
        _stop.set()
        robot, _robot = _robot, None
        safe_disconnect_device(robot, __import__('logging').getLogger(__name__))
        teleoperate.teleoperation_active = False
        return {"success": True}


def start_visualization(websocket_manager):
    """Publish the measured follower pose as a passive virtual leader pose."""
    def stream():
        while not _stop.wait(1 / 5):
            with _lock:
                if _robot is None:
                    return
                try:
                    joints = teleoperate.get_joint_positions_from_robot(_robot)
                    if websocket_manager and websocket_manager.active_connections:
                        websocket_manager.broadcast_joint_data_sync({"type": "joint_update", "joints": joints, "timestamp": time.time()})
                except Exception as exc:
                    logger.exception("Virtual visualization failed")
                    stop(f"Ошибка визуализации: {exc}")
                    return
    threading.Thread(target=stream, daemon=True, name="virtual-leader-visualizer").start()
