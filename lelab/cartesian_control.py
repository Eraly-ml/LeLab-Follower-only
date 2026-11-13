"""Incremental Cartesian IK helpers for the SO-101.

This module only computes positions; callers are responsible for hardware I/O.
"""
from pathlib import Path
import math
import numpy as np
from lerobot.model.kinematics import RobotKinematics

JOINTS = ("Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll")
MOTORS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
MAX_OFFSET_M = 0.06
# Limits from the bundled SO-101 URDF, in degrees.
MODEL_BOUNDS = ((-110.0, 110.0), (-100.0, 100.0), (-100.0, 90.0), (-95.0, 95.0), (-160.0, 160.0))

class CartesianSolver:
    def __init__(self):
        root = Path(__file__).resolve().parent.parent
        model = root / 'frontend/public/so-101-urdf/urdf/so101_new_calib.urdf'
        meshes = (model.parent.parent / 'meshes').resolve()
        # Placo resolves absolute paths; the viewer's package URI is not a ROS installation.
        self.urdf = root.parent / 'so101_ik.urdf'
        self.urdf.write_text(model.read_text().replace('package://so_arm_description/meshes/', str(meshes) + '/'))
        self.kin = RobotKinematics(str(self.urdf), target_frame_name='gripper', joint_names=list(JOINTS))

    def anchor(self, q):
        return self.kin.forward_kinematics(np.asarray(q, dtype=float)).copy()

    def solve(self, q, anchor, delta, bounds):
        q = np.asarray(q, dtype=float)
        delta = np.asarray(delta, dtype=float)
        if q.shape != (5,) or delta.shape != (3,) or not np.all(np.isfinite(q)) or not np.all(np.isfinite(delta)):
            raise ValueError('Некорректная позиция или смещение')
        if np.linalg.norm(delta) > MAX_OFFSET_M + 1e-6:
            raise ValueError('Слишком большое перемещение за одно перетаскивание')
        target = anchor.copy(); target[:3, 3] += delta
        solution = np.asarray(self.kin.inverse_kinematics(q, target, orientation_weight=0, max_iters=15), dtype=float)
        if solution.shape != (5,) or not np.all(np.isfinite(solution)) or np.max(np.abs(solution - q)) > 25:
            raise ValueError('Положение недостижимо без большого движения суставов')
        for value, (low, high) in zip(solution, bounds, strict=True):
            if not (low <= value <= high):
                raise ValueError('Положение выходит за пределы калибровки')
        reached = self.kin.forward_kinematics(solution)
        if np.linalg.norm(reached[:3, 3] - target[:3, 3]) > 0.003:
            raise ValueError('Не удалось точно рассчитать движение')
        return solution
