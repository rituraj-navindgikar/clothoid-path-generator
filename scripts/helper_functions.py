import threading
import math

from clothoid_arc_turn.solve import solveTrianlgeParams
from clothoid_arc_turn.fresnel_trigonometry import fresnelDAndDe
from clothoid_arc_turn.scene import Scene

import numpy as np


def show_plot_nonblocking(planner):
    threading.Thread(target=planner.plot(), daemon=True).start()


def sinE(delta, lmbda):
    eta = math.sqrt(2 * lmbda * abs(delta) / math.pi)
    s, _ = fresnelDAndDe(delta, lmbda)[0:2]
    return 2 * lmbda * delta * s + (1 - math.cos((1 - lmbda) * delta))

def cosE(delta, lmbda):
    _, c = fresnelDAndDe(delta, lmbda)[0:2]
    return 2 * lmbda * delta * c + math.sin((1 - lmbda) * delta)

def solveDeltaDeltaUnsymmetric(delta, delta_phi, lmbda, tol=1e-5, max_iter=20):
    delta_delta = 0.0  # Start symmetric
    for _ in range(max_iter):
        # Compute sinE and cosE for δ+Δδ and δ−Δδ
        sinE_plus  = sinE(delta + delta_delta, lmbda)
        cosE_plus  = cosE(delta + delta_delta, lmbda)
        sinE_minus = sinE(delta - delta_delta, lmbda)
        cosE_minus = cosE(delta - delta_delta, lmbda)

        dS = 0.5 * (sinE_plus - sinE_minus)
        C  = 0.5 * (cosE_plus + cosE_minus)

        lhs = math.atan2(dS, C)
        rhs = delta_delta - delta_phi
        f = lhs - rhs

        # Derivative
        d_dS = (cosE_plus + cosE_minus) / 2
        d_C  = (sinE_plus - sinE_minus) / 2
        d_f = (C * d_dS - dS * d_C) / (C**2 + dS**2) - 1

        if d_f == 0.0:
            break

        delta_step = -f / d_f
        delta_delta += delta_step

        if abs(delta_step) < tol:
            break

    return delta_delta

def findMidControlPoseUnsymmetric(start, end, delta_delta):
    x_diff = end[0] - start[0]
    y_diff = end[1] - start[1]
    T = 0.25 * math.sqrt(x_diff**2 + y_diff**2)
    theta = math.atan2(y_diff, x_diff)

    delta = (end[2] - start[2]) / 2
    delta_0 = delta + delta_delta
    delta_1 = delta - delta_delta

    theta_0 = start[2] - theta
    theta_1 = end[2] - theta

    mean_delta = -0.5 * (theta_0 + theta_1)

    delta_a = mean_delta - 0.5 * (delta_0 - delta_1)
    t_a = -2 * T * math.sin(mean_delta + delta - delta_delta / 2) / math.sin(delta_0 - delta_1)

    mid = np.array([
        start[0] + 2 * t_a * math.cos(mean_delta + delta - delta_delta / 2 + theta),
        start[1] + 2 * t_a * math.sin(mean_delta + delta - delta_delta / 2 + theta),
        start[2] + 2 * delta_a
    ], dtype=np.float64)

    return mid

def solveDualSceneUnsymmetric(start, end, lmbda, vehicle_half_width=1.0, vehicle_base_front=4.0):
    # Compute envelope triangle
    tri_params = solveTrianlgeParams(start, end, symmetry=False)
    if not tri_params:
        raise ValueError("Cannot solve triangle params for unsymmetric setup.")

    delta     = tri_params.delta
    delta_phi = tri_params.delta_phi

    # Solve delta_delta
    delta_delta = solveDeltaDeltaUnsymmetric(delta, delta_phi, lmbda)

    # Find mid pose
    mid_pose = findMidControlPoseUnsymmetric(start, end, delta_delta)

    # Construct scenes
    scene_a = Scene(start, mid_pose, lmbda, vehicle_half_width, vehicle_base_front, symmetry=False)
    scene_b = Scene(mid_pose, end, lmbda, vehicle_half_width, vehicle_base_front, symmetry=False)

    return scene_a, scene_b
