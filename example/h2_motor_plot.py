#!/usr/bin/env python3
"""
H2 Robot Joint Motor Visualizer
Plots command vs state for position and/or torque in real time.

Subscribes to:
  lowstate  (topstar_hg/msg/LowState) — motor_state[i].q, dq, tau_est
  /lowcmd   (topstar_hg/msg/LowCmd)   — motor_cmd[i].q, dq, kp, kd, tau (tau_ff)

Torque "cmd" shown is the full PD + feedforward estimate:
  τ_cmd = kp*(q_cmd − q_state) + kd*(dq_cmd − dq_state) + tau_ff
This is what the motor firmware will compute from the command, making it
directly comparable to tau_est even when tau_ff = 0 (pure PD control).

Usage:
    source ~/topstar_ros2/setup.sh
    python3 h2_motor_plot.py [options]

Joint groups: left_leg, right_leg, legs, left_arm, right_arm, arms, torso, all
Examples:
    python3 h2_motor_plot.py
    python3 h2_motor_plot.py --joints left_leg --mode torque
    python3 h2_motor_plot.py --joints 0 1 2 3 --mode both --window 5
    python3 h2_motor_plot.py --joints legs arms --cols 4
"""

import argparse
import collections
import threading
from typing import Dict, List, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

try:
    from topstar_hg.msg import LowState, LowCmd
except ImportError:
    print("ERROR: topstar_hg messages not found. Did you source setup.sh?")
    raise

import matplotlib
matplotlib.use('TkAgg')  # explicit GUI backend; fall back gracefully below
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ──────────────────────────────────────────────────────────────────────────────
# Joint metadata
# ──────────────────────────────────────────────────────────────────────────────

JOINT_NAMES: List[str] = [
    # Left leg  0-5
    'L_HipPitch', 'L_HipRoll', 'L_HipYaw', 'L_Knee', 'L_AnklePitch', 'L_AnkleRoll',
    # Right leg 6-11
    'R_HipPitch', 'R_HipRoll', 'R_HipYaw', 'R_Knee', 'R_AnklePitch', 'R_AnkleRoll',
    # Torso/Head 12-14
    'WaistYaw', 'HeadYaw', 'HeadPitch',
    # Left arm  15-21
    'L_ShoulderPitch', 'L_ShoulderRoll', 'L_ShoulderYaw', 'L_Elbow',
    'L_WristYaw', 'L_WristPitch', 'L_WristRoll',
    # Right arm 22-28
    'R_ShoulderPitch', 'R_ShoulderRoll', 'R_ShoulderYaw', 'R_Elbow',
    'R_WristYaw', 'R_WristPitch', 'R_WristRoll',
]

JOINT_GROUPS: Dict[str, List[int]] = {
    'left_leg':  list(range(0, 6)),
    'right_leg': list(range(6, 12)),
    'legs':      list(range(0, 12)),
    'torso':     list(range(12, 15)),
    'left_arm':  list(range(15, 22)),
    'right_arm': list(range(22, 29)),
    'arms':      list(range(15, 29)),
    'all':       list(range(0, 29)),
}

# ──────────────────────────────────────────────────────────────────────────────
# Thread-safe data buffer
# ──────────────────────────────────────────────────────────────────────────────

class MotorDataBuffer:
    """Stores a rolling time window of state+command samples."""

    def __init__(self, joint_indices: List[int], window_sec: float,
                 assumed_rate: float = 500.0):
        self.joint_indices = joint_indices
        self.window_sec = window_sec
        maxlen = int(window_sec * assumed_rate * 1.5)

        self._lock = threading.Lock()
        self._timestamps: collections.deque = collections.deque(maxlen=maxlen)

        self._state_q:   Dict[int, collections.deque] = {j: collections.deque(maxlen=maxlen) for j in joint_indices}
        self._state_dq:  Dict[int, collections.deque] = {j: collections.deque(maxlen=maxlen) for j in joint_indices}
        self._state_tau: Dict[int, collections.deque] = {j: collections.deque(maxlen=maxlen) for j in joint_indices}
        self._cmd_q:     Dict[int, collections.deque] = {j: collections.deque(maxlen=maxlen) for j in joint_indices}
        self._cmd_tau:   Dict[int, collections.deque] = {j: collections.deque(maxlen=maxlen) for j in joint_indices}

        # Latest command fields — updated on each /lowcmd message
        self._latest_cmd_q:      Dict[int, float] = {j: 0.0 for j in joint_indices}
        self._latest_cmd_dq:     Dict[int, float] = {j: 0.0 for j in joint_indices}
        self._latest_cmd_tau_ff: Dict[int, float] = {j: 0.0 for j in joint_indices}
        self._latest_cmd_kp:     Dict[int, float] = {j: 0.0 for j in joint_indices}
        self._latest_cmd_kd:     Dict[int, float] = {j: 0.0 for j in joint_indices}

    def update_cmd(self, msg: LowCmd) -> None:
        with self._lock:
            for j in self.joint_indices:
                mc = msg.motor_cmd[j]
                self._latest_cmd_q[j]      = float(mc.q)
                self._latest_cmd_dq[j]     = float(mc.dq)
                self._latest_cmd_tau_ff[j] = float(mc.tau)
                self._latest_cmd_kp[j]     = float(mc.kp)
                self._latest_cmd_kd[j]     = float(mc.kd)

    def update_state(self, msg: LowState, t: float) -> None:
        with self._lock:
            self._timestamps.append(t)
            for j in self.joint_indices:
                ms   = msg.motor_state[j]
                q    = float(ms.q)
                dq   = float(ms.dq)
                self._state_q[j].append(q)
                self._state_dq[j].append(dq)
                self._state_tau[j].append(float(ms.tau_est))
                self._cmd_q[j].append(self._latest_cmd_q[j])
                # Full PD + feedforward torque the firmware will produce:
                # τ = kp*(q_cmd − q) + kd*(dq_cmd − dq) + tau_ff
                tau_pd = (self._latest_cmd_kp[j] * (self._latest_cmd_q[j] - q)
                          + self._latest_cmd_kd[j] * (self._latest_cmd_dq[j] - dq)
                          + self._latest_cmd_tau_ff[j])
                self._cmd_tau[j].append(tau_pd)

    def snapshot(self, window_sec: Optional[float] = None):
        """Return a dict of numpy arrays trimmed to the requested time window."""
        ws = window_sec if window_sec is not None else self.window_sec
        with self._lock:
            if not self._timestamps:
                return None
            ts = np.array(self._timestamps)
            t_now = ts[-1]
            mask = ts >= (t_now - ws)
            ts_win = ts[mask]
            t_rel = ts_win - ts_win[0]  # relative time starting at 0

            result: Dict = {'t': t_rel, 't_abs': ts_win}
            for j in self.joint_indices:
                result[j] = {
                    'state_q':   np.array(self._state_q[j])[mask],
                    'state_tau': np.array(self._state_tau[j])[mask],
                    'cmd_q':     np.array(self._cmd_q[j])[mask],
                    'cmd_tau':   np.array(self._cmd_tau[j])[mask],
                }
        return result

# ──────────────────────────────────────────────────────────────────────────────
# ROS2 node
# ──────────────────────────────────────────────────────────────────────────────

class H2PlotNode(Node):
    def __init__(self, buffer: MotorDataBuffer):
        super().__init__('h2_motor_plot')
        self._buffer = buffer
        self._t0: Optional[float] = None

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(LowState, 'lowstate', self._on_state, best_effort_qos)
        self.create_subscription(LowCmd,   '/lowcmd',  self._on_cmd,   best_effort_qos)
        self.get_logger().info('H2 motor plot node ready — waiting for data...')

    def _on_state(self, msg: LowState) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = now
            self.get_logger().info('First lowstate received.')
        self._buffer.update_state(msg, now)

    def _on_cmd(self, msg: LowCmd) -> None:
        self._buffer.update_cmd(msg)

# ──────────────────────────────────────────────────────────────────────────────
# Matplotlib figure builder
# ──────────────────────────────────────────────────────────────────────────────

def _autoscale_y(ax, ys: List[np.ndarray], min_span: float = 0.05) -> None:
    all_y = np.concatenate(ys)
    if len(all_y) == 0:
        return
    lo, hi = float(all_y.min()), float(all_y.max())
    span = max(hi - lo, min_span)
    pad = span * 0.12
    ax.set_ylim(lo - pad, hi + pad)


def build_figure(joint_indices: List[int], mode: str, n_cols: int):
    channels = []
    if mode in ('pos',    'both'):
        channels.append('pos')
    if mode in ('torque', 'both'):
        channels.append('torque')

    n_joints   = len(joint_indices)
    total_axes = n_joints * len(channels)
    n_cols     = min(n_cols, total_axes)
    n_rows     = (total_axes + n_cols - 1) // n_cols

    fig_w = max(4.5 * n_cols, 8)
    fig_h = max(2.8 * n_rows, 4)
    fig, raw_axes = plt.subplots(n_rows, n_cols, figsize=(fig_w, fig_h), squeeze=False)
    fig.suptitle('H2 Joint Monitor  ·  cmd (dashed/orange) vs state (solid/blue)',
                 fontsize=11, fontweight='bold')

    flat_axes = raw_axes.flatten()
    ax_map: Dict = {}  # (joint_idx, channel) -> ax

    ax_idx = 0
    for j in joint_indices:
        for ch in channels:
            ax = flat_axes[ax_idx]
            unit = 'rad' if ch == 'pos' else 'Nm'
            label = 'Position' if ch == 'pos' else 'Torque  [cmd=kp·Δq+kd·Δdq+τff]'
            ax.set_title(f'{JOINT_NAMES[j]}  [{label}]', fontsize=8, pad=3)
            ax.set_xlabel('Time (s)', fontsize=7)
            ax.set_ylabel(unit, fontsize=7)
            ax.tick_params(labelsize=7)
            ax.grid(True, linewidth=0.4, alpha=0.5)
            ax_map[(j, ch)] = ax
            ax_idx += 1

    for i in range(ax_idx, len(flat_axes)):
        flat_axes[i].set_visible(False)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    return fig, ax_map

# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

def resolve_joints(tokens: List[str]) -> List[int]:
    indices = []
    for tok in tokens:
        if tok in JOINT_GROUPS:
            indices.extend(JOINT_GROUPS[tok])
        else:
            try:
                idx = int(tok)
                if 0 <= idx < 29:
                    indices.append(idx)
                else:
                    print(f'Warning: joint {idx} out of range [0,28], skipping.')
            except ValueError:
                print(f'Warning: unknown joint token "{tok}", skipping.')

    seen: set = set()
    return [j for j in indices if not (j in seen or seen.add(j))]


def main() -> None:
    parser = argparse.ArgumentParser(
        description='H2 Robot Joint Motor Visualizer',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        '--joints', nargs='+', default=['left_leg'],
        metavar='J',
        help='Joint indices (0-28) and/or group names. '
             'Groups: left_leg right_leg legs left_arm right_arm arms torso all. '
             'Default: left_leg')
    parser.add_argument(
        '--mode', choices=['pos', 'torque', 'both'], default='both',
        help='Channels to display (default: both)')
    parser.add_argument(
        '--window', type=float, default=10.0, metavar='SEC',
        help='Scrolling time window in seconds (default: 10)')
    parser.add_argument(
        '--cols', type=int, default=3, metavar='N',
        help='Number of subplot columns (default: 3)')
    parser.add_argument(
        '--rate', type=float, default=10.0, metavar='HZ',
        help='Plot refresh rate in Hz (default: 10)')

    # rclpy passes its own args; strip them before argparse sees them
    import sys
    args = parser.parse_args(args=[a for a in sys.argv[1:] if not a.startswith('--ros-args')])

    joint_indices = resolve_joints(args.joints)
    if not joint_indices:
        print('No valid joints specified. Exiting.')
        return

    print(f"Joints  : {[JOINT_NAMES[j] for j in joint_indices]}")
    print(f"Mode    : {args.mode}")
    print(f"Window  : {args.window} s")
    print(f"Refresh : {args.rate} Hz")

    # ── ROS2 setup ──────────────────────────────────────────────────
    rclpy.init()
    buffer = MotorDataBuffer(joint_indices, args.window)
    node   = H2PlotNode(buffer)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # ── Matplotlib figure ────────────────────────────────────────────
    fig, ax_map = build_figure(joint_indices, args.mode, args.cols)

    channels = []
    if args.mode in ('pos',    'both'): channels.append('pos')
    if args.mode in ('torque', 'both'): channels.append('torque')

    # Pre-create line objects
    line_pairs: Dict = {}
    for (j, ch), ax in ax_map.items():
        cmd_label   = 'cmd (q)' if ch == 'pos' else 'cmd (PD+ff)'
        state_label = 'state (q)' if ch == 'pos' else 'state (τ_est)'
        cmd_ln,   = ax.plot([], [], '--', color='tab:orange', lw=1.4, label=cmd_label)
        state_ln, = ax.plot([], [], '-',  color='tab:blue',   lw=1.4, label=state_label)
        ax.legend(loc='upper right', fontsize=7, framealpha=0.6)
        line_pairs[(j, ch)] = (cmd_ln, state_ln)

    interval_ms = max(50, int(1000.0 / args.rate))

    def update(_frame):
        data = buffer.snapshot()
        if data is None or len(data['t']) < 2:
            return [ln for pair in line_pairs.values() for ln in pair]

        t = data['t']
        t_lo, t_hi = float(t[0]), float(t[-1])

        for (j, ch), (cmd_ln, state_ln) in line_pairs.items():
            jd  = data[j]
            ax  = ax_map[(j, ch)]
            cmd_y   = jd['cmd_q']   if ch == 'pos' else jd['cmd_tau']
            state_y = jd['state_q'] if ch == 'pos' else jd['state_tau']

            cmd_ln.set_data(t, cmd_y)
            state_ln.set_data(t, state_y)
            ax.set_xlim(t_lo, max(t_hi, t_lo + 0.5))
            _autoscale_y(ax, [cmd_y, state_y])

        return [ln for pair in line_pairs.values() for ln in pair]

    ani = animation.FuncAnimation(  # noqa: F841 — must be kept alive
        fig, update, interval=interval_ms, blit=False, cache_frame_data=False)

    print("Plot open. Close window or press Ctrl+C to quit.")
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        print("Done.")


if __name__ == '__main__':
    main()
