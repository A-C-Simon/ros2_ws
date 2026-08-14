#!/usr/bin/env python3
"""Multi-rover teleop — single terminal, digit keys switch active rover.

Usage:
    rover_teleop.py --rovers rover_0 rover_1 rover_2

Legacy single-rover form still works:
    rover_teleop.py -n rover_0 -y 0.0

Controls:
  arrows        accelerate ACTIVE rover in the arrow direction; release to coast
                down to a stop. Forward/back control linear speed, left/right
                control turn rate.
  SPACE         hard stop ACTIVE rover (zero velocity immediately)
  1..9          switch active rover (index into --rovers list, 1-based)
  b             broadcast toggle — same twist to ALL rovers (formation drive)
  q / a         active rover max linear speed +20% / -20%
  w / s         active rover max turn rate   +20% / -20%
  R             reset active rover to its spawn pose
  ?             reprint help
  Ctrl+C        quit

While any rover moves (or is still ramping), a live status line prints on
this terminal at 2 Hz: linear/angular velocity (v, ω) and acceleration
(a, α). It keeps updating through the release coast-down — the rover never
stops instantly — and prints one final "stopped" line when it comes to rest.

Motion limits are loaded from ROS parameters (see config/swarm.yaml). The
critical rule still holds: the teleop acceleration constants MUST match the
URDF DiffDrive plugin's max_linear_acceleration / max_angular_acceleration.
Mismatched limits cause double-filtering and constant jerk.
"""
import argparse
import sys
import select
import termios
import tty
import time
import subprocess
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

# Python-side defaults. These are also the ROS parameter fallbacks, so the
# node works when launched without a parameter file (legacy single-rover use).
# ─────────────────────────────────────────────────────────────────────────────
SPEED_INIT = 6.0    # initial linear speed ceiling (m/s) — URDF max_linear_velocity, no artificial cap
TURN_INIT = 3.0     # initial turn rate ceiling (rad/s) — URDF max_angular_velocity, no artificial cap
SPEED_MIN = 0.0     # allow creep / fine-positioning, not a hard 0.56 floor
SPEED_MAX = 6.0     # match URDF max_linear_velocity
TURN_MIN = 0.0
TURN_MAX = 3.0      # match URDF max_angular_velocity
SPEED_STEP = 1.2    # q/a/w/s scale factor
RELEASE_WINDOW = 0.12  # seconds without a linear/turn arrow before that axis coasts down
# MUST equal URDF DiffDrive max_linear_acceleration / max_angular_acceleration
LIN_ACCEL = 1.0     # m/s^2
ANG_ACCEL = 1.0     # rad/s^2
PUBLISH_PERIOD = 0.05  # 20 Hz
STATUS_PERIOD = 0.5    # 2 Hz live velocity/acceleration readout on the teleop terminal
# ─────────────────────────────────────────────────────────────────────────────

ARROW_UP, ARROW_DOWN, ARROW_RIGHT, ARROW_LEFT = '\x1b[A', '\x1b[B', '\x1b[C', '\x1b[D'


def get_key(poll=PUBLISH_PERIOD):
    r, _, _ = select.select([sys.stdin], [], [], poll)
    if not r:
        return ''
    key = sys.stdin.read(1)
    if key == '\x1b':
        # Arrow keys are ESC [ X. Drain follow-up bytes to avoid stray
        # characters being interpreted as bare keys on the next loop.
        for _ in range(2):
            r2, _, _ = select.select([sys.stdin], [], [], 0.25)
            if not r2:
                break
            key += sys.stdin.read(1)
    return key


def reset_rover(name, x=0.0, y=0.0, world='test_station'):
    subprocess.run([
        'ign', 'service', '-s', f'/world/{world}/set_pose',
        '--reqtype', 'ignition.msgs.Pose',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '500',
        '--req', f'name: "{name}", position: {{x: {x}, y: {y}, z: 0.2}}, orientation: {{w: 1}}',
    ], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


class RoverState:
    """Per-rover: publisher + ramp state + reset pose."""
    def __init__(self, node, namespace, spawn_x, spawn_y, cfg):
        self.cfg = cfg
        self.namespace = namespace
        self.spawn_x = spawn_x
        self.spawn_y = spawn_y
        topic = f'/{namespace}/cmd_vel' if namespace else '/cmd_vel'
        self.pub = node.create_publisher(Twist, topic, 10)
        self.target_lin = 0.0
        self.target_ang = 0.0
        self.cur_lin = 0.0
        self.cur_ang = 0.0
        # Axis-active flags: True while the matching arrow is being held.
        self.lin_active = False
        self.ang_active = False
        self.lin_dir = 0.0   # +1 forward, -1 reverse
        self.ang_dir = 0.0   # +1 left,   -1 right
        self.last_lin_arrow = 0.0
        self.last_ang_arrow = 0.0
        # Acceleration state, computed each tick for the status readout.
        self.accel_lin = 0.0
        self.accel_ang = 0.0
        # Per-rover live-adjustable speed profile (q/a/w/s modify the ACTIVE rover only).
        self.speed = cfg['speed_init']
        self.turn = cfg['turn_init']

    def _coast_axis(self, active, last_arrow, now):
        """Return True if too long has passed since the last arrow for this axis."""
        return active and (now - last_arrow) > self.cfg['release_window']

    def tick(self, now=None):
        """Ramp target up/down, ramp current toward target, publish."""
        if now is None:
            now = time.time()
        if self._coast_axis(self.lin_active, self.last_lin_arrow, now):
            self.lin_active = False
        if self._coast_axis(self.ang_active, self.last_ang_arrow, now):
            self.ang_active = False

        dt = self.cfg['publish_period']
        dv = self.cfg['linear_accel'] * dt
        dw = self.cfg['angular_accel'] * dt

        # Ramp the *target* velocity up while the key is held, down when released.
        if self.lin_active:
            self.target_lin += dv * self.lin_dir
            self.target_lin = max(-self.speed, min(self.speed, self.target_lin))
        else:
            if self.target_lin > 0.0:
                self.target_lin = max(0.0, self.target_lin - dv)
            elif self.target_lin < 0.0:
                self.target_lin = min(0.0, self.target_lin + dv)

        if self.ang_active:
            self.target_ang += dw * self.ang_dir
            self.target_ang = max(-self.turn, min(self.turn, self.target_ang))
        else:
            if self.target_ang > 0.0:
                self.target_ang = max(0.0, self.target_ang - dw)
            elif self.target_ang < 0.0:
                self.target_ang = min(0.0, self.target_ang + dw)

        # Snapshot current velocity so the acceleration readout can be derived
        # from the actual delta commanded this tick.
        prev_lin = self.cur_lin
        prev_ang = self.cur_ang

        # Smooth current velocity toward the target so the DiffDrive plugin sees
        # a feasible command (matches its own acceleration limit).
        self.cur_lin += max(-dv, min(dv, self.target_lin - self.cur_lin))
        self.cur_ang += max(-dw, min(dw, self.target_ang - self.cur_ang))
        # Clamp to the active rover's current speed/turn ceilings.
        self.cur_lin = max(-self.speed, min(self.speed, self.cur_lin))
        self.cur_ang = max(-self.turn, min(self.turn, self.cur_ang))

        # Current acceleration = velocity delta over the ramp step. Uses the same
        # fixed dt as the ramp model, so the readout matches the configured
        # linear_accel / angular_accel limits exactly: +limit while accelerating,
        # -limit while coasting down, ~0 at steady speed.
        self.accel_lin = (self.cur_lin - prev_lin) / dt
        self.accel_ang = (self.cur_ang - prev_ang) / dt

        t = Twist()
        t.linear.x = self.cur_lin
        t.angular.z = self.cur_ang
        self.pub.publish(t)


def declare_teleop_params(node):
    """Declare all motion-tuning parameters with script defaults as fallbacks."""
    node.declare_parameters(
        namespace='',
        parameters=[
            ('speed_init', SPEED_INIT),
            ('turn_init', TURN_INIT),
            ('speed_min', SPEED_MIN),
            ('speed_max', SPEED_MAX),
            ('turn_min', TURN_MIN),
            ('turn_max', TURN_MAX),
            ('speed_step', SPEED_STEP),
            ('release_window', RELEASE_WINDOW),
            ('linear_accel', LIN_ACCEL),
            ('angular_accel', ANG_ACCEL),
            ('publish_period', PUBLISH_PERIOD),
            ('status_period', STATUS_PERIOD),
        ]
    )


def load_teleop_cfg(node):
    """Return a plain dict of motion parameters from the ROS node."""
    return {
        'speed_init': float(node.get_parameter('speed_init').value),
        'turn_init': float(node.get_parameter('turn_init').value),
        'speed_min': float(node.get_parameter('speed_min').value),
        'speed_max': float(node.get_parameter('speed_max').value),
        'turn_min': float(node.get_parameter('turn_min').value),
        'turn_max': float(node.get_parameter('turn_max').value),
        'speed_step': float(node.get_parameter('speed_step').value),
        'release_window': float(node.get_parameter('release_window').value),
        'linear_accel': float(node.get_parameter('linear_accel').value),
        'angular_accel': float(node.get_parameter('angular_accel').value),
        'publish_period': float(node.get_parameter('publish_period').value),
        'status_period': float(node.get_parameter('status_period').value),
    }


def print_help(rovers, active_idx, broadcast, cfg):
    slots = '  '.join(
        f'[{i+1}]{r.namespace}{"*" if i == active_idx else ""}'
        for i, r in enumerate(rovers)
    )
    active = rovers[active_idx]
    mode = 'BROADCAST (all)' if broadcast else f'active: {active.namespace}'
    sys.stdout.write(
        f'\r\n=== Rover teleop  →  {mode} ===\r\n'
        f'  slots: {slots}   (* = active)\r\n'
        f'  active speed={active.speed:.2f} m/s  turn={active.turn:.2f} rad/s\r\n'
        f'  limits: lin [{cfg["speed_min"]:.2f}, {cfg["speed_max"]:.2f}] m/s  '
        f'ang [{cfg["turn_min"]:.2f}, {cfg["turn_max"]:.2f}] rad/s  '
        f'accel {cfg["linear_accel"]:.2f}/{cfg["angular_accel"]:.2f}\r\n'
        '  arrows accelerate / release coasts to stop   SPACE hard stop\r\n'
        '  status v/ω + a/α at 2 Hz while moving\r\n'
        '  1..9 select rover   b broadcast toggle   q/a speed ±20%   w/s turn ±20%\r\n'
        '  R reset active rover   ? help   Ctrl+C quit\r\n'
    )
    sys.stdout.flush()


def _motion_state(active, target, limit):
    """Return a single-word ramping state for one axis."""
    if active:
        if abs(target) >= limit - 1e-6:
            return 'max'
        return 'accel'
    if abs(target) > 1e-3:
        return 'coast'
    return 'stop'


def rover_moving(r):
    """True while a rover is actually moving OR still ramping up/down.

    Velocity alone misses the instant after a key press (still at 0 m/s while
    accelerating) and the ramp tail; the axis-active flags catch both, so the
    2 Hz readout starts on key-down and continues through the coast-down.
    """
    return (abs(r.cur_lin) > 1e-3 or abs(r.cur_ang) > 1e-3
            or r.lin_active or r.ang_active)


def print_status(rovers, active_idx, broadcast):
    """Overwrite the bottom line with direction words and current velocities."""
    active = rovers[active_idx]

    directions = []
    # Linear direction word (use current velocity when coasting, last direction when still active).
    if active.lin_active or abs(active.cur_lin) > 1e-3:
        sign = active.cur_lin if abs(active.cur_lin) > 1e-3 else active.lin_dir
        directions.append('forward' if sign > 0 else 'backward')
    # Angular direction word.
    if active.ang_active or abs(active.cur_ang) > 1e-3:
        sign = active.cur_ang if abs(active.cur_ang) > 1e-3 else active.ang_dir
        directions.append('left turn' if sign > 0 else 'right turn')

    if directions:
        dir_text = ' + '.join(directions)
    else:
        dir_text = 'stopped'

    # Overall ramping state: accel if any axis still ramping up, coast if any
    # axis is ramping down, max if active and at ceiling, otherwise stop.
    lin_state = _motion_state(active.lin_active, active.target_lin, active.speed)
    ang_state = _motion_state(active.ang_active, active.target_ang, active.turn)
    if 'accel' in (lin_state, ang_state):
        state = 'accel'
    elif 'coast' in (lin_state, ang_state):
        state = 'coast'
    elif 'max' in (lin_state, ang_state):
        state = 'max'
    else:
        state = 'stop'

    mode = 'BCAST' if broadcast else active.namespace
    line = (
        f'\r[{mode}] {dir_text} | '
        f'v {abs(active.cur_lin):.2f} m/s  ω {abs(active.cur_ang):.2f} rad/s | '
        f'a {active.accel_lin:+.2f} m/s²  α {active.accel_ang:+.2f} rad/s²  [{state}]'
    )
    sys.stdout.write(line + ' ' * 10 + '\r')
    sys.stdout.flush()


def main():
    argv = sys.argv[1:]
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    ap = argparse.ArgumentParser()
    ap.add_argument('--rovers', nargs='+', default=None,
                    help='Multi-rover mode: space-separated namespaces (e.g. rover_0 rover_1 rover_2)')
    ap.add_argument('--spawn-poses', nargs='+', default=None,
                    help='Per-rover spawn poses as "x,y" pairs, aligned with --rovers. '
                         'Comes from swarm.yaml via the launch file; used by the R reset key. '
                         'Overrides --spawn-spacing.')
    ap.add_argument('--spawn-spacing', type=float, default=0.8,
                    help='Fallback Y spacing between rovers when --spawn-poses is absent. Default 0.8.')
    ap.add_argument('--world', default='test_station',
                    help='Ignition world name for the reset service. MUST match the '
                         '<world name="..."> of the running .sdf (e.g. test_station, '
                         'rover_world), or the R reset key silently fails.')
    # Legacy single-rover flags (kept so old launch invocations still work).
    ap.add_argument('-n', '--namespace', default='')
    ap.add_argument('-x', '--spawn-x', type=float, default=0.0)
    ap.add_argument('-y', '--spawn-y', type=float, default=0.0)
    args = ap.parse_args(argv)

    rclpy.init()
    node = rclpy.create_node('rover_teleop')
    declare_teleop_params(node)
    cfg = load_teleop_cfg(node)

    # E-stop subscription: while True, teleop skips publishing so it doesn't fight
    # the estop_manager's zero-twist. This is the "downstream nodes MUST subscribe"
    # side of the fleet-wide E-stop contract.
    estop_state = {'stopped': False}
    def _on_estop(msg):
        estop_state['stopped'] = msg.data
    node.create_subscription(Bool, '/emergency_stop', _on_estop, 10)

    # Build the rover roster. Multi-rover form wins if provided; else fall back to legacy single.
    if args.rovers:
        if args.spawn_poses and len(args.spawn_poses) == len(args.rovers):
            spawn_xy = [tuple(float(v) for v in p.split(',')[:2]) for p in args.spawn_poses]
        else:
            if args.spawn_poses:
                sys.stderr.write('rover_teleop: --spawn-poses count != --rovers count; '
                                 'falling back to --spawn-spacing line layout\n')
            spawn_xy = [(args.spawn_x, i * args.spawn_spacing) for i in range(len(args.rovers))]
        rovers = [
            RoverState(node, ns, *spawn_xy[i], cfg)
            for i, ns in enumerate(args.rovers)
        ]
    else:
        rovers = [RoverState(node, args.namespace, args.spawn_x, args.spawn_y, cfg)]

    active_idx = 0
    broadcast = False
    status_last = 0.0   # wall-clock time of the last 2 Hz status draw
    was_moving = False  # True while the status line should stay live (moving/ramping)

    print_help(rovers, active_idx, broadcast, cfg)

    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())

    try:
        while True:
            key = get_key(poll=cfg['publish_period'])
            now = time.time()
            active = rovers[active_idx]

            # Digit → switch active rover
            if key in tuple('123456789'):
                idx = int(key) - 1
                if 0 <= idx < len(rovers):
                    active_idx = idx
                    print_help(rovers, active_idx, broadcast, cfg)
            elif key == 'b':
                # LOWERCASE only. Uppercase 'B' is the tail of arrow-DOWN's ESC[B
                # sequence — accepting it would false-trigger broadcast when the
                # user just drives backward.
                broadcast = not broadcast
                print_help(rovers, active_idx, broadcast, cfg)
            elif key == '?':
                print_help(rovers, active_idx, broadcast, cfg)
            elif key == ARROW_UP:
                targets = rovers if broadcast else [active]
                for r in targets:
                    r.lin_active = True
                    r.lin_dir = 1.0
                    r.last_lin_arrow = now
            elif key == ARROW_DOWN:
                targets = rovers if broadcast else [active]
                for r in targets:
                    r.lin_active = True
                    r.lin_dir = -1.0
                    r.last_lin_arrow = now
            elif key == ARROW_LEFT:
                targets = rovers if broadcast else [active]
                for r in targets:
                    r.ang_active = True
                    r.ang_dir = 1.0
                    r.last_ang_arrow = now
            elif key == ARROW_RIGHT:
                targets = rovers if broadcast else [active]
                for r in targets:
                    r.ang_active = True
                    r.ang_dir = -1.0
                    r.last_ang_arrow = now
            elif key == ' ':
                targets = rovers if broadcast else [active]
                for r in targets:
                    r.lin_active = False
                    r.ang_active = False
                    r.lin_dir = 0.0
                    r.ang_dir = 0.0
                    r.target_lin = 0.0
                    r.target_ang = 0.0
                    r.cur_lin = 0.0
                    r.cur_ang = 0.0
                    r.last_lin_arrow = 0.0
                    r.last_ang_arrow = 0.0
            elif key == 'q':
                # LOWERCASE only — same defense as 'b'/'r' below: uppercase
                # 'A' is the tail byte of the arrow-UP escape sequence (ESC[A),
                # and a garbled/split sequence can deliver it as its own key.
                # Treating it as "reduce speed" would silently shrink the speed
                # ceiling with every arrow push until the rover barely moves.
                active.speed = min(cfg['speed_max'], max(cfg['speed_min'], active.speed * cfg['speed_step']))
                sys.stdout.write(f'\r  {active.namespace} speed = {active.speed:.2f} m/s\r\n'); sys.stdout.flush()
            elif key == 'a':
                active.speed = min(cfg['speed_max'], max(cfg['speed_min'], active.speed / cfg['speed_step']))
                sys.stdout.write(f'\r  {active.namespace} speed = {active.speed:.2f} m/s\r\n'); sys.stdout.flush()
            elif key == 'w':
                active.turn = min(cfg['turn_max'], max(cfg['turn_min'], active.turn * cfg['speed_step']))
                sys.stdout.write(f'\r  {active.namespace} turn  = {active.turn:.2f} rad/s\r\n'); sys.stdout.flush()
            elif key == 's':
                active.turn = min(cfg['turn_max'], max(cfg['turn_min'], active.turn / cfg['speed_step']))
                sys.stdout.write(f'\r  {active.namespace} turn  = {active.turn:.2f} rad/s\r\n'); sys.stdout.flush()
            elif key == 'r':
                # LOWERCASE only. Uppercase 'R' avoids collision with any stray
                # follow-byte from a garbled escape sequence.
                name = active.namespace if active.namespace else 'rover'
                reset_rover(name, active.spawn_x, active.spawn_y, args.world)
                # Clear the motion state so the rover doesn't keep driving right
                # after the teleport.
                active.target_lin, active.target_ang = 0.0, 0.0
                active.cur_lin, active.cur_ang = 0.0, 0.0
                active.lin_active = False
                active.ang_active = False
                active.last_lin_arrow = 0.0
                active.last_ang_arrow = 0.0
                sys.stdout.write(f'\r  {name} reset to ({active.spawn_x:.1f}, {active.spawn_y:.1f})\r\n'); sys.stdout.flush()
            elif key == '\x03':  # Ctrl+C
                break

            # Pump ROS callbacks (e-stop subscription).
            rclpy.spin_once(node, timeout_sec=0.0)

            # Every rover ticks. If e-stop is engaged, publish zero-twist instead
            # of the ramped target — the estop_manager already floods zeros, but
            # this makes the intent explicit at the teleop layer too.
            if estop_state['stopped']:
                zero = Twist()
                for r in rovers:
                    r.lin_active = False
                    r.ang_active = False
                    r.lin_dir = 0.0
                    r.ang_dir = 0.0
                    r.target_lin = 0.0
                    r.target_ang = 0.0
                    r.cur_lin = 0.0
                    r.cur_ang = 0.0
                    r.last_lin_arrow = 0.0
                    r.last_ang_arrow = 0.0
                    r.pub.publish(zero)
            else:
                for r in rovers:
                    r.tick(now)

            # 2 Hz live velocity/acceleration readout. Drawn only while any
            # rover actually moves (or is still ramping up/down); the moment
            # everything comes to rest, one final "stopped" line is drawn and
            # the readout goes quiet until the next key press.
            moving = any(rover_moving(r) for r in rovers) if broadcast else rover_moving(active)
            if moving:
                if now - status_last >= cfg['status_period']:
                    print_status(rovers, active_idx, broadcast)
                    status_last = now
                was_moving = True
            elif was_moving:
                print_status(rovers, active_idx, broadcast)
                was_moving = False

    finally:
        # Publish a final zero-twist to every rover so nothing keeps rolling on exit.
        for r in rovers:
            r.pub.publish(Twist())
        sys.stdout.write('\r\n')
        sys.stdout.flush()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
