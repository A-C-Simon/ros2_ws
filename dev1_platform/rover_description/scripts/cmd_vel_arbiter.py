#!/usr/bin/env python3
"""Cmd-vel arbiter: single-writer twist mux in front of every rover's DiffDrive.

Subscribes three sources per rover and republishes the winner to /<ns>/cmd_vel_arb
— the topic the ros_gz bridge actually forwards into Ignition:

  /emergency_stop      (std_msgs/Bool, fleet-wide)   — HIGHEST: force zero
  /<ns>/cmd_vel_teleop (geometry_msgs/Twist)         — priority while "active":
                        within teleop_timeout of its last message
  /<ns>/cmd_vel        (geometry_msgs/Twist)         — autonomy / external
                        controller; lowest active priority (last message wins)

Priority: estop > teleop-active > autonomy > zero.

Why this exists: rover_teleop used to publish a zero-twist to /<ns>/cmd_vel on
every 20 Hz tick even with no key held, which (a) stomped any autonomy command
and (b) — combined with the DiffDrive's max_linear_acceleration ramp — capped
real speed at ~0.02 m/s. Now teleop writes its own /<ns>/cmd_vel_teleop topic
and goes SILENT once the rover reaches rest. The arbiter gives teleop priority
only while it is actually sending (plus a short handoff window), and autonomy
flows through the moment teleop goes quiet. It also re-emits the winner at
publish_rate, so the DiffDrive always sees a steady, non-flapping twist.

Teleop priority is measured in WALL time (the teleop publishes on wall time), so
the handoff latency is constant in real seconds regardless of sim RTF.
"""
import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool

# Script defaults; overridden by the cmd_vel_arbiter block in config/swarm.yaml.
TELEOP_TIMEOUT = 0.2   # s (WALL time) teleop keeps priority after its last message
PUBLISH_RATE = 30.0    # Hz — re-emit the winner to /<ns>/cmd_vel_arb


class CmdVelArbiter(Node):
    """Mux per-rover twist sources onto the single topic the bridge forwards."""

    def __init__(self, rover_namespaces):
        # Node name matches the YAML block header so params load cleanly.
        super().__init__('cmd_vel_arbiter',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        self.teleop_timeout = float(self._param('teleop_timeout', TELEOP_TIMEOUT))
        publish_rate = float(self._param('publish_rate', PUBLISH_RATE))
        self.estop = False
        self.last_source = {}   # ns -> 'estop'|'teleop'|'autonomy'|'zero' (for logs)

        # rovers[ns] = {'autonomy': (stamp, Twist) | None, 'teleop': (stamp, Twist) | None}
        self.rovers = {}
        self.pubs = {}
        for ns in rover_namespaces:
            self.rovers[ns] = {'autonomy': None, 'teleop': None}
            self.create_subscription(Twist, f'/{ns}/cmd_vel',
                                     self._on_twist_factory(ns, 'autonomy'), 10)
            self.create_subscription(Twist, f'/{ns}/cmd_vel_teleop',
                                     self._on_twist_factory(ns, 'teleop'), 10)
            self.pubs[ns] = self.create_publisher(Twist, f'/{ns}/cmd_vel_arb', 10)
            self.last_source[ns] = None

        self.create_subscription(Bool, '/emergency_stop', self._on_estop, 10)
        self.create_timer(1.0 / publish_rate, self.tick)

    def _param(self, name, default):
        if self.has_parameter(name):
            return self.get_parameter(name).value
        return default

    def _on_estop(self, msg):
        if msg.data != self.estop:
            self.get_logger().warn(f'E-STOP {"engaged" if msg.data else "released"}')
        self.estop = msg.data

    def _on_twist_factory(self, ns, key):
        def _on_twist(msg):
            if self.rovers[ns][key] is None:
                self.get_logger().info(f'{ns}: first {key} twist received '
                                       f'(vx={msg.linear.x:.2f})')
            # Stamped with WALL time: the teleop publishes on wall time, so the
            # handoff window must be wall-bounded too — a sim-time timeout would
            # stretch to seconds of real time at low RTF.
            self.rovers[ns][key] = (time.monotonic(), msg)
        return _on_twist

    def _teleop_active(self, ns):
        """True while a fresh teleop message is within the handoff window.

        Wall-clock comparison (teleop stamps are wall time): the rover keeps
        the last teleop velocity for teleop_timeout real seconds after the
        teleop's final message, then control falls through to autonomy.
        """
        tel = self.rovers[ns]['teleop']
        if tel is None:
            return False
        return time.monotonic() - tel[0] <= self.teleop_timeout

    def _select(self, ns):
        """Pick the winner for one rover; returns (source_name, Twist)."""
        if self.estop:
            return 'estop', Twist()
        if self._teleop_active(ns):
            return 'teleop', self.rovers[ns]['teleop'][1]
        autonomy = self.rovers[ns]['autonomy']
        if autonomy is not None:
            return 'autonomy', autonomy[1]
        return 'zero', Twist()

    def tick(self):
        for ns in self.rovers:
            source, twist = self._select(ns)
            if source != self.last_source[ns]:
                self.get_logger().info(f'{ns}: arbiter source -> {source}')
                self.last_source[ns] = source
            self.pubs[ns].publish(twist)


def main():
    argv = sys.argv[1:]
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    ap = argparse.ArgumentParser()
    ap.add_argument('--rovers', nargs='+', default=['rover_0'],
                    help='Rover namespaces to mux (space-separated)')
    args = ap.parse_args(argv)

    rclpy.init()
    node = CmdVelArbiter(args.rovers)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # One final zero so a dying arbiter never leaves a rover with a stale twist.
        for ns in args.rovers:
            node.pubs[ns].publish(Twist())
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
