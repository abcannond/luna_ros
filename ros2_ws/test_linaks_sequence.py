#!/usr/bin/env python3
"""
Standalone dig-and-dump sequence test.

Calls /luna_linaks_node/* Trigger services directly — no Nav2, no mission supervisor.
Run with: ros2 run luna_nav test_linaks_sequence  (after registering)
     or:  python3 test_linaks_sequence.py         (from this directory, with ROS sourced)

Steps:
  1. Excavation — shoulder down, wrist level, drive, wrist up, shoulder up
  2. Dump       — lift extend, reverse slowly, stop

All timing values are set conservatively; tune to match hardware.
"""

import time
import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist


# ── tuneable timing (seconds) ────────────────────────────────────────────────
EXCAVATION_SHOULDER_S   = 3.0   # time to lower arm with shoulder actuators
EXCAVATION_WRIST_S      = 2.0   # time to level scoop with wrist
EXCAVATION_DRIVE_SPEED  = 0.15  # m/s forward to scoop regolith
EXCAVATION_DRIVE_S      = 5.0   # how long to drive into regolith
EXCAVATION_RAISE_S      = 3.0   # time to tilt scoop up + raise arm

DUMP_LIFT_DELAY_S       = 2.0   # wait after lift extends before reversing
DUMP_BACKUP_SPEED       = 0.10  # m/s reverse while spreading berm
DUMP_BACKUP_S           = 6.0   # how long to reverse
# ─────────────────────────────────────────────────────────────────────────────


class SequenceTester(Node):

    def __init__(self):
        super().__init__('test_linaks_sequence')

        svc_names = [
            'shoulder/extend', 'shoulder/retract',
            'wrist/extend',    'wrist/retract',
            'lift/extend',
            'stop_all',
        ]
        self._clients = {
            n: self.create_client(Trigger, f'/luna_linaks_node/{n}')
            for n in svc_names
        }
        self._cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

    # ── helpers ──────────────────────────────────────────────────────────────

    def _wait_ready(self, timeout=5.0):
        self.get_logger().info('Waiting for luna_linaks_node services…')
        deadline = time.time() + timeout
        for name, client in self._clients.items():
            remaining = max(0.1, deadline - time.time())
            if not client.wait_for_service(timeout_sec=remaining):
                self.get_logger().error(f'Service {name} not available — is luna_linaks_node running?')
                return False
        self.get_logger().info('All services ready.')
        return True

    def _call(self, name: str):
        client = self._clients[name]
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.result() is None:
            self.get_logger().warn(f'{name} call timed out')
        else:
            self.get_logger().info(f'{name}: {future.result().message}')

    def _drive(self, speed_ms: float, duration_s: float, label: str):
        self.get_logger().info(f'{label} at {speed_ms:+.2f} m/s for {duration_s}s')
        cmd = Twist()
        cmd.linear.x = float(speed_ms)
        end = time.time() + duration_s
        while time.time() < end:
            self._cmd_vel.publish(cmd)
            time.sleep(0.1)
        self._cmd_vel.publish(Twist())  # stop

    # ── sequences ────────────────────────────────────────────────────────────

    def run_excavation(self):
        self.get_logger().info('=== EXCAVATION SEQUENCE START ===')

        self.get_logger().info('Step 1: lowering arm (shoulder extend)')
        self._call('shoulder/extend')
        time.sleep(EXCAVATION_SHOULDER_S)

        self.get_logger().info('Step 2: leveling scoop (wrist extend)')
        self._call('wrist/extend')
        time.sleep(EXCAVATION_WRIST_S)

        self.get_logger().info('Step 3: driving into regolith')
        self._drive(EXCAVATION_DRIVE_SPEED, EXCAVATION_DRIVE_S, 'Forward')

        self.get_logger().info('Step 4: tilting scoop up (wrist retract)')
        self._call('wrist/retract')

        self.get_logger().info('Step 5: raising arm (shoulder retract)')
        self._call('shoulder/retract')
        time.sleep(EXCAVATION_RAISE_S)

        self.get_logger().info('=== EXCAVATION SEQUENCE DONE ===')

    def run_dump(self):
        self.get_logger().info('=== DUMP SEQUENCE START ===')

        self.get_logger().info('Step 1: extending scissor lift')
        self._call('lift/extend')
        time.sleep(DUMP_LIFT_DELAY_S)

        self.get_logger().info('Step 2: reversing to spread berm')
        self._drive(-abs(DUMP_BACKUP_SPEED), DUMP_BACKUP_S, 'Reverse')

        self.get_logger().info('=== DUMP SEQUENCE DONE (lift stays extended) ===')

    def stop_all(self):
        self.get_logger().info('Sending stop_all')
        self._call('stop_all')
        self._cmd_vel.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = SequenceTester()

    if not node._wait_ready():
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # Determine what to run from command-line args
    # Usage: python3 test_linaks_sequence.py [excavate|dump|both|stop]
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'both'

    try:
        if cmd == 'excavate':
            node.run_excavation()
        elif cmd == 'dump':
            node.run_dump()
        elif cmd == 'stop':
            node.stop_all()
        else:  # 'both' or anything else
            node.run_excavation()
            node.get_logger().info('Pausing 3s between sequences…')
            time.sleep(3.0)
            node.run_dump()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted — stopping all actuators')
        node.stop_all()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
