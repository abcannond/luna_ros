#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist


class LinaksJoyNode(Node):

    def __init__(self):
        super().__init__('linaks_joy')

        # --- position targets (mm) from CAD measurements ---
        self.declare_parameter('shoulder_dig_mm',   150.00)  # hardware max reach
        self.declare_parameter('shoulder_up_mm',      0.00)  # fully retracted
        self.declare_parameter('shoulder_dump_mm',   27.00)  # stage 9: arm position for dump
        self.declare_parameter('wrist_safe_mm',      130.00)  # fully extended (hardware max) — clears chassis during shoulder movement
        self.declare_parameter('wrist_dig_mm',       130.00)  # at max extension for scooping
        self.declare_parameter('wrist_up_mm',        110.10)  # tilted up to retain sand while raising arm
        self.declare_parameter('wrist_dump_mm',       30.70)  # fully retracted at top — dumps into collection bucket
        self.declare_parameter('lift_dump_mm',       100.0)  # scissor lift extended for dump
        self.declare_parameter('retract_dump_mm',        0) #scissor lift retracted 

        # --- position wait parameters ---
        self.declare_parameter('position_tolerance_mm', 3.0)
        self.declare_parameter('position_timeout_s',   15.0)

        # --- drive parameters ---
        self.declare_parameter('excavation_drive_speed', 0.15)
        self.declare_parameter('excavation_drive_s',     5.0)
        self.declare_parameter('dump_backup_speed',      0.10)
        self.declare_parameter('dump_backup_s',          6.0)

        # --- button / axis mapping ---
        self.declare_parameter('btn_excavate',         3)   # Y
        self.declare_parameter('btn_dump',             1)   # B
        self.declare_parameter('btn_stop_all',         2)   # X
        self.declare_parameter('btn_shoulder_retract', 4)   # LB — shoulder run in
        self.declare_parameter('btn_wrist_retract',    5)   # RB — wrist run in
        self.declare_parameter('axis_shoulder_extend', 2)   # LT — shoulder run out
        self.declare_parameter('axis_wrist_extend',    5)   # RT — wrist run out
        self.declare_parameter('axis_trigger_thresh',  0.25)
        self.declare_parameter('axis_trigger_rest',    1.0)   # F310 triggers rest at 1.0; Xbox at 0.0

        self._p = lambda n: self.get_parameter(n).value

        # --- Trigger service clients (manual joystick control) ---
        self._svc = {
            'shoulder/extend':  self.create_client(Trigger, '/luna_linaks_node/shoulder/extend'),
            'shoulder/retract': self.create_client(Trigger, '/luna_linaks_node/shoulder/retract'),
            'shoulder/stop':    self.create_client(Trigger, '/luna_linaks_node/shoulder/stop'),
            'wrist/extend':     self.create_client(Trigger, '/luna_linaks_node/wrist/extend'),
            'wrist/retract':    self.create_client(Trigger, '/luna_linaks_node/wrist/retract'),
            'wrist/stop':       self.create_client(Trigger, '/luna_linaks_node/wrist/stop'),
            'lift/extend':      self.create_client(Trigger, '/luna_linaks_node/lift/extend'),
            'stop_all':         self.create_client(Trigger, '/luna_linaks_node/stop_all'),
        }

        # --- position target publishers ---
        self._target_pubs = {
            'shoulder': self.create_publisher(Float32, '/luna_linaks_node/shoulder/target_mm', 10),
            'wrist':    self.create_publisher(Float32, '/luna_linaks_node/wrist/target_mm',    10),
            'lift':     self.create_publisher(Float32, '/luna_linaks_node/lift/target_mm',     10),
        }

        # --- position feedback ---
        self._positions = {'shoulder': 0.0, 'wrist': 0.0, 'lift': 0.0}
        self.create_subscription(Float32, '/luna_linaks_node/shoulder/position_mm',
                                 lambda m: self._positions.__setitem__('shoulder', m.data), 10)
        self.create_subscription(Float32, '/luna_linaks_node/wrist/position_mm',
                                 lambda m: self._positions.__setitem__('wrist', m.data), 10)
        self.create_subscription(Float32, '/luna_linaks_node/lift/position_mm',
                                 lambda m: self._positions.__setitem__('lift', m.data), 10)

        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._seq_thread = None
        self._seq_running = False
        self._prev_buttons = []

        self.create_subscription(Joy, '/joy', self._joy_cb, QoSProfile(depth=10))

        self.create_service(Trigger, '~/run_excavation', self._run_excavation_cb)
        self.create_service(Trigger, '~/run_dump',       self._run_dump_cb)
        self.create_service(Trigger, '~/stop',           self._stop_cb)

        self.get_logger().info(
            'linaks_joy ready\n'
            '  Y=excavate  B=dump  X=stop\n'
            '  LT=shoulder-out  LB=shoulder-in  RT=wrist-out  RB=wrist-in\n'
            '  services: ~/run_excavation  ~/run_dump  ~/stop')

    # ------------------------------------------------------------------
    # Service callbacks (terminal-triggered sequences)
    # ------------------------------------------------------------------

    def _run_excavation_cb(self, _, response):
        if self._seq_running:
            response.success = False
            response.message = 'sequence already running — call ~/stop first'
            return response
        self._start_sequence(self._excavation_sequence, 'excavation')
        response.success = True
        response.message = 'excavation sequence started'
        return response

    def _run_dump_cb(self, _, response):
        if self._seq_running:
            response.success = False
            response.message = 'sequence already running — call ~/stop first'
            return response
        self._start_sequence(self._dump_sequence, 'dump')
        response.success = True
        response.message = 'dump sequence started'
        return response

    def _stop_cb(self, _, response):
        self._abort_sequence()
        self._call('stop_all')
        response.success = True
        response.message = 'stopped'
        return response

    # ------------------------------------------------------------------
    # Joy callback
    # ------------------------------------------------------------------

    def _joy_cb(self, msg: Joy):
        btns = list(msg.buttons)
        axes = list(msg.axes)

        while len(self._prev_buttons) < len(btns):
            self._prev_buttons.append(0)

        def rising(idx):
            return (idx < len(btns) and btns[idx] and
                    (idx >= len(self._prev_buttons) or not self._prev_buttons[idx]))

        def held(idx):
            return idx < len(btns) and bool(btns[idx])

        def axis_held(idx, thresh):
            # measure displacement from resting value (F310 triggers rest at 1.0)
            rest = self._p('axis_trigger_rest')
            return idx < len(axes) and abs(axes[idx] - rest) > abs(thresh)

        if rising(self._p('btn_stop_all')):
            self._abort_sequence()
            self._call('stop_all')
            self.get_logger().info('STOP ALL')

        elif rising(self._p('btn_excavate')):
            self._start_sequence(self._excavation_sequence, 'excavation')

        elif rising(self._p('btn_dump')):
            self._start_sequence(self._dump_sequence, 'dump')

        else:
            if not self._seq_running:
                thresh = self._p('axis_trigger_thresh')

                # Shoulder: LT axis = extend (run out), LB button = retract (run in)
                if axis_held(self._p('axis_shoulder_extend'), thresh):
                    self._call('shoulder/extend')
                elif held(self._p('btn_shoulder_retract')):
                    self._call('shoulder/retract')
                else:
                    self._call('shoulder/stop')

                # Wrist: RT axis = extend (run out), RB button = retract (run in)
                if axis_held(self._p('axis_wrist_extend'), thresh):
                    self._call('wrist/extend')
                elif held(self._p('btn_wrist_retract')):
                    self._call('wrist/retract')
                else:
                    self._call('wrist/stop')

        self._prev_buttons = btns

    # ------------------------------------------------------------------
    # Sequence runner
    # ------------------------------------------------------------------

    def _start_sequence(self, fn, name: str):
        if self._seq_running:
            self.get_logger().warn('Sequence already running — press X to stop first')
            return
        self._abort_sequence()
        self._seq_running = True
        self._seq_thread = threading.Thread(target=self._run_sequence,
                                            args=(fn, name), daemon=True)
        self._seq_thread.start()

    def _abort_sequence(self):
        self._seq_running = False
        if self._seq_thread and self._seq_thread.is_alive():
            self._seq_thread.join(timeout=0.5)
        self._cmd_vel_pub.publish(Twist())

    def _run_sequence(self, fn, name: str):
        self.get_logger().info(f'--- {name} sequence START ---')
        try:
            fn()
        except Exception as e:
            self.get_logger().error(f'{name} sequence error: {e}')
        finally:
            self._seq_running = False
            self._cmd_vel_pub.publish(Twist())
            self.get_logger().info(f'--- {name} sequence END ---')

    def _move_to_multi(self, targets: dict):
        """Send multiple position targets simultaneously and wait until all are reached."""
        tol     = self._p('position_tolerance_mm')
        timeout = self._p('position_timeout_s')
        for actuator, target_mm in targets.items():
            msg = Float32()
            msg.data = float(target_mm)
            self._target_pubs[actuator].publish(msg)
            self.get_logger().info(f'  {actuator} → {target_mm:.1f} mm')

        deadline = time.time() + timeout
        while time.time() < deadline:
            if not self._seq_running:
                raise InterruptedError('sequence aborted')
            if all(abs(self._positions[a] - t) <= tol for a, t in targets.items()):
                return
            time.sleep(0.05)

        for actuator, target_mm in targets.items():
            self.get_logger().warn(
                f'  {actuator} position timeout (at {self._positions[actuator]:.1f} mm, '
                f'target {target_mm:.1f} mm)')

    def _move_to(self, actuator: str, target_mm: float):
        """Command actuator to target_mm and block until reached or timeout/abort."""
        tol     = self._p('position_tolerance_mm')
        timeout = self._p('position_timeout_s')
        msg = Float32()
        msg.data = float(target_mm)
        self._target_pubs[actuator].publish(msg)
        self.get_logger().info(f'  {actuator} → {target_mm:.1f} mm')

        deadline = time.time() + timeout
        while time.time() < deadline:
            if not self._seq_running:
                raise InterruptedError('sequence aborted')
            if abs(self._positions[actuator] - target_mm) <= tol:
                return
            time.sleep(0.05)

        self.get_logger().warn(
            f'  {actuator} position timeout (at {self._positions[actuator]:.1f} mm, '
            f'target {target_mm:.1f} mm)')

    def _drive(self, speed: float, duration_s: float):
        """Drive at speed m/s for duration_s seconds (interruptible)."""
        twist = Twist()
        twist.linear.x = float(speed)
        deadline = time.time() + duration_s
        while time.time() < deadline:
            if not self._seq_running:
                raise InterruptedError('sequence aborted')
            self._cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        self._cmd_vel_pub.publish(Twist())

    # ------------------------------------------------------------------
    # Excavation sequence
    # ------------------------------------------------------------------

    def _excavation_sequence(self):
        p = self._p
        '''
        # Extend wrist fully before moving shoulder — prevents scoop hitting chassis/door
        self.get_logger().info('Excavation: extending wrist to safe position')
        self._move_to('wrist', p('wrist_safe_mm'))

        self.get_logger().info('Excavation: lowering arm to dig position')
        self._move_to('shoulder', p('shoulder_dig_mm'))

        self.get_logger().info('Excavation: wrist to dig angle')
        self._move_to('wrist', p('wrist_up_mm'))

        self.get_logger().info('Excavation: driving forward to fill scoop')
        self._drive(p('excavation_drive_speed'), p('excavation_drive_s'))

        # Raise shoulder with wrist held back — keeps regolith in scoop
        self.get_logger().info('Excavation: raising arm')
        self._move_to('shoulder', p('shoulder_up_mm'))

        # Retract wrist to stage-9 position (matches shoulder_up_mm = 27mm)
        self.get_logger().info('Excavation: wrist to final raised position')
        self._move_to('wrist', p('wrist_dump_mm'))'''
        
        self.get_logger().info('Retracting')
        self._move_to('lift', p('retract_dump_mm'))

    # ------------------------------------------------------------------
    # Dump sequence
    # ------------------------------------------------------------------

    def _dump_sequence(self):
        p = self._p

        '''# Extend wrist fully before moving shoulder — prevents scoop hitting chassis/door
        self.get_logger().info('Dump: extending wrist to safe position')
        self._move_to('wrist', p('wrist_safe_mm'))

        self.get_logger().info('Dump: moving shoulder to dump position')
        self._move_to('shoulder', p('shoulder_dump_mm'))

        self.get_logger().info('Dump: moving wrist to dump position')
        self._move_to('wrist', p('wrist_dump_mm'))

        self.get_logger().info('Dump: extending scissor lift')
        self._move_to('lift', p('lift_dump_mm'))

        self.get_logger().info('Dump: reversing to spread berm')
        self._drive(-abs(p('dump_backup_speed')), p('dump_backup_s'))

        self.get_logger().info('Dump: complete — lift remains extended')'''
        self.get_logger().info('Dumping')
        self._move_to('lift', p('lift_dump_mm'))


    # ------------------------------------------------------------------
    # Service call helper (fire-and-forget for manual control)
    # ------------------------------------------------------------------

    def _call(self, name: str):
        client = self._svc.get(name)
        if client is None:
            return
        if not client.service_is_ready():
            self.get_logger().warn(f'linaks/{name} not ready (node running?)')
            return
        client.call_async(Trigger.Request())

    def _drec_drive(self, speed: float, duration_s: float):
        # drive for duration seconds at speed
        twist = Twist()
        twist.linear.x = speed
        deadline = time.time() + duration_s
        while time.time() < deadline:
            self._cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        #return; 


def main(autoMode: int):
    rclpy.init()
    node = LinaksJoyNode()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    def startup():
        time.sleep(1.0)
        if(autoMode == 1):
            node._drec_drive(0.5, 0.75)
            node._drec_drive(0.0, 0.5)
            node._start_sequence(node._dump_sequence, 'dump')
            node._seq_thread.join()
            '''
            # drive forward for 2 seconds
            twist = Twist()
            twist.linear.x = 0.25
            deadline = time.time() + 2.0
            while time.time() < deadline:
                node._cmd_vel_pub.publish(twist)
                time.sleep(0.1)

            # stop for 1 second
            node._cmd_vel_pub.publish(Twist())
            time.sleep(1.0)
            '''
            node._drec_drive(-0.5, 0.75)
            node._drec_drive(0.0, 0.5)

            node._start_sequence(node._excavation_sequence, 'excavation')
        elif(autoMode == 2):
            node._start_sequence(node._excavation_sequence, 'excavation')
            node._seq_thread.join()
        else:
            node._drec_drive(0,0)

    threading.Thread(target=startup, daemon=True).start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    import sys
    mode = 0
    if len(sys.argv) > 1:
        try:
            mode = int(sys.argv[1])
        except ValueError:
            mode = 0
    main(mode)