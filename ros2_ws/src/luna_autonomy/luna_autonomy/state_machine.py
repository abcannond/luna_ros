import signal
from enum import Enum
from types import SimpleNamespace

import rclpy
from rclpy.node import Node


class State(Enum):
    BOOT = "BOOT"
    # Excavation phase
    CALC_EXCAVATION_ZONE = "CALC_EXCAVATION_ZONE"
    NAV_TO_EXCAVATION = "NAV_TO_EXCAVATION"
    SAFETY_CHECK_EXCAVATION = "SAFETY_CHECK_EXCAVATION"
    EXCAVATE = "EXCAVATE"
    # Berm / dump phase
    CALC_BERM_ZONE = "CALC_BERM_ZONE"
    NAV_TO_BERM = "NAV_TO_BERM"
    SAFETY_CHECK_BERM = "SAFETY_CHECK_BERM"
    DUMP_SEQUENCE = "DUMP_SEQUENCE"
    # Done?
    SATISFIED_CHECK = "SATISFIED_CHECK"


class NavResult(Enum):
    SUCCESS = 0
    FAILED = 1


# -----------------------------------------------------------------------------
# State machine node: subscribe to topics, decide transitions only
# -----------------------------------------------------------------------------

class StateMachineNode(Node):
    """
    Subscribes to topics published by other packages. Stores last value per topic.
    Run loop only checks those values and transitions;
    """

    def __init__(self):
        super().__init__("state_machine")
        self._state = State.BOOT
        self._e_stop = False

        # What we heard from topics (other packages publish; we only listen).
        # Callbacks set e.g. self._in.boot_ready = msg.data
        self._in = SimpleNamespace(
            boot_ready=False,
            excavation_target=None,  # (x, y) or None
            berm_target=None,       # (x, y) or None
            nav_result=None,        # NavResult.SUCCESS / FAILED / None
            safe_to_excavate=False,
            safe_to_dump=False,
            excavation_done=False,
            dump_done=False,
            satisfied=False,
        )

        # TODO 1: Subscribe to topic; callback sets self._in.boot_ready = msg.data
        # TODO 2: Subscribe to excavation target; callback sets self._in.excavation_target = (x, y)
        # TODO 3: Subscribe to berm target; callback sets self._in.berm_target = (x, y)
        # TODO 4: Subscribe to nav result; callback sets self._in.nav_result = NavResult.SUCCESS or FAILED
        # TODO 5: Subscribe; callback sets self._in.safe_to_excavate = msg.data
        # TODO 6: Subscribe; callback sets self._in.safe_to_dump = msg.data
        # TODO 7: Subscribe; callback sets self._in.excavation_done = msg.data
        # TODO 8: Subscribe; callback sets self._in.dump_done = msg.data
        # TODO 9: Subscribe; callback sets self._in.satisfied = msg.data

        # Optional: publish nav goal so another node executes it (we only decide "go here").
        # TODO: If we publish /nav_goal, create publisher here; when entering NAV_TO_* we publish once.
        # self._pub_nav_goal = self.create_publisher(PoseStamped, '/nav_goal', 10)

        def _e_stop_handler(signum, frame):
            self._e_stop = True
            self.get_logger().warn("E-STOP requested (signal %s)" % signum)

        signal.signal(signal.SIGINT, _e_stop_handler)

    def run(self) -> None:
        while not self._e_stop:
            if self._state == State.BOOT:
                ########## TODO 1
                if self._in.boot_ready:
                    self._state = State.CALC_EXCAVATION_ZONE
                # else: keep waiting for topic

            elif self._state == State.CALC_EXCAVATION_ZONE:
                ########## TODO 2
                if self._in.excavation_target is not None:
                    # TODO: Publish self._in.excavation_target to /nav_goal if another node drives.
                    self._state = State.NAV_TO_EXCAVATION
                    self._in.nav_result = None  # wait for fresh result

            elif self._state == State.NAV_TO_EXCAVATION:
                ########## TODO 4
                if self._in.nav_result == NavResult.SUCCESS:
                    self._state = State.SAFETY_CHECK_EXCAVATION
                elif self._in.nav_result == NavResult.FAILED:
                    self.get_logger().warn("NAV_TO_EXCAVATION failed")
                    self._state = State.CALC_EXCAVATION_ZONE
                # else: keep waiting for nav result topic

            elif self._state == State.SAFETY_CHECK_EXCAVATION:
                ########## TODO 5
                if self._in.safe_to_excavate:
                    self._state = State.EXCAVATE
                    self._in.excavation_done = False
                # else: keep waiting or timeout and go back to CALC_EXCAVATION_ZONE (TBD)

            elif self._state == State.EXCAVATE:
                ########## TODO 7
                if self._in.excavation_done:
                    self._state = State.CALC_BERM_ZONE
                # else: other pkg is running excavation; we wait for topic

            elif self._state == State.CALC_BERM_ZONE:
                ########## TODO 3
                if self._in.berm_target is not None:
                    # TODO: Publish self._in.berm_target to /nav_goal if another node drives.
                    self._state = State.NAV_TO_BERM
                    self._in.nav_result = None

            elif self._state == State.NAV_TO_BERM:
                ########## TODO 4
                if self._in.nav_result == NavResult.SUCCESS:
                    self._state = State.SAFETY_CHECK_BERM
                elif self._in.nav_result == NavResult.FAILED:
                    self.get_logger().warn("NAV_TO_BERM failed")
                    self._state = State.CALC_EXCAVATION_ZONE
                # else: keep waiting

            elif self._state == State.SAFETY_CHECK_BERM:
                ########## TODO 6
                if self._in.safe_to_dump:
                    self._state = State.DUMP_SEQUENCE
                    self._in.dump_done = False
                # else: keep waiting or fallback (TBD)

            elif self._state == State.DUMP_SEQUENCE:
                ########## TODO 8
                if self._in.dump_done:
                    self._state = State.SATISFIED_CHECK
                # else: other pkg runs dump; we wait for topic

            elif self._state == State.SATISFIED_CHECK:
                ########## TODO 9
                if self._in.satisfied:
                    self.get_logger().info("Mission satisfied; done.")
                    return
                self._state = State.CALC_EXCAVATION_ZONE
                # Clear so we wait for new targets from topics
                self._in.excavation_target = None
                self._in.berm_target = None

            else:
                self.get_logger().error("Unknown state %s" % self._state)
                break

            rclpy.spin_once(self, timeout_sec=0.1)

        if self._e_stop:
            self.get_logger().warn("E-STOP; exiting.")


# -----------------------------------------------------------------------------
# Entry point
# -----------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
