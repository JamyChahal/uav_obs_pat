import math
import random

import numpy as np
import rclpy

from ..ActionManager import ActionManager
from ..Controller import Controller
from ..Pid import PID


def main(args=None):
    rclpy.init(args=args)

    print("ASK FOR EMERGENCY LAND", flush=True)

    # Wait for take off service to be ready
    action_manager = ActionManager()

    # ASK FOR LANDING
    action_manager.ask_for_landing()
    while rclpy.ok():
        rclpy.spin_once(action_manager)
        if action_manager.future.done():
            try:
                response = action_manager.future.result()
                if response.rc == 1:  # OK
                    print("Landing is a success !")
                    break  # Only if landing is ok
            except Exception as e:
                action_manager.get_logger().info("Service call failed %r" % (e,))
            break

    rclpy.shutdown()


if __name__ == '__main__':
    main()
