import math
import random

import numpy as np
import rclpy

from .Map import Map
from ..ActionManager import ActionManager
from ..Controller import Controller
from ..Pid import PID


class HI(Controller):

    def __init__(self):
        super().__init__()

        self.ID = int(self.namespace.replace('drone', ''))

        self.x_des, self.y_des = 0., 0.  # Init, no importance about the first value because updated continuously
        self.x_motion, self.y_motion = 0., 0.
        self.x_des_pat, self.y_des_pat = 0., 0.
        self.yaw_des = 0.
        self.speed_norm = 1
        self.pid_yaw = PID(1., 0., 0.)
        self.discretization = 1

        # Perform mask
        self.kernel_size = int(self.obs_range * self.discretization * 2)
        self.kernel = np.ones(self.kernel_size) * 1 / self.kernel_size

        # For patrol
        self.index_x_des, self.index_y_des = self.random_index()  # Init, no importance about the first value

        self.myMap = Map(map_size_x=self.map_size_x, map_size_y=self.map_size_y,
                         obs_range=int(self.obs_range), discretization=self.map_discrete)

    def random_index(self):
        index_x_des = random.randint(0, (self.map_size_x - 1) * 2 * self.discretization)
        index_y_des = random.randint(0, (self.map_size_y - 1) * 2 * self.discretization)
        return index_x_des, index_y_des

    def distance_to_goal(self):
        return math.sqrt(math.pow(self.x - self.x_des, 2) + math.pow(self.y - self.y_des, 2))

    def get_pipi_angle(self, angle):
        # Check if angle is between -pi and pi
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def update_angle(self):
        self.yaw_des = math.atan2(self.y_des - self.y, self.x_des - self.x)
        self.yaw_des = self.get_pipi_angle(self.yaw_des)

    def get_random_point(self):
        safety = 1
        return random.uniform(-self.map_size_x + safety, self.map_size_x - safety), \
            random.uniform(-self.map_size_y + safety, self.map_size_y - safety)

    def pose_callback(self, data):
        super().pose_callback(data)

        self.myMap.new_time()
        self.myMap.observe(self.x, self.y, update_idleness=True)

        self.desired_high_idleness()
        self.update_angle()

        if abs(self.yaw_des - self.yaw) > math.pi:
            if self.yaw_des > self.yaw:
                self.yaw_des -= 2 * math.pi
            else:
                self.yaw_des += 2 * math.pi

        # self.v_yaw = self.pid_yaw.loop(self.yaw_des - self.yaw)
        # self.vx = self.max_speed
        Fx, Fy = self.get_force_collision_avoidance()

        if self.fast_rviz:
            self.vx = Fx + self.max_speed
            self.vy = Fy
            self.v_yaw = self.pid_yaw.loop(self.yaw_des - self.yaw)
        else:
            self.vx = Fx + self.max_speed * math.cos(self.yaw_des)
            self.vy = Fy + self.max_speed * math.sin(self.yaw_des)
            self.v_yaw = self.pid_yaw.loop(0.-self.yaw)


        if self.fast_rviz:
            self.send_timestep_ok()

    def desired_high_idleness(self):
        #c = cv2.filter2D(self.myMap.get_map(), -1, self.kernel)
        c = self.myMap.get_map()

        cmax_old = int(c[self.index_x_des, self.index_y_des])
        cmax = int(c.max())
        if cmax_old < cmax:  # Another better area to check seen

            cmax = c.max()
            index = np.where(c == cmax)
            if len(index[0]) > 1:
                r = random.randint(0, len(index[0]) - 1)
                self.index_x_des, self.index_y_des = index[0][r], index[1][r]
            elif len(index[0] == 1):
                self.index_x_des, self.index_y_des = index[0][0], index[1][0]
            else:
                print("PROBLEM ABOUT INDEX IN CMAX")

        self.x_des = (self.index_x_des / self.discretization) - self.map_size_x
        self.y_des = (self.index_y_des / self.discretization) - self.map_size_y  # Recadrage autour de (0,0)

def main(args=None):
    rclpy.init(args=args)

    # Wait for take off service to be ready
    action_manager = ActionManager()
    action_manager.ask_for_takeoff()
    ready_to_continue_mission = False

    # Try to takeoff, wait for the return of the service
    while rclpy.ok():
        rclpy.spin_once(action_manager)
        if action_manager.future.done():
            try:
                response = action_manager.future.result()
                if response.rc == 1:  # OK
                    print("Take off is a success !")
                    ready_to_continue_mission = True
                else:  # NOT OK
                    print("Something is wrong with the takeoff. LANDING NOW")
            except Exception as e:
                action_manager.get_logger().info("Service call failed %r" % (e,))
            break

    if ready_to_continue_mission:
        controller = HI()
        try:
            while rclpy.ok():
                rclpy.spin_once(controller)
        except KeyboardInterrupt:
            print("Stopping the control. Ask for landing.")
        controller.destroy_node()

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
