import math
import random

import numpy as np
import rclpy

from ..ActionManager import ActionManager
from ..Controller import Controller
from ..Pid import PID


class RandomBehavior(Controller):

    def __init__(self):
        super().__init__()

        print("I'm random running !!!", flush=True)

        #self.yaw_des = 0.
        self.x_des = 0.
        self.y_des = 0.

        self.pid_yaw = PID(0.5, 0.1, 0.)
        random.seed()
        self.new_desired_pose()

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

    def pose_callback(self, data):
        super().pose_callback(data)

        if self.distance_to_goal() < self.radius_to_goal:
            self.new_desired_pose()

        self.update_angle()

        yaw_des_temp = self.yaw_des

        if abs(yaw_des_temp - self.yaw) > math.pi:
            if yaw_des_temp > self.yaw:
                yaw_des_temp -= 2 * math.pi
            else:
                yaw_des_temp += 2 * math.pi


        # self.v_yaw = self.pid_yaw.loop(self.yaw_des - self.yaw)
        # self.vx = self.max_speed
        Fx, Fy = self.get_force_collision_avoidance()

        if self.fast_rviz:
            self.vx = Fx + self.max_speed
            self.vy = Fy
            self.v_yaw = self.pid_yaw.loop(yaw_des_temp - self.yaw)
        else:
            self.vx = Fx + self.max_speed * math.cos(self.yaw_des)
            self.vy = Fy + self.max_speed * math.sin(self.yaw_des)
            self.v_yaw = self.pid_yaw.loop(-1*(0.-self.yaw))
            self.vz = self.pid_z.loop(self.des_z - self.z)

        # DEBUG
        '''
        print("DRONE : " + str(self.namespace), flush=True)
        print("DESIRED POSE", flush=True)
        print(self.x_des, self.y_des, flush=True)
        print("ACTUAL POSE", flush=True)
        print(self.x, self.y, flush=True)
        '''

        if self.fast_rviz:
            self.send_timestep_ok()

    def new_desired_pose(self):
        self.x_des = random.uniform(-self.map_size_x, self.map_size_x)
        self.y_des = random.uniform(-self.map_size_y, self.map_size_y)
        self.update_angle()


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
        controller = RandomBehavior()
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
