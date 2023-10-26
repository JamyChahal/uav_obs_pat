import copy
import math
import os
import random
from operator import itemgetter

import rclpy

import numpy as np

from ..ActionManager import ActionManager
from ..Controller import Controller
from ..Pid import PID

from ray.rllib.policy.policy import Policy

MAX_SEEN_TARGET = 8
MAX_SEEN_AGENT = 8


class FFRLBehavior(Controller):

    def __init__(self):
        super().__init__()

        self.declare_parameter('ffrl_model', "col")
        self.model = self.get_parameter('ffrl_model').get_parameter_value().string_value

        print("MODEL NAME :")
        print(self.model)

        self.policy = self.get_policy()

        self.yaw_des = 0.
        self.x_des = 0.
        self.y_des = 0.

        self.pid_yaw = PID(1., 0.3, 0.)
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

    def compute_strategy(self):
        walk_random = True
        # Prepare here the observation
        obs = [self.x, self.y]
        # Update friends relative pose
        friend_list = []
        for friend_name in self.friends_pose:
            pose = self.friends_pose[friend_name]
            friend_list.append((pose.x - self.x, pose.y - self.y, self.get_distance(pose)))

        friend_list = sorted(friend_list, key=itemgetter(2))

        for i in range(0, MAX_SEEN_AGENT):
            if i < len(friend_list):
                obs.append(friend_list[i][0])  # X
                obs.append(friend_list[i][1])  # Y
            else:
                obs.append(0)
                obs.append(0)

        # Update targets relative pose
        target_list = []
        for target_name in self.targets_pose:
            walk_random = False
            pose = self.targets_pose[target_name]
            target_list.append((pose.x - self.x, pose.y - self.y, self.get_distance(pose)))

        target_list = sorted(target_list, key=itemgetter(2))

        for i in range(0, MAX_SEEN_TARGET):
            if i < len(target_list):
                obs.append(target_list[i][0])  # X
                obs.append(target_list[i][1])  # Y
            else:
                obs.append(0)
                obs.append(0)

        obs = np.reshape(obs, (MAX_SEEN_TARGET * 2 + MAX_SEEN_AGENT * 2 + 2))

        if walk_random:
            if self.distance_to_goal() < self.radius_to_goal:
                self.new_desired_pose()
            self.update_angle()
            self.vx = self.max_speed * math.cos(self.yaw_des)
            self.vy = self.max_speed * math.sin(self.yaw_des)
        else:
            x, y = self.policy.compute_single_action(obs)[0]
            self.vx, self.vy = self.get_continuous_action(x, y)

    def new_desired_pose(self):
        self.x_des = random.uniform(-self.map_size_x, self.map_size_x)
        self.y_des = random.uniform(-self.map_size_y, self.map_size_y)
        self.update_angle()

    def get_continuous_action(self, x_speed_choice, y_speed_choice):
        if x_speed_choice == 0:
            x_speed = 1
        elif x_speed_choice == 1:
            x_speed = 0.5
        elif x_speed_choice == 2:
            x_speed = 0
        elif x_speed_choice == 3:
            x_speed = -0.5
        elif x_speed_choice == 4:
            x_speed = -1
        else:
            x_speed = 0

        if y_speed_choice == 0:
            y_speed = 1
        elif y_speed_choice == 1:
            y_speed = 0.5
        elif y_speed_choice == 2:
            y_speed = 0
        elif y_speed_choice == 3:
            y_speed = -0.5
        elif y_speed_choice == 4:
            y_speed = -1
        else:
            y_speed = 0

        return x_speed, y_speed

    def get_policy(self):
        path = os.path.expanduser("~") + '/uav_obs_pat_ws/src/tello_obs_pat/tello_strategy/tello_strategy/FFRL'
        if self.model == 'col':
            path += '/FFRL_COL'
            print("LOAD MODEL 128 128 COL")
        elif self.model == 'ind':
            path += '/FFRL_IND'
            print("LOAD MODEL 128 128 IND")
        else:
            print("ERROR IN THE MODEL !")

        print("PATH : " + str(path))

        policy = Policy.from_checkpoint(path, ['default_policy'])
        return policy['default_policy']

    def pose_callback(self, data):
        super().pose_callback(data)

        self.compute_strategy()
        self.yaw_des = 0.

        Fx, Fy = self.get_force_collision_avoidance()

        self.vx = (self.vx + Fx) * self.max_speed
        self.vy = (self.vy + Fy) * self.max_speed
        self.v_yaw = self.pid_yaw.loop(-1 * (0. - self.yaw))

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
        controller = FFRLBehavior()
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
