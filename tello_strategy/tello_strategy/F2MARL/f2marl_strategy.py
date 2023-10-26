import platform

if platform.node() == 'YOUR_MACHINE_NAME':
    # USED ONLY TO DEBUG AND SHOW IMAGE
    use_matplotlib = True
else:
    use_matplotlib = False

import copy
import math
import os
import random

import rclpy

if use_matplotlib:
    from matplotlib import pyplot as plt

from std_msgs.msg import Int64MultiArray, Float32MultiArray

from .cnn_local_global import CNN_LOCAL_GLOBAL
from . import ros_np_multiarray as rnm

import numpy as np

from .Map_Obs import Map_Env
from ..ActionManager import ActionManager
from ..Controller import Controller, TrackPose
from ..Pid import PID

from ray.rllib.policy.policy import Policy
from ray.rllib.models import ModelCatalog

MAX_SEEN_TARGET = 8
MAX_SEEN_AGENT = 8


class F2MARLBehavior(Controller):

    def __init__(self):
        super().__init__()

        if use_matplotlib:
            self.fig, self.axes = plt.subplots(nrows=2, ncols=2)
        self.first_fig = True

        self.targets_pose_shared = []

        self.policy = self.get_policy()

        self.my_maps = Map_Env(map_size=self.map_size_x, obs_range=int(self.obs_range), com_range=int(self.com_range),
                               discretization=0.3)

        self.yaw_des = 0.
        self.x_des = 0.
        self.y_des = 0.

        self.target_from_share = {}

        self.pid_yaw = PID(1., 0.3, 0.)
        random.seed()
        self.new_desired_pose()

        # Publisher for map's
        self.map_idleness_publisher_ = self.create_publisher(Int64MultiArray, self.namespace + "/map_idleness", 10)
        # Listener for map's
        self.friends_map_idleness_subscription_ = {}

        # Publisher for target's share
        self.target_publisher_ = self.create_publisher(Float32MultiArray, self.namespace + "/share_target", 10)
        # Listener for target's
        self.target_publisher_subscription_ = {}

        self.timer_publish_map = self.create_timer(1, self.publish_map)
        self.timer_publish_targets = self.create_timer(1, self.publish_targets)
        self.timer_update_listener_map = self.create_timer(1, self.update_listener_map_idleness)
        self.timer_update_listener_map = self.create_timer(1, self.update_listener_targets_share)

    def display_all_maps(self):
        if not use_matplotlib:
            return
        m = self.my_maps.get_map(self.x, self.y)
        i = 0
        if not self.first_fig and not plt.get_fignums():
            return
        for ax in self.axes.flat:
            im = ax.imshow(np.flip(np.rot90(m[:, :, i], -1), axis=1), vmin=0, vmax=1, origin="lower")
            if i == 0:
                ax.set_title("Environment's map", pad=5)
            if i == 1:
                ax.set_title("Target's map", pad=5)
            if i == 2:
                ax.set_title("Agent's map", pad=5)
            if i == 3:
                ax.set_title("Idleness's map", pad=5)
            i = i + 1

        self.fig.subplots_adjust(right=0.8)
        # fig.tight_layout()
        cbar_ax = self.fig.add_axes([0.85, 0.15, 0.05, 0.7])
        self.fig.colorbar(im, cax=cbar_ax)

        plt.subplots_adjust(bottom=0.05, top=0.95)

        self.first_fig = False
        plt.show(block=False)
        plt.pause(0.000001)

    def publish_targets(self):
        msg = Float32MultiArray()
        msg_list = []

        for target_name in self.targets_pose:
            pose = self.targets_pose[target_name]
            msg_list.append(float(str.replace(target_name, 'target', '')))
            msg_list.append(pose.x)
            msg_list.append(pose.y)

        msg.data = msg_list
        self.target_publisher_.publish(msg)

    def share_target_callback(self, data):
        # Receive target detection from other drone in com'
        targets_list = data.data
        for i in range(0, len(targets_list), 3):
            name = 'target' + str(int(targets_list[i]))
            if not name in self.targets_pose:
                t = TrackPose()
                t.x = targets_list[i + 1]
                t.y = targets_list[i + 2]
                self.targets_pose[name] = t

    def update_listener_targets_share(self):
        # Avoid thread conflict, when friends_pose is updated during execution
        friends_pose = copy.copy(self.friends_pose)
        agent_to_be_removed = []

        for agent_name_callback in self.target_publisher_subscription_:
            if agent_name_callback not in friends_pose:
                agent_to_be_removed.append(agent_name_callback)

        for a in agent_to_be_removed:
            self.destroy_subscription(self.target_publisher_subscription_[a])
            self.target_publisher_subscription_.pop(a)

        for agent_name in friends_pose:
            if agent_name not in self.target_publisher_subscription_:
                drone_name = copy.copy(agent_name)
                self.target_publisher_subscription_[agent_name] = self.create_subscription \
                    (Float32MultiArray,
                     drone_name + '/share_target',
                     self.share_target_callback,
                     1)

    def map_idleness_callback(self, data):
        received_map = rnm.to_numpy_i64(data)
        self.my_maps.update_from_map(received_map)

    def publish_map(self):
        msg = rnm.to_multiarray_i64(self.my_maps.get_idleness_map().astype(np.int64))
        self.map_idleness_publisher_.publish(msg)

    def update_listener_map_idleness(self):
        # Avoid thread conflict, when friends_pose is updated during execution
        friends_pose = copy.copy(self.friends_pose)
        agent_to_be_removed = []

        # MAP
        for agent_name_callback in self.friends_map_idleness_subscription_:
            if agent_name_callback not in friends_pose:
                agent_to_be_removed.append(agent_name_callback)

        for a in agent_to_be_removed:
            self.destroy_subscription(self.friends_map_idleness_subscription_[a])
            self.friends_map_idleness_subscription_.pop(a)

        for agent_name in friends_pose:
            if agent_name not in self.friends_map_idleness_subscription_:
                drone_name = copy.copy(agent_name)
                self.friends_map_idleness_subscription_[agent_name] = self.create_subscription \
                    (Int64MultiArray,
                     drone_name + '/map_idleness',
                     self.map_idleness_callback,
                     1)

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
        # Prepare empty tensor to fit the model
        nbr_poses = 2 + 8 * 2 + 8 * 2
        arr_poses = np.zeros((nbr_poses))
        # 30x30 empty tensor for the critic model
        arr_image_critic = np.zeros((30 * 30 * 4))

        self.my_maps.next_time()

        self.my_maps.observe(self.x, self.y, update_idleness=True)
        self.my_maps.add_agent_moi(self.x, self.y)
        for friend_name in self.friends_pose:
            pose = self.friends_pose[friend_name]
            self.my_maps.add_agent_friend(pose.x, pose.y)

        for target_name in self.targets_pose:
            pose = self.targets_pose[target_name]
            self.my_maps.add_target(pose.x, pose.y)

        arr_image_actor = self.my_maps.get_map(self.x, self.y).flatten()

        obs = np.concatenate((arr_poses, arr_image_actor, arr_image_critic))

        x, y = self.policy.compute_single_action(obs)[0]
        self.vx, self.vy = self.get_continuous_action(x, y)

        if self.namespace == 'drone0' and use_matplotlib:
            self.display_all_maps()

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
        ModelCatalog.register_custom_model("CNN_MODEL", CNN_LOCAL_GLOBAL)
        path = os.path.expanduser("~") + '/uav_obs_pat_ws/src/tello_obs_pat/tello_strategy/tello_strategy/F2MARL'
        path += '/checkpoint_local_global'  # TODO : Change with your own checkpoint

        policy = Policy.from_checkpoint(path)

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
        controller = F2MARLBehavior()
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
