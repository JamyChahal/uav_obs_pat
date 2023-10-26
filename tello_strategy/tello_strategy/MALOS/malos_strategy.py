import copy
import math
import random

import rclpy

import numpy as np
from matplotlib import pyplot as plt

from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Int64MultiArray

import tensorflow as tf
import os

from ..ActionManager import ActionManager
from ..Controller import Controller
from ..Pid import PID
from .Map_Obs import Map_Env
from . import ros_np_multiarray as rnm


# import ros_np_multiarray as rnm


class MALOSBehavior(Controller):

    def __init__(self):
        super().__init__()

        self.yaw_des = 0.
        self.x_des = 0.
        self.y_des = 0.

        self.my_maps = Map_Env(map_size_x=self.map_size_x, map_size_y=self.map_size_y,
                               obs_range=int(self.obs_range), discretization=self.map_discrete)

        self.pid_yaw = PID(1., 0.3, 0.)
        random.seed()
        self.new_desired_pose()

        # Publisher for map's
        self.map_idleness_publisher_ = self.create_publisher(Int64MultiArray, self.namespace + "/map_idleness", 10)
        # Listener for map's
        self.friends_map_idleness_subscription_ = {}
        # For display debug
        self.fig, self.axes = plt.subplots(nrows=2, ncols=2)

        self.timer_publish_map = self.create_timer(1, self.publish_map)
        self.timer_update_listener_map = self.create_timer(1, self.update_listener_map_idleness)

        self.model = self.get_model()

    def update_listener_map_idleness(self):
        # Delete the listener that we are not communicate anymore

        # Avoid thread conflict, when friends_pose is updated during execution
        friends_pose = copy.copy(self.friends_pose)
        agent_to_be_removed = []
        for agent_name_callback in self.friends_map_idleness_subscription_:
            if agent_name_callback not in friends_pose:
                agent_to_be_removed.append(agent_name_callback)

        for a in agent_to_be_removed:
            self.friends_map_idleness_subscription_.pop(a)

        for agent_name in friends_pose:
            if agent_name not in self.friends_map_idleness_subscription_:
                drone_name = copy.copy(agent_name)
                self.friends_map_idleness_subscription_[agent_name] = self.create_subscription \
                    (Int64MultiArray,
                     drone_name + '/map_idleness',
                     self.map_idleness_callback,
                     1)

    def map_idleness_callback(self, data):
        received_map = rnm.to_numpy_i64(data)
        self.my_maps.update_from_map(received_map)

    def publish_map(self):
        msg = rnm.to_multiarray_i64(self.my_maps.get_idleness_map().astype(np.int64))
        self.map_idleness_publisher_.publish(msg)

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

    def friends_pose_callback(self, data, agent_name):
        super().friends_pose_callback(data, agent_name)
        for friend_name in self.friends_pose:
            pose = self.friends_pose[friend_name]
            self.my_maps.add_agent_friend(pose.x, pose.y)

    def targets_pose_callback(self, data, target_name):
        super().targets_pose_callback(data, target_name)
        for target_name in self.targets_pose:
            pose = self.targets_pose[target_name]
            self.my_maps.add_target(pose.x, pose.y)

    def update_pose_mymaps(self):
        self.my_maps.add_agent_moi(self.x, self.y)

        for friend_name in self.friends_pose:
            pose = self.friends_pose[friend_name]
            self.my_maps.add_agent_friend(pose.x, pose.y)

        for target_name in self.targets_pose:
            pose = self.targets_pose[target_name]
            self.my_maps.add_target(pose.x, pose.y)

        self.my_maps.observe(self.x, self.y, update_idleness=True)

    def get_model(self):
        # https://github.com/ray-project/ray/blob/master/rllib/tests/test_export.py

        model_dir = os.path.join(
            os.path.expanduser("~") + "/uav_obs_pat_ws/src/tello_obs_pat/tello_strategy/tello_strategy/MALOS/model_MALOS")

        model = tf.saved_model.load(model_dir)
        sign = model.signatures['serving_default']
        return sign

    def call_model(self):
        maps = self.my_maps.get_map(self.x, self.y)

        yhat = self.model(input_e=np.reshape(maps[:, :, 0], [1, 30, 30]),
                          input_t=np.reshape(maps[:, :, 1], [1, 30, 30]),
                          input_a=np.reshape(maps[:, :, 2], [1, 30, 30]),
                          input_i=np.reshape(maps[:, :, 3], [1, 30, 30]))

        action_x = yhat['output_x'].numpy()
        action_y = yhat['output_y'].numpy()

        self.x_des = self.x + action_x
        self.y_des = self.y + action_y

        # Add a security
        self.x_des = self.x_des if self.x_des <= self.map_size_x else self.map_size_x
        self.x_des = self.x_des if self.x_des >= -self.map_size_x else -self.map_size_x
        self.y_des = self.y_des if self.y_des <= self.map_size_y else self.map_size_y
        self.y_des = self.y_des if self.y_des >= -self.map_size_y else -self.map_size_y

    def pose_callback(self, data):
        super().pose_callback(data)

        # Do not forget, this is called at 10Hz here
        self.my_maps.next_time()
        self.update_pose_mymaps()

        # Here we prepare the data for the model, and listen to the new desired pose
        self.call_model()

        self.update_angle()

        if abs(self.yaw_des - self.yaw) > math.pi:
            if self.yaw_des > self.yaw:
                self.yaw_des -= 2 * math.pi
            else:
                self.yaw_des += 2 * math.pi

        Fx, Fy = self.get_force_collision_avoidance()

        if self.fast_rviz:
            desired_speed = math.sqrt(math.pow(self.x - self.x_des, 2) + math.pow(self.y - self.y_des, 2))
            self.vx = Fx + self.max_speed * desired_speed
            self.vy = Fy
            self.v_yaw = self.pid_yaw.loop(self.yaw_des - self.yaw)
        else:
            self.vx = Fx + self.max_speed * math.cos(self.yaw_des)
            self.vy = Fy + self.max_speed * math.sin(self.yaw_des)
            self.v_yaw = self.pid_yaw.loop(-1*(0.-self.yaw))

        if self.fast_rviz:
            self.send_timestep_ok()

    def new_desired_pose(self):
        self.x_des = random.uniform(-self.map_size_x, self.map_size_x)
        self.y_des = random.uniform(-self.map_size_y, self.map_size_y)
        self.update_angle()

    def display_all_maps(self):
        m = self.my_maps.get_map(self.x, self.y)
        # fig, axes = plt.subplots(nrows=2, ncols=2)
        fig, axes = self.fig, self.axes
        i = 0
        for ax in axes.flat:
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

        fig.subplots_adjust(right=0.8)
        # fig.tight_layout()
        cbar_ax = fig.add_axes([0.85, 0.15, 0.05, 0.7])
        fig.colorbar(im, cax=cbar_ax)

        plt.subplots_adjust(bottom=0.05, top=0.95)

        plt.show(block=False)
        plt.pause(0.001)


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
        controller = MALOSBehavior()
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
