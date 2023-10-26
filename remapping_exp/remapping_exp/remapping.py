import copy
import math
import time

import cv2
import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from tello_msgs.srv import TelloAction

import numpy as np

from vicon_receiver.msg import Position

class ActionManager(Node):
    def __init__(self, drone_name):
        super().__init__('controller_manager')

        namespace = drone_name

        self.cli = self.create_client(TelloAction, namespace + '/tello_action')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TelloAction.Request()

    def ask_for_takeoff(self):
        self.req.cmd = "takeoff"
        self.future = self.cli.call_async(self.req)

    def ask_for_landing(self):
        self.req.cmd = "land"
        self.future = self.cli.call_async(self.req)


class Remapping(Node):

    def __init__(self):
        super().__init__('remap')

        ### CHECK FOR THE LIST OF AVAILABLE PEOPLE AROUND
        self.drones_pose_subscription_ = {}
        self.drones_pose_pub_ = {}
        self.targets_pose_pub_ = {}
        self.targets_pose_subscription_ = {}

        # Link between ID and name
        self.id2drone_dict = {'TelloS9' : 'drone0',
                              'TelloS10' : 'drone1',
                              'TelloS11' : 'drone2'
                              }

        self.id2target_dict = {'Target01' : 'target0',
                               'Target02' : 'target1',
                               'Target03' : 'target2'}

        # Create filter
        self.history_filter = {}
        for id in self.id2drone_dict:
            print(self.id2drone_dict[id])
            self.history_filter[self.id2drone_dict[id]] = {
                'x' : [0.] * 50,
                'y' : [0.] * 50,
                'z' : [0.] * 50
            }

        timer_period = 1  # seconds
        self.update_listener()  # Start at the beginning
        self.update_timer = self.create_timer(timer_period, self.update_listener)

    def id2drone(self, name):
        if name in self.id2drone_dict:
            return self.id2drone_dict[name]
        else:
            print("ERROR : No name like" + str(name))
            return None

    def id2target(self, name):
        if name in self.id2target_dict:
            return self.id2target_dict[name]
        else:
            print("ERROR : No name like" + str(name))
            return None

    def generate_lambda_drones_callback(self, drone_name):
        return lambda msg: self.drones_pose_callback(msg, drone_name)

    def generate_lambda_robots_callback(self, drone_name):
        return lambda msg: self.robots_pose_callback(msg, drone_name)


    def update_listener(self):
        # Issue from https://github.com/ros2/rclpy/issues/629
        topics = self.get_topic_names_and_types()
        for t in topics:
            sp = t[0].split('/')
            #if "vicon" in sp and "Tello" in sp:
            if "vicon" in sp:
                for sub in sp[1:]:
                    if "Tello" in sub:
                        drone_id = sp[2]
                        if not drone_id in self.drones_pose_subscription_:
                            drone_name = self.id2drone(drone_id)
                            if drone_name is not None:
                                self.drones_pose_subscription_[drone_id] = self.create_subscription \
                                    (Position,
                                     '/vicon/' + str(drone_id) + '/' + str(drone_id),
                                     self.generate_lambda_drones_callback(drone_name),
                                     1)
                                self.drones_pose_pub_[drone_name] = self.create_publisher(PoseStamped, drone_name + '/pose', 10)
                        break # Avoid continue loop

            #if "vicon" in sp and "Target" in sp:
            if "vicon" in sp:
                for sub in sp[1:]:
                    if "Target" in sub:
                        robot_id = sp[2]
                        if not robot_id in self.targets_pose_subscription_:
                            robot_name = self.id2target(robot_id)
                            if robot_name is not None:
                                self.targets_pose_subscription_[robot_id] = self.create_subscription \
                                    (Position,
                                     '/vicon/' + str(robot_id) + '/' + str(robot_id),
                                     self.generate_lambda_robots_callback(robot_name),
                                     1)
                                self.targets_pose_pub_[robot_name] = self.create_publisher(Odometry, robot_name + '/odom', 10)
                        break
    def drones_pose_callback(self, data, agent_name):
        if data.x_trans == 0 and data.y_trans == 0 and data.z_trans == 0:
            print(str(time.time()) + " [OUTSIDE ENVIRONMENT] drone : " + str(agent_name), flush=True)
            return

        x = data.x_trans / 1000
        y = data.y_trans / 1000
        z = data.z_trans / 1000

        # FILTERING DATA
        self.history_filter[agent_name]['x'].append(x)
        self.history_filter[agent_name]['x'].pop(0)

        self.history_filter[agent_name]['y'].append(y)
        self.history_filter[agent_name]['y'].pop(0)

        self.history_filter[agent_name]['z'].append(z)
        self.history_filter[agent_name]['z'].pop(0)

        x_filter = sum(self.history_filter[agent_name]['x']) / len(self.history_filter[agent_name]['x'])
        y_filter = sum(self.history_filter[agent_name]['y']) / len(self.history_filter[agent_name]['y'])
        z_filter = sum(self.history_filter[agent_name]['z']) / len(self.history_filter[agent_name]['z'])

        # Translation around the center
        #x_center = -0.5
        #y_center = -0.75

        x_center = -1.218
        y_center = -0.636

        x_filter -= x_center
        y_filter -= y_center

        p = PoseStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = x_filter
        p.pose.position.y = y_filter
        p.pose.position.z = z_filter
        p.pose.orientation.x = data.x_rot
        p.pose.orientation.y = data.y_rot
        p.pose.orientation.z = data.z_rot
        p.pose.orientation.w = data.w

        self.drones_pose_pub_[agent_name].publish(p)

    def robots_pose_callback(self, data, agent_name):
        if data.x_trans == 0 and data.y_trans == 0 and data.z_trans == 0:
            print(str(time.time()) + "[OUTSIDE ENVIRONMENT] robot : " + str(agent_name), flush=True)
            return

        x = data.x_trans / 1000
        y = data.y_trans / 1000
        z = data.z_trans / 1000

        # Translation around the center
        #x_center = -0.5
        #y_center = -0.75

        x_center = -1.218
        y_center = -0.636

        x -= x_center
        y -= y_center

        p = Odometry()
        p.pose.pose.position.x = x
        p.pose.pose.position.y = y
        p.pose.pose.position.z = z
        p.pose.pose.orientation.x = data.x_rot
        p.pose.pose.orientation.y = data.y_rot
        p.pose.pose.orientation.z = data.z_rot
        p.pose.pose.orientation.w = data.w

        self.targets_pose_pub_[agent_name].publish(p)

def main(args=None):
    rclpy.init(args=args)

    remap = Remapping()
    rclpy.spin(remap)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
