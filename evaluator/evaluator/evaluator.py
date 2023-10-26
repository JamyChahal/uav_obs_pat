import copy
import math
import os
import time

import cv2
import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

from datetime import datetime

from tello_rviz_msg.srv import TimeStepOK
import numpy as np
from .Map import Map


class TrackPose:

    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.time = 0


class Evaluator(Node):

    def __init__(self):
        super().__init__('evaluator')

        self.full_result_is_saved = False

        self.declare_parameter("log_dir", "result")
        self.log_dir = self.get_parameter("log_dir").get_parameter_value().string_value
        # File path
        self.directory = os.path.dirname(os.path.realpath(__file__)) + "/" + str(self.log_dir) + "/"
        self.create_dirs()
        # Continous backup variable
        self.last_time_backup = 0
        now = datetime.now()

        # Variables for other functions
        self.time = 0
        self.nbr_cells = 0

        self.declare_parameter("fast_rviz", False)
        self.fast_rviz = self.get_parameter('fast_rviz').value

        if self.fast_rviz:
            # Execute service
            self.cli = self.create_client(TimeStepOK, 'timestep')
            while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Wait for timestep service')
            self.service_req = TimeStepOK.Request()
            self.send_timestep_ok()

        self.declare_parameter("map_size_x", 10.)
        self.declare_parameter("map_size_y", 10.)
        self.map_size_x = self.get_parameter("map_size_x").get_parameter_value().double_value
        self.map_size_y = self.get_parameter("map_size_y").get_parameter_value().double_value

        self.declare_parameter("obs_range", 10.)
        self.obs_range = self.get_parameter("obs_range").get_parameter_value().double_value

        self.declare_parameter("com_range", 10.)
        self.com_range = self.get_parameter("com_range").get_parameter_value().double_value

        self.declare_parameter('agent_speed', 1.)
        self.agent_speed = self.get_parameter("agent_speed").get_parameter_value().double_value

        self.declare_parameter('target_speed', 0.5)
        self.target_speed = self.get_parameter("target_speed").get_parameter_value().double_value

        self.declare_parameter("map_discrete", 10)
        self.map_discrete = self.get_parameter("map_discrete").get_parameter_value().integer_value

        self.declare_parameter("mission_duration", 30)
        self.mission_duration = self.get_parameter("mission_duration").get_parameter_value().integer_value

        self.declare_parameter("nbr_drone", 3)
        self.nbr_drone = self.get_parameter("nbr_drone").get_parameter_value().integer_value

        self.declare_parameter("nbr_target", 3)
        self.nbr_target = self.get_parameter("nbr_target").get_parameter_value().integer_value

        self.declare_parameter("strategy", "run_random")
        self.strategy = self.get_parameter("strategy").get_parameter_value().string_value

        self.declare_parameter('ffrl_model', "128x128_col")
        self.model = self.get_parameter('ffrl_model').get_parameter_value().string_value

        self.declare_parameter('sigma', 0.2 * 60 * 30)
        self.sigma = self.get_parameter('sigma').value
        self.sigma_perc = round(float(self.sigma / self.mission_duration), 1)

        self.strategy_name = "UNKNOWN"

        if self.strategy == "run_random":
            self.strategy_name = "RANDOM"
        elif self.strategy == "run_a_cmommt":
            self.strategy_name = "A-CMOMMT"
        elif self.strategy == "run_malos":
            self.strategy_name = "MALOS"
        elif self.strategy == "run_i_cmommt":
            self.strategy_name = "I-CMOMMT-" + str(self.sigma_perc)
        elif self.strategy == 'run_hi':
            self.strategy_name = "HI"
        elif self.strategy == 'run_ci':
            self.strategy_name = "CI"
        elif self.strategy == 'run_ffrl':
            if "col" in self.model:
                self.strategy_name = "FFRL_SHARE"
            else:
                self.strategy_name = "FFRL_IND"
        elif self.strategy == 'run_f2marl':
            self.strategy_name = 'F2MARL'
        else:
            self.strategy_name = "UNKNOWN"

        self.continuous_backup_filename = str(self.strategy_name) + "_" + now.strftime("%d_%m_%Y_%H_%M_%S_%f") + str(".txt")

        self.obs_i = {}  # Keep the observation of each target, to get the observation diversity
        for j in range(0, self.nbr_target):
            self.obs_i['target'+str(j)] = 0
        self.counter_obs = 0  # Count the number of target seen during the full experience time
        self.counter_loop = 0

        ### CHECK FOR THE LIST OF AVAILABLE PEOPLE AROUND
        self.drones_pose_subscription_ = {}
        self.targets_pose_subscription_ = {}

        # Pose of drones and targets
        self.drones_pose = {}
        self.targets_pose = {}

        # Header for logs
        self.logs_header = ["method", "nbr_agent", "nbr_target", "obs_range", "com_range", "duration",
                            "map_size_x", "map_size_y", "agent_speed", "target_speed", "target_behavior",
                            "A_metric", "H_metric", "sigma_n", "I_oav_m", "max_idleness", "max_idleness_region", "MI", "MSI"]

        self.summary_logs_header = ["filename", "method", "nbr_agent", "nbr_target", "obs_range", "com_range", "map_size_x",
                                    "map_size_y"]

        self.continuous_logs_header = ["time", "A_metric", "H_metric", "sigma_n", "I_oav_m", "I_inst", "max_idleness",
                                       "max_idleness_region", "MI", "MSI"]

        self.map = Map(map_size_x=self.map_size_x, map_size_y=self.map_size_y, obs_range=self.obs_range,
                       discretization=self.map_discrete)
        self.max_idleness = 0
        self.max_idleness_blur = 0
        self.average_idleness_sum = 0

        self.map_size_x_discrete = int(self.map_size_x * self.map_discrete * 2)
        self.map_size_y_discrete = int(self.map_size_y * self.map_discrete * 2)

        self.intervals = {}
        self.previous_idleness = {}  # To keep in memory the previous idleness, when cell is visited
        for i in range(0, self.map_size_x_discrete):
            for j in range(0, self.map_size_y_discrete):
                self.intervals[i, j] = []
                self.previous_idleness[i, j] = 0

        # For saved data
        self.is_data_saved = False

        # Listener
        self.time = 0
        self.clock_subscription_ = self.create_subscription(
            Clock, "/clock", self.clock_callback, 10
        )

        timer_period = 1  # seconds
        self.update_listener()  # Start at the beginning
        self.update_timer = self.create_timer(timer_period, self.update_listener)
        #self.main_timer = self.create_timer(timer_period, self.main_loop)

        # Create the summary data
        self.create_summary_backup()

    def send_timestep_ok(self):
        self.service_req.name = "evaluator"
        future = self.cli.call_async(self.service_req)

    def create_dirs(self):
        if not os.path.exists(self.directory + "/raw_data"):
            os.makedirs(self.directory + "/raw_data")

    def main_loop(self):
        self.counter_loop += 1

        # Regarding the observation
        for target_name in self.targets_pose:
            p_target = self.targets_pose[target_name]
            for drone_name in self.drones_pose:
                p_drone = self.drones_pose[drone_name]
                if self.get_distance(p_target, p_drone) < self.obs_range:
                    self.counter_obs += 1
                    if target_name in self.obs_i:
                        self.obs_i[target_name] += 1
                    else:
                        self.obs_i[target_name] = 1
                    break  # The target is seen at least once, move on to the next one

        # Regarding the patrol
        self.map.new_time()
        for drone_name in self.drones_pose:
            p_drone = self.drones_pose[drone_name]
            self.map.observe(p_drone.x, p_drone.y, update_idleness=True)

        # If a cell is seen, update the interval
        mmap = self.map.get_map()
        for i in range(0, self.map_size_x_discrete):
            for j in range(0, self.map_size_y_discrete):
                if mmap[i][j] == 0 and self.previous_idleness[i, j] > 0:
                    self.intervals[i, j].append(self.previous_idleness[i, j])

        # Update the other metric
        self.max_idleness = self.get_idleness_max()
        self.max_idleness_blur = self.get_idleness_max_blur()
        self.update_idleness_average()

        # Update the previous idleness value
        mmap = self.map.get_map()
        for i in range(0, self.map_size_x_discrete):
            for j in range(0, self.map_size_y_discrete):
                self.previous_idleness[i, j] = mmap[i][j]

    def get_instant_idleness_average(self):
        # square = self.map_discrete * self.map_size * 2
        # S = np.sum(self.map.get_map()) / math.pow(square, 2)
        S = np.sum(self.map.get_map()) / (self.map_size_x_discrete * self.map_size_y_discrete)
        return S

    def update_idleness_average(self):
        self.average_idleness_sum += self.get_instant_idleness_average()

    def get_idleness_average(self):
        if self.counter_loop <= 0:
            self.counter_loop = 1
        return self.average_idleness_sum / self.counter_loop

    def get_instant_idleness_max(self):
        return np.max(self.map.get_map())

    def get_idleness_max(self):
        mm = self.get_instant_idleness_max()
        if mm > self.max_idleness:
            self.max_idleness = mm
        return self.max_idleness

    def get_instant_idleness_max_blur(self):
        m = copy.copy(self.map.get_map())
        kernel_size = int(self.obs_range * self.map_discrete * 2)
        kernel = np.ones((kernel_size, kernel_size)) / (kernel_size * kernel_size)
        c = cv2.filter2D(m, -1, kernel)
        return np.max(c)

    def get_idleness_max_blur(self):
        mm = self.get_instant_idleness_max_blur()
        if mm > self.max_idleness_blur:
            self.max_idleness_blur = mm
        return self.max_idleness_blur

    def get_A_metric(self):
        if self.counter_loop <= 0:
            return 0
        return self.counter_obs / self.counter_loop

    def get_sigma_metric(self):
        if self.counter_loop <= 0:
            return 0.
        S = 0
        T = self.counter_loop
        n = self.nbr_target
        if n <= 0:
            return 0.
        An = self.get_A_metric() / n
        for j in range(0, n):
            if 'target'+str(j) not in self.obs_i:
                self.obs_i['target'+str(j)] = 0
            alpha_j = 1/T * self.obs_i["target"+str(j)]
            S += math.pow(alpha_j - An, 2)

        sigma = math.sqrt(1/n * S)

        return sigma

    def get_H_metric(self):
        # From dict to list
        n = self.nbr_target
        p = np.zeros(n)
        i = 0
        for o_key in self.obs_i:
            p[i] = self.obs_i[o_key]
            i += 1

        S = np.sum(p)

        if S <= 0:
            return 0.

        p = p / S
        H = 0
        for p_i in p:
            if p_i < 1e-10:  # equal to 0
                p_i = 1e-10
            H += p_i * math.log2(p_i)
        return -1 * H

    def get_MI(self):
        """ Medium interval """
        N = 0  # Number of interval
        S = 0  # Sum of intervals
        for keys in self.intervals:
            N += len(self.intervals[keys])
            S += sum(self.intervals[keys])

        if N > 0:
            MI = float(S / N)
        else:
            MI = -1
        return MI

    def get_MSI(self):
        """ Mean Square Interval """
        N = 0  # Number of interval
        S = 0  # Sum of intervals ²
        for keys in self.intervals:
            N += len(self.intervals[keys])
            for value in self.intervals[keys]:
                S += math.pow(value, 2)

        if N > 0:
            MSI = math.sqrt(1 / N * S)
        else:
            MSI = -1

        return MSI

    def generate_lambda_drones_callback(self, drone_name):
        return lambda msg: self.drones_pose_callback(msg, drone_name)

    def generate_lambda_targets_callback(self, drone_name):
        return lambda msg: self.targets_pose_callback(msg, drone_name)

    def update_listener(self):
        # Issue from https://github.com/ros2/rclpy/issues/629
        topics = self.get_topic_names_and_types()
        for t in topics:
            sp = t[0].split('/')
            for s in sp:
                if "drone" in s:
                    if not s in self.drones_pose_subscription_:
                        drone_name = copy.copy(s)
                        self.drones_pose_subscription_[drone_name] = self.create_subscription \
                            (PoseStamped,
                             drone_name + '/pose',
                             self.generate_lambda_drones_callback(drone_name),
                             1)
                if "target" in s:
                    if not s in self.targets_pose_subscription_:
                        target_name = copy.copy(s)

                        self.targets_pose_subscription_[target_name] = self.create_subscription \
                                (
                                Odometry,
                                target_name + '/odom',
                                self.generate_lambda_targets_callback(target_name),
                                1
                            )

    def drones_pose_callback(self, data, agent_name):
        p = TrackPose()
        p.x = data.pose.position.x
        p.y = data.pose.position.y
        p.z = data.pose.position.z
        p.time = data.header.stamp
        self.drones_pose[agent_name] = p

    def targets_pose_callback(self, data, target_name):
        p = TrackPose()
        p.x = data.pose.pose.position.x
        p.y = data.pose.pose.position.y
        p.z = data.pose.pose.position.z
        p.time = data.header.stamp
        self.targets_pose[target_name] = p

    def create_summary_backup(self, filename="summary_exp_filenames.txt"):
        writing = {}
        writing['filename'] = self.continuous_backup_filename
        writing['method'] = self.strategy_name
        writing['nbr_agent'] = self.nbr_drone
        writing['nbr_target'] = self.nbr_target
        writing['obs_range'] = self.obs_range
        writing['com_range'] = self.com_range
        writing['map_size_x'] = self.map_size_x
        writing['map_size_y'] = self.map_size_y

        # Check if the file already exists, otherwise create header
        if not os.path.isfile(self.directory + filename):
            f = open(self.directory + filename, "a+")
            for h in self.summary_logs_header:
                f.write(h)
                f.write(";")

        f = open(self.directory + filename, "a+")
        f.write("\n")
        sep = ";"
        for h in self.summary_logs_header:
            f.write(str(writing[h]))
            f.write(sep)
        f.close()



    def backup_continuous_result(self):

        logs = {}
        logs['time'] = self.last_time_backup
        logs['I_oav_m'] = self.get_idleness_average()
        logs['I_inst'] = self.get_instant_idleness_average()
        logs['max_idleness'] = self.get_idleness_max()
        logs['max_idleness_region'] = self.get_idleness_max_blur()
        logs['A_metric'] = self.get_A_metric()
        logs['H_metric'] = self.get_H_metric()
        #logs['H_metric'] = 0.
        logs['sigma_n'] = self.get_sigma_metric()
        logs['MI'] = self.get_MI()
        logs['MSI'] = self.get_MSI()

        # Check if the file already exists, otherwise create header
        if not os.path.isfile(self.directory + "raw_data/" + self.continuous_backup_filename):
            f = open(self.directory + "raw_data/" + self.continuous_backup_filename, "a+")
            for h in self.continuous_logs_header:
                f.write(h)
                f.write(";")

        f = open(self.directory + "raw_data/" + self.continuous_backup_filename, "a+")
        f.write("\n")
        sep = ";"
        for h in self.continuous_logs_header:
            f.write(str(round(logs[h], 3)))
            f.write(sep)
        f.close()

    def backup_result(self, filename="result.txt"):
        if self.is_data_saved:
            print("[DEBUG] Back up result not possible, because already done previously.", flush=True)
            return

        logs = {}
        """
        self.logs_header = ["method", "nbr_agent", "nbr_target", "obs_range", "com_range", "duration", "I_oav_m",
          "map_size_x",
          "map_size_y", "max_idleness", "max_idleness_blur", "A_metric", "agent_speed", "target_speed",
          "target_behavior", "H_metric"]
        """
        logs['I_oav_m'] = self.get_idleness_average()
        logs['nbr_agent'] = self.nbr_drone
        logs['nbr_target'] = self.nbr_target
        logs['obs_range'] = self.obs_range
        logs['com_range'] = self.com_range
        logs['map_size_x'] = self.map_size_x
        logs['map_size_y'] = self.map_size_y
        logs['method'] = self.strategy_name
        logs['max_idleness'] = self.get_idleness_max()
        logs['max_idleness_region'] = self.get_idleness_max_blur()
        logs['A_metric'] = self.get_A_metric()
        logs['agent_speed'] = self.agent_speed
        logs['target_speed'] = self.target_speed
        logs['target_behavior'] = "random"
        logs['H_metric'] = self.get_H_metric()
        #logs['H_metric'] = 0.
        logs['sigma_n'] = self.get_sigma_metric()
        logs['MI'] = self.get_MI()
        logs['MSI'] = self.get_MSI()
        if self.mission_duration > 0:
            logs['duration'] = self.mission_duration
        else:
            logs['duration'] = self.time

        # Check if the file already exists, otherwise create header
        if not os.path.isfile(self.directory + filename):
            f = open(self.directory + filename, "a+")
            for h in self.logs_header:
                f.write(h)
                f.write(";")

        f = open(self.directory + filename, "a+")
        f.write("\n")
        sep = ";"
        for h in self.logs_header:
            f.write(str(logs[h]))
            f.write(sep)
        f.close()

    def clock_callback(self, data):
        """
        # Save data at the end of the simulation
        # During the simulation
        # and set 'STOP' to stop all the other nodes from launch trigger
        :param data:
        :return:
        """

        self.time = data.clock.sec
        # Backup every second
        if self.time - self.last_time_backup >= 1:
            self.last_time_backup = self.time
            self.main_loop()
            self.backup_continuous_result()

        if self.mission_duration > 0:
            if self.time > self.mission_duration - 2 and not self.is_data_saved:
                # Save the result, 2 seconds before the end of everything
                self.backup_result()
                self.is_data_saved = True
            if self.time > self.mission_duration:
                print('STOP', flush=True)  # Keyword to stop the launch execution

        if self.fast_rviz:
            self.send_timestep_ok()

    def get_distance(self, p1: TrackPose, p2: TrackPose):
        return math.sqrt(math.pow(p1.x - p2.x, 2) + math.pow(p1.y - p2.y, 2))


def main(args=None):
    rclpy.init(args=args)

    evaluator = Evaluator()
    try:
        while rclpy.ok():
            rclpy.spin_once(evaluator)
    except KeyboardInterrupt:
        print("Stopping the logs, try to log the result..")
        evaluator.backup_result()

    evaluator.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
