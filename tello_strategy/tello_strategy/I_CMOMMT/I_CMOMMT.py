import copy
import math
import random

import cv2
import numpy as np
import rclpy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Int64MultiArray
from tello_msgs.msg import DesiredPose

from ..ActionManager import ActionManager
from ..Controller import Controller
from ..Pid import PID
from .PF import PotentialField, PotentialFieldPatrol
from .Map import Map
from . import ros_np_multiarray as rnm


class I_CMOMMT(Controller):

    def __init__(self):
        super().__init__()

        self.ID = int(self.namespace.replace('drone', ''))

        self.declare_parameter('sigma', 0.2 * 60 * 30)
        self.sigma = self.get_parameter('sigma').value

        self.N_highest_cells = 5  # Select the among the N highest cells

        self.x_des, self.y_des = 0., 0.
        self.x_motion, self.y_motion = 0., 0.
        self.x_des_pat, self.y_des_pat = 0., 0.
        self.speed_norm = 1
        self.pid_yaw = PID(1, 0.2, 0.)
        self.pid_x = PID(1, 0.2, 0.)
        self.pid_y = PID(1, 0.2, 0.)

        # Parameters for A-CMOMMT
        if self.is_exp:
            self.do1 = 0.2  # Repulsion until this distance
            self.do2 = 0.5  # Desired distance : Maximum magnitude = 1
            self.do3 = self.obs_range  # Last desired distance : Magnitude = 1. Start decrease
            if self.do3 < self.obs_range:
                self.do4 = self.obs_range  # Magnitude 0
            else:
                self.do4 = self.obs_range + 0.1
            self.dr1 = 1  # Extreme repulsion until this distance
            self.dr2 = 1.5  # Slow repulsion until 0 at this distance
        else:
            self.do1 = 0.5  # Repulsion until this distance
            self.do2 = 1  # Desired distance : Maximum magnitude = 1
            self.do3 = self.obs_range - 0.5  # Last desired distance : Magnitude = 1. Start decrease
            if self.do3 < self.obs_range:
                self.do4 = self.obs_range  # Magnitude 0
            else:
                self.do4 = self.obs_range + 0.1
            self.dr1 = 1  # Extreme repulsion until this distance
            self.dr2 = 1.5  # Slow repulsion until 0 at this distance

        # Perform mask
        self.kernel_size = int(self.obs_range * self.map_discrete * 2)
        self.kernel = np.ones(self.kernel_size) * 1 / self.kernel_size

        # For patrol
        self.index_x_des, self.index_y_des = int(0), int(0)  # Init, no importance about the first value
        self.init_pat_done = False

        self.myMap = Map(map_size_x=self.map_size_x, map_size_y=self.map_size_y,
                         obs_range=int(self.obs_range), discretization=self.map_discrete)

        self.PF = PotentialFieldPatrol(self.do1, self.do2, self.do3, self.do4, self.dr1, self.dr2,
                                       map=self.myMap)

        # Publisher for map's
        self.map_idleness_publisher_ = self.create_publisher(Int64MultiArray, self.namespace + "/map_idleness", 10)
        # Listener for map's
        self.friends_map_idleness_subscription_ = {}

        self.timer_publish_map = self.create_timer(1, self.publish_map)
        self.timer_update_listener_map = self.create_timer(1, self.update_listener_map_idleness)

        # Publisher for desired pose
        self.desired_pose_publisher_ = self.create_publisher(DesiredPose, self.namespace + '/desired_patrol_position',
                                                             10)
        # Listener for desired pose
        self.friends_desired_pose_subscription_ = {}
        self.friends_desired_pose = {}  # Store the last value of desired pose from friends


    def publish_desired_pose(self, x, y):
        p = DesiredPose()
        p.x = x
        p.y = y
        p.id = self.ID

        self.desired_pose_publisher_.publish(p)

    def update_listener_map_idleness(self):
        # Delete the listener that we are not communicate anymore

        # Avoid thread conflict, when friends_pose is updated during execution
        friends_pose = copy.copy(self.friends_pose)
        agent_to_be_removed = []

        # MAP
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

        # DESIRED POSE
        agent_to_be_removed = []
        for agent_name_callback in self.friends_desired_pose_subscription_:
            if agent_name_callback not in friends_pose:
                agent_to_be_removed.append(agent_name_callback)

        for a in agent_to_be_removed:
            self.friends_desired_pose_subscription_.pop(a)

        for agent_name in friends_pose:
            if agent_name not in self.friends_desired_pose_subscription_:
                drone_name = copy.copy(agent_name)
                self.friends_desired_pose_subscription_[agent_name] = self.create_subscription \
                    (DesiredPose,
                     drone_name + '/desired_patrol_position',
                     self.desired_pose_callback,
                     1)

    def desired_pose_callback(self, data):
        x = data.x
        y = data.y
        ID = data.id

        self.friends_desired_pose[ID] = {'x': x, 'y': y, 'ID': ID, 'time': float(self.get_clock().now().to_msg().sec)}
        # print(self.friends_desired_pose, flush=True)

    def map_idleness_callback(self, data):
        received_map = rnm.to_numpy_i64(data)
        self.myMap.update_from_map(received_map)

    def publish_map(self):
        msg = rnm.to_multiarray_i64(self.myMap.get_map().astype(np.int64))
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

    def get_random_point(self):
        safety = 1
        return random.uniform(-self.map_size_x + safety, self.map_size_x - safety), \
            random.uniform(-self.map_size_y + safety, self.map_size_y - safety)

    def desired_direction(self):
        if self.PF.is_empty():  # Random walk, but patrolling should avoid random to be called
            self.speed_norm = 1
            if not self.PF.is_still_empty() or self.distance_to_goal() < 0.5:
                self.x_des, self.y_des = self.get_random_point()
            self.x_motion = self.pid_x.loop(self.x_des - self.x)
            self.y_motion = self.pid_y.loop(self.y_des - self.y)
            # self.update_angle()

        else:  # Follow potential field attraction
            angle, norm = self.PF.get_resultante()
            self.x_motion = norm * math.cos(angle)
            self.y_motion = norm * math.sin(angle)

            # self.yaw_des = 0.
            if self.fast_rviz:
                self.yaw_des = self.get_pipi_angle(angle)
            else:
                self.yaw_des = 0.
            self.speed_norm = norm

    def feed_PF(self):
        self.PF.set_pose(self.x, self.y)

        # Check lambda value
        c = cv2.filter2D(self.myMap.get_map(), -1, self.kernel)
        lambda_ = math.tanh(np.max(c) / self.sigma)

        self.PF.set_lambda(lambda_)

        target_seen = False

        for friend_name in self.friends_pose:
            pose = self.friends_pose[friend_name]
            self.PF.add_friend(pose.x, pose.y)

        for target_name in self.targets_pose:
            target_seen = True
            pose = self.targets_pose[target_name]
            self.PF.add_target(pose.x, pose.y)

        if lambda_ > 0:
            self.patrol_force(target_seen)

    def compute_desired_position_to_patrol(self, m):
        """
        Objective : Check if the desired cell is within the N potential cell to visit.
        If not, generate N highest idleness cell, and select one among them randomly;
        :param m:
        :return:
        """
        # Check the cell targeted by other friends
        cells_to_avoid = []
        cells_targeted_by_friends = []
        for index in self.friends_desired_pose:
            desired_pose = self.friends_desired_pose[index]
            ID = desired_pose['ID']
            if ID > self.ID:
                # Check since how long it has been
                now = self.get_clock().now().to_msg().sec
                delta_t = now - desired_pose['time']

                if delta_t < 10:
                    cells_targeted_by_friends.append((desired_pose['x'], desired_pose['y']))

        for cells in cells_targeted_by_friends:
            x_index = int((cells[0] + self.map_size_x) * self.map_discrete)
            y_index = int((cells[1] + self.map_size_y) * self.map_discrete)
            cells_to_avoid.append((x_index, y_index))

        # Consider also the observation range around the desired pose
        for cells in cells_targeted_by_friends:
            x_index = int((cells[0] + self.map_size_x) * self.map_discrete)
            y_index = int((cells[1] + self.map_size_y) * self.map_discrete)
            obs_discret = int(self.obs_range * self.map_discrete)
            for xx in range(x_index - obs_discret, x_index + obs_discret):
                for yy in range(y_index - obs_discret, y_index + obs_discret):
                    cells_to_avoid.append((xx, yy))

        # Compute the desired pose
        m = copy.copy(np.round(m, decimals=1))
        # Get all the unique idleness value
        unique, _ = np.unique(m, return_counts=True)
        highest_idleness_values = - np.sort(-unique)

        # Check if our desired pose can be kept or do we need to change the direction ?
        desired_idleness = m[self.index_x_des, self.index_y_des]
        if (self.index_x_des, self.index_y_des) in cells_to_avoid \
                or desired_idleness not in highest_idleness_values[:self.N_highest_cells] \
                or not self.init_pat_done:
            # If we have to avoid the selected cell, or if the desired cell is not on the N top list of highest idleness
            # Or if the first time we are initialized
            self.init_pat_done = True
            # Select another value
            candidat_cells = []
            for index_value, high_value in enumerate(highest_idleness_values):
                # Stop condition
                if len(candidat_cells) >= self.N_highest_cells:
                    break

                cells = np.where(m == high_value)
                cells_to_examine = []

                # Compute the position of all the cells with the highest idleness level
                for i in range(0, len(cells[0])):
                    x_index, y_index = cells[0][i], cells[1][i]
                    if (x_index, y_index) not in cells_to_avoid:
                        cells_to_examine.append((x_index, y_index))

                # If all cells have to be considered for this high value
                if len(candidat_cells) + len(cells_to_examine) < self.N_highest_cells:
                    candidat_cells += cells_to_examine
                else:  # Case where we have to select only some cells to get the N candidat cells
                    nbr_candidat_needed = self.N_highest_cells - len(candidat_cells)
                    candidat_cells += self.get_closest_candidat_(nbr_candidat_needed, cells_to_examine)

            # Select randomly among the N candidats cells
            if len(candidat_cells) > 0:
                rand_value = random.randrange(0, len(candidat_cells))
                winner_cell = candidat_cells[rand_value]
                self.index_x_des, self.index_y_des = winner_cell[0], winner_cell[1]
            else:
                self.index_x_des, self.index_y_des = random.randint(0, self.map_size_x), random.randint(0,

    def get_closest_candidat_(self, nbr_candidat_needed, cells_to_examine):
        cells_selected = []
        cells_distance = []
        for cell in cells_to_examine:
            x, y = self.index2pose(cell[0], cell[1])
            dist = math.sqrt(math.pow(self.x - x, 2) + math.pow(self.y - y, 2))
            cells_distance.append((dist, cell[0], cell[1]))

        # Sort by the closest distance
        distance_sorted = sorted(cells_distance, key=lambda x: x[0])
        i = 0
        for cell in distance_sorted:
            if i > nbr_candidat_needed:
                break
            i += 1
            cells_selected.append((cell[1], cell[2]))

        return cells_selected

    def index2pose(self, x_index, y_index):
        xx = (x_index / self.map_discrete) - self.map_size_x
        yy = (y_index / self.map_discrete) - self.map_size_x
        return xx, yy

    def pose2index(self, x, y):
        x_index = x * self.map_discrete + self.map_size_x
        y_index = y * self.map_discrete + self.map_size_y
        return x_index, y_index

    def patrol_force(self, target_seen):

        c = cv2.filter2D(self.myMap.get_map(), -1, self.kernel)

        self.compute_desired_position_to_patrol(c)

        self.x_des_pat = (self.index_x_des / self.map_discrete) - self.map_size_x
        self.y_des_pat = (self.index_y_des / self.map_discrete) - self.map_size_y  # Recadrage autour de (0,0)

        self.publish_desired_pose(self.x_des_pat, self.y_des_pat)

        self.PF.add_patrolling_force(self.x_des_pat, self.y_des_pat, target_seen)

    def pose_callback(self, data):
        super().pose_callback(data)

        self.myMap.new_time()
        self.myMap.observe(self.x, self.y, update_idleness=True)

        # Feed the potential field with the surrounding targets / agents
        self.feed_PF()

        # Compute the direction
        self.desired_direction()

        Fx, Fy = self.get_force_collision_avoidance()

        if self.fast_rviz:
            self.vx = Fx + self.max_speed * self.speed_norm
            self.vy = Fy
        else:
            self.vx = Fx + self.x_motion * self.max_speed
            self.vy = Fy + self.y_motion * self.max_speed
            self.v_yaw = self.pid_yaw.loop(-1*(0.-self.yaw))
            self.vz = self.pid_z.loop(self.des_z - self.z)

        self.PF.reset()

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
        controller = I_CMOMMT()
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
