import copy
import math
import random

import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tello_msgs.msg import FlightData

from tello_rviz_msg.srv import TimeStepOK
from .ActionManager import ActionManager

from .Pid import PID


def get_yaw_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return yaw_z  # in radians

class TrackPose:

    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.time = 0


class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        random.seed()

        self.declare_parameter('namespace', 'drone1')
        self.namespace = self.get_parameter('namespace').value

        ### CREATE THE PUBLISHER FOR COMMAND VELOCITY
        self.cmd_vel_publisher_ = self.create_publisher(Twist, self.namespace + '/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.cmd_vel_loop)

        self.declare_parameter("fast_rviz", False)
        self.fast_rviz = self.get_parameter('fast_rviz').value

        self.declare_parameter('is_exp', False)
        self.is_exp = self.get_parameter('is_exp').value

        self.radius_to_goal = 1.
        if self.is_exp:
            self.radius_to_goal = 0.4

        if self.fast_rviz:
            # Execute service
            self.cli = self.create_client(TimeStepOK, 'timestep')
            while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Wait for timestep service')
            self.service_req = TimeStepOK.Request()
            self.send_timestep_ok()

        # Init desired speed
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.v_yaw = 0.0
        self.yaw_des = 0.

        # Const. desired altitude
        self.history_z = [0.] * 20
        self.des_z = 0.8
        self.pid_z = PID(1., 0., 0.)

        # Self pose
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0

        self.battery = 0.0

        # Pose of friends and targets
        self.friends_pose = {}
        self.targets_pose = {}

        # Variables for obs. pat problem
        self.declare_parameter('com_range', 10.)
        self.com_range = self.get_parameter('com_range').get_parameter_value().double_value
        self.declare_parameter('obs_range', 10.)
        self.obs_range = self.get_parameter('obs_range').get_parameter_value().double_value

        self.declare_parameter('map_size_x', 10.)
        self.declare_parameter('map_size_y', 10.)
        self.map_size_x = self.get_parameter('map_size_x').get_parameter_value().double_value
        self.map_size_y = self.get_parameter('map_size_y').get_parameter_value().double_value

        self.declare_parameter('map_discrete', 1)
        self.map_discrete = self.get_parameter('map_discrete').get_parameter_value().integer_value
        
        self.declare_parameter('max_speed', 1.0)
        self.max_speed = float(self.get_parameter('max_speed').value)
        # Avoidance controller
        self.declare_parameter('safety_range', 3.0)
        self.declare_parameter('dangerous_range', 5.0)
        self.dangerous_range = float(self.get_parameter('dangerous_range').value)  # Extreme repulsion until this distance
        self.safety_range = float(self.get_parameter('safety_range').value)  # Slow repulsion until 0 at this distance
        if abs(self.dangerous_range - self.safety_range) < 1e-5:  # Avoid dividing by zero
            self.safety_range = self.dangerous_range + 0.01
        if self.safety_range < self.dangerous_range: # Cannot be possible
            self.safety_range = self.dangerous_range + 0.01

        self.fr1a = 1 / float(self.safety_range - self.dangerous_range)
        self.fr1b = -1 - self.fr1a * self.dangerous_range


        ### CREATE THE SUBSCRIBER FOR POSE RECEPTION
        self.qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                               history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                               depth=1)
        self.pose_subscription_ = self.create_subscription(PoseStamped, self.namespace + '/pose', self.pose_callback,
                                                           qos_profile=self.qos_policy)

        # Subscribe to the state
        self.state_sub_ = self.create_subscription(FlightData, self.namespace + "/flight_data", self.callback_state, 1)

        ### CHECK FOR THE LIST OF AVAILABLE PEOPLE AROUND
        self.friends_pose_subscription_ = {}
        self.targets_pose_subscription_ = {}

        timer_period = 1  # seconds
        self.update_listener()  # Start at the beginning
        self.timer = self.create_timer(timer_period, self.update_listener)

    def call_for_emergency_landing(self):
        # ASK FOR LANDING
        self.get_logger().info("ASK FOR EMERGENCY LANDING")
        self.get_logger().info("ALTITUDE REACHED : " + str(self.history_z))
        action_manager = ActionManager()
        action_manager.ask_for_landing()
        while rclpy.ok():
            rclpy.spin_once(action_manager)
            if action_manager.future.done():
                try:
                    response = action_manager.future.result()
                    if response.rc == 1:  # OK
                        print("Landing is a success !", flush=True)
                        break  # Only if landing is ok
                    else:
                        action_manager.ask_for_landing()
                        print("Ask again for landing ! BAD NEWS FROM DRONE", flush=True)
                except Exception as e:
                    action_manager.get_logger().info("Service call failed %r" % (e,))
                break

    def send_timestep_ok(self):
        self.service_req.name = self.namespace
        future = self.cli.call_async(self.service_req)

    def generate_lambda_friends_callback(self, drone_name):
        return lambda msg: self.friends_pose_callback(msg, drone_name)

    def generate_lambda_targets_callback(self, drone_name):
        return lambda msg: self.targets_pose_callback(msg, drone_name)

    def callback_state(self, data):
        self.battery = data.bat
        self.yaw = data.yaw * math.pi / 180

        if self.battery < 21:
            print("DRONE " + str(self.namespace) + " BATTERY = " + str(self.battery), flush=True)

    def update_listener(self):
        # Issue from https://github.com/ros2/rclpy/issues/629
        topics = self.get_topic_names_and_types()
        for t in topics:
            sp = t[0].split('/')
            for s in sp:
                if "drone" in s:
                    if not s in self.friends_pose_subscription_ and s != self.namespace:
                        drone_name = copy.copy(s)
                        self.friends_pose_subscription_[drone_name] = self.create_subscription \
                            (PoseStamped,
                             drone_name + '/pose',
                             self.generate_lambda_friends_callback(drone_name),
                             1)
                        # TODO : Delete the subscription if not available anymore
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
                        # TODO : Delete the subscription if not available anymore

    def get_distance(self, p: TrackPose):
        return math.sqrt(math.pow(p.x - self.x, 2) + math.pow(p.y - self.y, 2))

    def friends_pose_callback(self, data, agent_name):
        p = TrackPose()
        p.x = data.pose.position.x
        p.y = data.pose.position.y
        p.z = data.pose.position.z
        p.time = data.header.stamp

        if self.get_distance(p) <= self.com_range:
            self.friends_pose[agent_name] = p
        elif agent_name in self.friends_pose:
            self.friends_pose.pop(agent_name)

    def targets_pose_callback(self, data, target_name):
        p = TrackPose()
        p.x = data.pose.pose.position.x
        p.y = data.pose.pose.position.y
        p.z = data.pose.pose.position.z
        p.time = data.header.stamp

        if self.get_distance(p) <= self.obs_range:
            self.targets_pose[target_name] = p
        elif target_name in self.targets_pose:
            self.targets_pose.pop(target_name)

    def get_distance_from_env_limits(self, pose_axis, axis='x'):
        """
        Return the distance separating the UAV from the env boundaries on the specific axis
        """
        if axis == 'x':
            if self.map_size_x >= pose_axis >= - self.map_size_x:
                return 0.
            elif pose_axis > self.map_size_x:
                return pose_axis - self.map_size_x
            else:
                return abs(pose_axis + self.map_size_x)
        elif axis == 'y':
            if self.map_size_y >= pose_axis >= - self.map_size_y:
                return 0.
            elif pose_axis > self.map_size_y:
                return pose_axis - self.map_size_y
            else:
                return abs(pose_axis + self.map_size_y)

    def motion_safety(self):
        """
        Used to avoid linear motion upper than limits,
        and avoid drone to leave the environment
        """
        # Set limits to the speeds
        self.vx = self.minmax(self.vx, -self.max_speed, self.max_speed)
        self.vy = self.minmax(self.vy, -self.max_speed, self.max_speed)
        self.vz = self.minmax(self.vz, -0.1, 0.1)

        # Avoid going out env.
        x_distance = self.get_distance_from_env_limits(self.x, axis='x')
        y_distance = self.get_distance_from_env_limits(self.y, axis='y')

        # Get future position with vx motion
        future_x_vx = self.x + self.vx * math.cos(self.yaw)
        future_y_vx = self.y + self.vx * math.sin(self.yaw)

        # Check how far are we going with this vx motion
        x_vx_distance = self.get_distance_from_env_limits(future_x_vx, axis='x')
        y_vx_distance = self.get_distance_from_env_limits(future_y_vx, axis='y')

        if x_vx_distance > x_distance or y_vx_distance > y_distance:
            self.vx = 0.

        # Get future position with vy motion
        future_x_vy = self.x - self.vy * math.sin(self.yaw)
        future_y_vy = self.y + self.vy * math.cos(self.yaw)

        # Check how far are we going with this vy motion
        x_vy_distance = self.get_distance_from_env_limits(future_x_vy, axis='x')
        y_vy_distance = self.get_distance_from_env_limits(future_y_vy, axis='y')

        if x_vy_distance > x_distance or y_vy_distance > y_distance:
            self.vy = 0.


    def cmd_vel_loop(self):
        """
        I'm used to publish every 0.5 seconds the velocity command.
        You don't need to modify me, just modify vx, vy and v_yaw when you need it.
        """
        self.motion_safety()

        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.linear.z = self.vz
        if self.fast_rviz:
            msg.angular.z = self.yaw_des
        else:
            msg.angular.z = self.v_yaw

        self.cmd_vel_publisher_.publish(msg)

    def pose_callback(self, data):
        self.x = data.pose.position.x
        self.y = data.pose.position.y
        self.z = data.pose.position.z

        self.history_z.append(self.z)
        self.history_z.pop(0)
        cpt = 0
        for h in self.history_z:
            if h > 2:
                cpt += 1

        if cpt >= len(self.history_z):
            self.call_for_emergency_landing()

        rx = data.pose.orientation.x
        ry = data.pose.orientation.y
        rz = data.pose.orientation.z
        rw = data.pose.orientation.w

        #self.yaw = get_yaw_from_quaternion(rx, ry, rz, rw)

    def get_force_collision_avoidance(self):
        def get_friend_magnitude(distance):
            m = 0
            if distance <= self.dangerous_range:
                m = -1
            if self.dangerous_range < distance <= self.safety_range:
                m = self.fr1a * distance + self.fr1b
            if distance > self.safety_range:
                m = 0

            return m

        def circular_mean(magnitudes, orientations):
            x = y = 0
            for i in range(0, len(magnitudes)):
                x += math.cos(orientations[i]) * magnitudes[i]
                y += math.sin(orientations[i]) * magnitudes[i]

            mean_angle = math.atan2(y, x)
            norm = math.sqrt(math.pow(x, 2) + math.pow(y, 2))

            return mean_angle, norm

        magn = []
        ori = []
        for friends in self.friends_pose:
            p = self.friends_pose[friends]
            dist = self.get_distance(p)
            if dist < self.safety_range:
                #print("[DEBUG] Trigger dist. " + str(dist) + ' from ' + str(self.namespace), flush=True)
                magn.append(get_friend_magnitude(dist))
                ori.append(math.atan2(self.y - p.y, self.x - p.x))

        orientation, magnitude = circular_mean(magn, ori)
        Fx = -1 * magnitude * math.cos(orientation - self.yaw)
        Fy = -1 * magnitude * math.sin(orientation - self.yaw)

        return Fx, Fy

    def minmax(self, a, minn, maxn):
        return max(min(maxn, a), minn)
