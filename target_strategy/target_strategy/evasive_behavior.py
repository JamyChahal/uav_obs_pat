import copy
import math
import random
from geometry_msgs.msg import PoseStamped

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from nav_msgs.msg import Odometry

from .Pid import PID

from tello_rviz_msg.srv import TimeStepOK

class PotentialFieldTarget:

    def __init__(self, obs_range):
        self.potential_field = []

        # Parameters for running away targets
        # ROBOT
        self.dr1 = obs_range - 0.1  # Extreme repulsion until this distance
        self.dr2 = obs_range  # Slow repulsion until 0 at this distance

        self.fr1a = 1 / float(self.dr2 - self.dr1)
        self.fr1b = -1 - self.fr1a * self.dr1

        self.was_empty = True

    def see_agent(self, x, y):
        dist = self.get_ground_distance(x, y)
        magnitude = self.get_robot_magnitude(dist)
        orientation = math.atan2(y, x)
        self.potential_field.append([orientation, magnitude])

    def reset(self):
        if len(self.potential_field) > 0:
            self.was_empty = False
        else:
            self.was_empty = True
        self.potential_field = []

    def is_empty(self):
        if len(self.potential_field) == 0:
            return True
        else:
            return False

    def is_still_empty(self):
        if self.was_empty and len(self.potential_field) == 0:
            return True
        else:
            return False

    def get_resultante(self):
        if not self.is_empty():
            angle, norm = self.circular_mean()
            return angle, norm
        else:
            #print("get_resultante should not be called yet!")
            return 0, 0

    def circular_mean(self):
        x = y = 0
        for i in range(0, len(self.potential_field)):
            x += math.cos(self.potential_field[i][0]) * self.potential_field[i][1]
            y += math.sin(self.potential_field[i][0]) * self.potential_field[i][1]

        mean_angle = math.atan2(y, x)
        norm = math.sqrt(math.pow(x, 2) + math.pow(y, 2))

        # print("Global target : " + str(x) + " : " + str(y))
        # print("Angle " + str(mean_angle) + " norm " + str(norm))

        return mean_angle, norm

    def get_robot_magnitude(self, distance):
        m = 0
        if distance <= self.dr1:
            m = -1
        if self.dr1 < distance <= self.dr2:
            m = self.fr1a * distance + self.fr1b
        if distance > self.dr2:
            m = 0

        # print(str(self.name) + " robot distance " + str(distance) + " + repulsive magnitude : " + str(m))
        return m

    def get_ground_distance(self, x, y):
        return math.sqrt(math.pow(x, 2) + math.pow(y, 2))

    def get_debug_info(self):
        if self.is_empty():
            print("PF is empty")
        else:
            for pf in self.potential_field:
                print("NORM:" + str(pf[1]) + " ORIENT:" + str((pf[0])))

    def add_destination(self, x, y):
        magnitude = 0.25
        orientation = math.atan2(y, x)
        self.potential_field.append([orientation, magnitude])

class TrackPose:

    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.time = 0
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians
class Controller(Node):

    def __init__(self):
        super().__init__('controller_target')

        self.declare_parameter('namespace', 'target1')
        self.declare_parameter('map_size_x', 10.)
        self.declare_parameter('map_size_y', 10.)
        self.namespace = self.get_parameter('namespace').value
        self.map_size_x = self.get_parameter('map_size_x').value
        self.map_size_y = self.get_parameter('map_size_y').value
        self.declare_parameter('max_speed', 1.0)
        self.max_speed = float(self.get_parameter('max_speed').value)

        self.declare_parameter("det_range", 5.)
        self.detection_range = self.get_parameter('det_range').value

        self.declare_parameter("fast_rviz", False)
        self.fast_rviz = self.get_parameter('fast_rviz').value

        if self.fast_rviz:
            # Execute service
            self.cli = self.create_client(TimeStepOK, 'timestep')
            while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Wait for timestep service')
            self.service_req = TimeStepOK.Request()
            self.send_timestep_ok()

        ### CREATE THE PUBLISHER FOR COMMAND VELOCITY
        self.cmd_vel_publisher_ = self.create_publisher(Twist, self.namespace+'/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.cmd_vel_loop)
        # Init desired speed
        self.vx = self.max_speed
        self.vy = 0.0
        self.vz = 0.0
        self.v_yaw = 0.0

        self.x = 0.
        self.y = 0.
        self.yaw = 0.

        self.desired_x = 0.
        self.desired_y = 0.
        self.desired_yaw = 0.
        self.new_desired_pose()

        # Listening to the surrounding agent
        self.agents_pose = {}
        self.agents_pose_subscription_ = {}

        self.PF = PotentialFieldTarget(self.detection_range)

        self.PID = PID(1., 0, 0)

        timer_period = 1  # seconds
        self.update_listener()  # Start at the beginning
        self.timer = self.create_timer(timer_period, self.update_listener)

        ### CREATE THE SUBSCRIBER FOR IMAGE RECEPTION
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        # Odometry : nav_msgs/msg/Odometry /target1/odom
        self.odometry_subscription_ = self.create_subscription(Odometry, self.namespace+"/odom", self.pose, qos_profile=qos_policy)

    def update_listener(self):
        # Issue from https://github.com/ros2/rclpy/issues/629
        topics = self.get_topic_names_and_types()
        for t in topics:
            sp = t[0].split('/')
            for s in sp:
                if "drone" in s:
                    if not s in self.agents_pose_subscription_ and s != self.namespace:
                        drone_name = copy.copy(s)
                        self.agents_pose_subscription_[drone_name] = self.create_subscription \
                            (PoseStamped,
                             drone_name + '/pose',
                             self.generate_lambda_agents_callback(drone_name),
                             1)

    def generate_lambda_agents_callback(self, drone_name):
        return lambda msg: self.agents_pose_callback(msg, drone_name)

    def agents_pose_callback(self, data, agent_name):
        p = TrackPose()
        p.x = data.pose.position.x
        p.y = data.pose.position.y
        p.z = data.pose.position.z
        p.time = data.header.stamp

        if self.get_distance(p) <= self.detection_range:
            self.agents_pose[agent_name] = p
        elif agent_name in self.agents_pose:
            self.agents_pose.pop(agent_name)

    def get_distance(self, p: TrackPose):
        return math.sqrt(math.pow(p.x - self.x, 2) + math.pow(p.y - self.y, 2))

    def send_timestep_ok(self):
        self.service_req.name = self.namespace
        future = self.cli.call_async(self.service_req)
    def new_desired_pose(self):
        self.desired_x = random.uniform(-self.map_size_x, self.map_size_x)
        self.desired_y = random.uniform(-self.map_size_y, self.map_size_y)

    def update_angle(self):
        self.desired_yaw = math.atan2(self.desired_y - self.y, self.desired_x - self.x)
        self.desired_yaw = self.get_pipi_angle(self.desired_yaw)

    def distance_to_goal(self):
        return math.sqrt(math.pow(self.x - self.desired_x, 2) + math.pow(self.y - self.desired_y, 2))

    def is_close_to_outside(self):
        safety = 2.
        if self.x < - self.map_size_x + safety \
            or self.x > self.map_size_x - safety \
            or self.y < - self.map_size_y + safety \
            or self.y > self.map_size_y - safety:
            return True
        else:
            return False


    def pose(self, data):
        # Get its own state
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

        ax = data.pose.pose.orientation.x
        ay = data.pose.pose.orientation.y
        az = data.pose.pose.orientation.z
        aw = data.pose.pose.orientation.w

        _, _, self.yaw = euler_from_quaternion(ax, ay, az, aw)

        # Strategy

        if self.distance_to_goal() < 1:
            self.new_desired_pose()
        self.PF.add_destination(self.desired_x - self.x, self.desired_y - self.y)

        if not self.is_close_to_outside():
            for agent in self.agents_pose:
                a = self.agents_pose[agent]
                self.PF.see_agent(a.x - self.x, a.y - self.y)

        # IF |alpha_d - alpha| > pi
        #   IF alpha_d > alpha => alpha_d = alpha_d + 2pi
        #   ELSE alpha_d = alpha_d - 2pi

        angle, norm = self.PF.get_resultante()
        self.desired_yaw = angle

        if abs(self.desired_yaw - self.yaw) > math.pi:
            if self.desired_yaw > self.yaw:
                self.desired_yaw -= 2*math.pi
            else:
                self.desired_yaw += 2*math.pi

        self.v_yaw = self.PID.loop(self.desired_yaw - self.yaw)
        self.vx = 1. * self.max_speed

        self.PF.reset()

        if self.fast_rviz:
            self.send_timestep_ok()

    def get_pipi_angle(self, angle):
        # Check if angle is between -pi and pi
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def cmd_vel_loop(self):
        """
        I'm used to publish every 0.5 seconds the velocity command.
        You don't need to modify me, just modify vx, vy and v_yaw when you need it.
        """
        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.linear.z = self.vz
        if self.fast_rviz:
            msg.angular.z = self.desired_yaw
        else:
            msg.angular.z = float(self.v_yaw)
        self.cmd_vel_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    controller = Controller()
    rclpy.spin(controller)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
