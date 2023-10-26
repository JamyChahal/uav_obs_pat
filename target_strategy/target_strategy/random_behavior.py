import math
import random

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from nav_msgs.msg import Odometry

from .Pid import PID

from tello_rviz_msg.srv import TimeStepOK


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

        self.desired_x = 0
        self.desired_y = 0
        self.desired_yaw = 0
        self.new_desired_pose()

        self.PID = PID(0.5, 0, 0)

        ### CREATE THE SUBSCRIBER FOR IMAGE RECEPTION
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        # Odometry : nav_msgs/msg/Odometry /target1/odom
        self.odometry_subscription_ = self.create_subscription(Odometry, self.namespace+"/odom", self.pose, qos_profile=qos_policy)

    def send_timestep_ok(self):
        self.service_req.name = self.namespace
        future = self.cli.call_async(self.service_req)
    def new_desired_pose(self):
        # TODO : called when need to change the direction, if goal reached or if obstacles
        self.desired_x = random.uniform(-self.map_size_x, self.map_size_x)
        self.desired_y = random.uniform(-self.map_size_y, self.map_size_y)
        self.update_angle()

    def update_angle(self):
        self.desired_yaw = math.atan2(self.desired_y - self.y, self.desired_x - self.x)
        self.desired_yaw = self.get_pipi_angle(self.desired_yaw)

    def distance_to_goal(self):
        return math.sqrt(math.pow(self.x - self.desired_x, 2) + math.pow(self.y - self.desired_y, 2))

    def pose(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

        ax = data.pose.pose.orientation.x
        ay = data.pose.pose.orientation.y
        az = data.pose.pose.orientation.z
        aw = data.pose.pose.orientation.w

        _, _, self.yaw = euler_from_quaternion(ax, ay, az, aw)

        if self.distance_to_goal() < 1:
            self.new_desired_pose()
            #print(self.desired_x, self.desired_y)

        self.update_angle()

        # IF |alpha_d - alpha| > pi
        #   IF alpha_d > alpha => alpha_d = alpha_d + 2pi
        #   ELSE alpha_d = alpha_d - 2pi

        if abs(self.desired_yaw - self.yaw) > math.pi:
            if self.desired_yaw > self.yaw:
                self.desired_yaw -= 2*math.pi
            else:
                self.desired_yaw += 2*math.pi

        self.v_yaw = self.PID.loop(self.desired_yaw - self.yaw)

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
            msg.angular.z = self.v_yaw
        self.cmd_vel_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    controller = Controller()
    rclpy.spin(controller)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
