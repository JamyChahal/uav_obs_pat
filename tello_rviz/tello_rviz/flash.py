import math
import random

import geometry_msgs
import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from tello_msgs.srv import TelloAction


def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return [qx, qy, qz, qw]

class flash(Node):

    def __init__(self):
        super().__init__('flash')
        print("Flash ready to work")
        self.duration = 0

        self.declare_parameter("nbr_drone", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("nbr_target", rclpy.Parameter.Type.INTEGER)

        self.nbr_drone = self.get_parameter("nbr_drone").get_parameter_value().integer_value
        self.nbr_target = self.get_parameter("nbr_target").get_parameter_value().integer_value

        self.declare_parameter("map_size_x", 10.)
        self.declare_parameter("map_size_y", 10.)
        self.map_size_x = self.get_parameter("map_size_x").get_parameter_value().double_value
        self.map_size_y = self.get_parameter("map_size_y").get_parameter_value().double_value

        self.declare_parameter("timestep", rclpy.Parameter.Type.DOUBLE)
        self.timestep_period = self.get_parameter("timestep").get_parameter_value().double_value

        # TF2
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, super())
        self.br = tf2_ros.TransformBroadcaster(self)

        self.positions = {}  # List of the pose
        self.desired_speed_sub_ = {}  # Listeners
        self.positions_pub_ = {}  # Pose publisher
        self.takeoff_service_ = {}  # Fake service listener
        self.desired_speed = {}

        drone_init_pose, target_init_pose = self.generate_init_pose()

        for i in range(0, self.nbr_drone):
            name = "drone"+str(i)
            p = self.Position(name)
            p.init(drone_init_pose[i][0], drone_init_pose[i][1])
            self.positions[name] = p
            sub = self.create_subscription(Twist, name+"/cmd_vel", self.generate_lambda_speed_callback(name), qos_profile=1)
            self.desired_speed_sub_[name] = sub
            self.desired_speed[name] = self.DesiredSpeed()
            self.positions_pub_[name] = self.create_publisher(PoseStamped, name + '/pose', 10)
            self.takeoff_service_[name] = self.create_service(TelloAction, name+'/tello_action', self.takeoff_service)

        for i in range(0, self.nbr_target):
            name = "target"+str(i)
            p = self.Position(name)
            p.init(target_init_pose[i][0], target_init_pose[i][1])
            self.positions[name] = p
            sub = self.create_subscription(Twist, name+"/cmd_vel", self.generate_lambda_speed_callback(name), qos_profile=1)
            self.desired_speed_sub_[name] = sub
            self.desired_speed[name] = self.DesiredSpeed()
            self.positions_pub_[name] = self.create_publisher(Odometry, name + '/odom', 10)



        # Publish pose at each time step

        self.timer_pub_pose = self.create_timer(self.timestep_period, self.publish_pose_loop)
        self.timer_update_pose = self.create_timer(self.timestep_period, self.update_pose)

    class Position:
        def __init__(self, name="noname"):
            self.x = 0.
            self.y = 0.
            self.z = 0.
            self.yaw = 0.
            self.name = name
            if "drone" in name:
                self.z = 2.

        def init(self, x, y):
            self.x = float(x)
            self.y = float(y)

    class DesiredSpeed:
        def __init__(self):
            self.vx = 0.
            self.vy = 0.
            self.vz = 0.
            self.vyaw = 0.

        def update(self, vx, vy, vz, vyaw):
            self.vx = vx
            self.vy = vy
            self.vz = vz
            self.vyaw = vyaw

    def generate_init_pose(self):
        targets_pose = []
        drones_pose = []

        random.seed()

        for nt in range(0, self.nbr_target):
            x, y = self.generate_random_pose()
            while (x, y) in targets_pose:
                x, y = self.generate_random_pose()
            targets_pose.append((str(x), str(y)))

        for nd in range(0, self.nbr_drone):
            x, y = self.generate_random_pose()
            while (x, y) in targets_pose or (x, y) in drones_pose:
                x, y = self.generate_random_pose()
            drones_pose.append((str(x), str(y)))

        return drones_pose, targets_pose

    def generate_random_pose(self):
        return random.uniform(-self.map_size_x, self.map_size_x), random.randint(-self.map_size_y, self.map_size_y)

    def takeoff_service(self, request, response):
        # Always say OK
        response.rc = 1
        return response

    def generate_lambda_speed_callback(self, drone_name):
        return lambda msg: self.speed_callback(msg, drone_name)

    def publish_transform(self, name):
        p = self.positions[name]

        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t._child_frame_id = name + "/base_link"
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z
        q = euler_to_quaternion(0, 0, p.yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.br.sendTransform(t)

    def publish_pose_loop(self):
        for key in self.positions_pub_:
            self.publish_pose(key)

    def publish_pose(self, name):
        msg = None
        p = self.positions[name]
        q = euler_to_quaternion(0, 0, p.yaw)
        if "drone" in name:
            msg = PoseStamped()
            msg.pose.position.x = p.x
            msg.pose.position.y = p.y
            msg.pose.position.z = p.z
            msg.pose.orientation.x = q[0]
            msg.pose.orientation.y = q[1]
            msg.pose.orientation.z = q[2]
            msg.pose.orientation.w = q[3]
        else:
            msg = Odometry()
            msg.pose.pose.position.x = p.x
            msg.pose.pose.position.y = p.y
            msg.pose.pose.position.z = p.z
            msg.pose.pose.orientation.x = q[0]
            msg.pose.pose.orientation.y = q[1]
            msg.pose.pose.orientation.z = q[2]
            msg.pose.pose.orientation.w = q[3]

        self.positions_pub_[name].publish(msg)

    def get_pipi_angle(self, angle):
        # Check if angle is between -pi and pi
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def update_pose(self):
        for name in self.desired_speed:
            p = self.positions[name]
            des = self.desired_speed[name]

            p.yaw = des.vyaw

            #p.yaw = self.get_pipi_angle(p.yaw)
            p.x += (des.vx * math.cos(p.yaw) - des.vy * math.sin(p.yaw)) * self.timestep_period
            p.y += (des.vx * math.sin(p.yaw) + des.vy * math.cos(p.yaw)) * self.timestep_period
            self.positions[name] = p

            self.publish_transform(name)


    def speed_callback(self, data, name):
        vx = data.linear.x
        vy = data.linear.y
        vz = data.linear.z
        vyaw = data.angular.z

        self.desired_speed[name].update(vx, vy, vz, vyaw)
        """
        p = self.positions[name]
        p.yaw += vyaw * self.timestep_period

        p.yaw = self.get_pipi_angle(p.yaw)
        self.positions[name] = p
        """

def main(args=None):
    rclpy.init(args=args)

    f = flash()

    rclpy.spin(f)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    f.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
