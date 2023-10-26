import math
import random

import rclpy

from ..ActionManager import ActionManager
from ..Controller import Controller
from ..Pid import PID
from .PF import PotentialField


class A_CMOMMT(Controller):

    def __init__(self):
        super().__init__()

        #self.y_des = 0.
        #self.x_des = 0.
        self.x_des, self.y_des = self.get_random_point()  # INIT
        self.speed_norm = 1
        self.pid_yaw = PID(1, 0.2, 0.)
        self.pid_x = PID(1.5, 0.2, 0.)
        self.pid_y = PID(1.5, 0.2, 0.)

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

        self.PF = PotentialField(self.do1, self.do2, self.do3, self.do4, self.dr1, self.dr2)

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
        safety = 0
        return random.uniform(-self.map_size_x + safety, self.map_size_x - safety), \
            random.uniform(-self.map_size_y + safety, self.map_size_y - safety)

    def desired_direction(self):

        if self.PF.is_empty():  # Random walk
            self.speed_norm = 1
            if not self.PF.is_still_empty() or self.distance_to_goal() < self.radius_to_goal:
                self.x_des, self.y_des = self.get_random_point()
            #self.update_angle()


        else:  # Follow potential field attraction
            angle, norm = self.PF.get_resultante()
            self.x_des = norm * math.cos(angle) + self.x
            self.y_des = norm * math.sin(angle) + self.y

            self.yaw_des = 0.
            #self.yaw_des = self.get_pipi_angle(angle)
            self.speed_norm = norm

    def feed_PF(self):
        self.PF.set_pose(self.x, self.y)

        for friend_name in self.friends_pose:
            pose = self.friends_pose[friend_name]
            self.PF.add_friend(pose.x, pose.y)

        for target_name in self.targets_pose:
            pose = self.targets_pose[target_name]
            self.PF.add_target(pose.x, pose.y)

    def pose_callback(self, data):
        super().pose_callback(data)

        # Feed the potential field with the surrounding targets / agents
        self.feed_PF()

        # Compute the direction
        self.desired_direction()

        Fx, Fy = self.get_force_collision_avoidance()

        if self.fast_rviz:
            self.update_angle()
            self.vx = Fx + self.max_speed * self.speed_norm
            self.vy = Fy
        else:
            Vx = self.pid_x.loop(self.x_des - self.x)
            Vy = self.pid_y.loop(self.y_des - self.y)
            self.vx = Fx + Vx * self.max_speed
            self.vy = Fy + Vy * self.max_speed
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
        controller = A_CMOMMT()
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
