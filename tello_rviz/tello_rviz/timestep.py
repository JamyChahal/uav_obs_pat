import math
import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
#from pynput import keyboard
#from rclpy.clock import Clock
from rosgraph_msgs.msg import Clock

from rclpy.node import Node

from builtin_interfaces.msg import Time
from tello_rviz_msg.srv import TimeStepOK


class timestep(Node):

    def __init__(self):
        super().__init__('timestep')

        print("New timestep built")
        self.duration_sec = 0
        self.duration_nsec = 0
        self.watch_dog_timeout = 5  # sec

        self.declare_parameter("timestep", rclpy.Parameter.Type.DOUBLE)
        self.timestep_period = self.get_parameter("timestep").get_parameter_value().double_value

        self.declare_parameter("time_wait", 0.)
        self.time_wait = self.get_parameter("time_wait").get_parameter_value().double_value

        self.declare_parameter("nbr_drone", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("nbr_target", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("fast", rclpy.Parameter.Type.BOOL)
        self.nbr_drone = self.get_parameter("nbr_drone").get_parameter_value().integer_value
        self.nbr_target = self.get_parameter("nbr_target").get_parameter_value().integer_value
        self.fast_speed = self.get_parameter("fast").get_parameter_value().bool_value
        #self.fast_speed = True if self.simu_speed == 1 else False
        self.slow_speed = False
        self.drone_nodes_ok = np.ones(self.nbr_drone)
        self.target_nodes_ok = np.ones(self.nbr_target)
        self.evaluator_node_ok = np.ones(1)

        self.pub = self.create_publisher(Clock, "/clock", 0)

        # Create a service
        self.service = self.create_service(TimeStepOK, 'timestep', self.timestep_service)

        if not self.fast_speed and not self.slow_speed:
            self.clock_publish_loop()
        if self.slow_speed:
            self.user_input_publish_loop()
        else:
            self.new_timestep()
            time.sleep(2)
            thread = threading.Thread(target=self.publish_loop)
            thread.start()
            thread_watchdog = threading.Thread(target=self.watch_dog)
            thread_watchdog.start()

    def watch_dog(self):
        """
        I'm here to check if the timestep is blocked
        """
        old_sec = self.duration_sec
        old_nsec = self.duration_nsec
        while True:
            time.sleep(self.watch_dog_timeout)
            if old_sec == self.duration_sec and old_nsec == self.duration_nsec:
                # We are stuck since watch dog timeout
                print("WATCH DOG CALLED " + str(time.time()), flush=True)
                self.new_timestep()   # We keep moving, too late
            old_sec = self.duration_sec
            old_nsec = self.duration_nsec

    def timestep_service(self, request, response):
        #self.get_logger().info("Received request :%s"%request.name)
        name = request.name
        if "target" in name:
            id = int(name.replace("target", ""))
            self.target_nodes_ok[id] = 1
        if "drone" in name:
            id = int(name.replace("drone", ""))
            self.drone_nodes_ok[id] = 1
        if "evaluator" in name:
            self.evaluator_node_ok = 1
        if np.all(self.target_nodes_ok == 1) and np.all(self.drone_nodes_ok == 1)\
                and self.evaluator_node_ok == 1:
            self.target_nodes_ok[self.target_nodes_ok > 0] = 0
            self.drone_nodes_ok[self.drone_nodes_ok > 0] = 0
            self.evaluator_node_ok = 0
            if self.fast_speed:
                time.sleep(self.time_wait)
                self.new_timestep()

        response.resp = 1
        return response

    def publish_loop(self):
        #while True:
        #time.sleep(0.0001)
        #self.duration_nsec += 1
        msg = Clock()
        msg.clock.sec = self.duration_sec
        msg.clock.nanosec = self.duration_nsec
        self.pub.publish(msg)

    def new_timestep(self):
        nsec = self.duration_nsec + self.timestep_period * math.pow(10, 9)

        add_sec = int(nsec // math.pow(10, 9))
        nsec = int(nsec % math.pow(10, 9))

        self.duration_sec += add_sec
        self.duration_nsec = nsec

        self.publish_loop()

    def clock_publish_loop(self):
        t = time.time()
        msg = Duration()
        while rclpy.ok():
            while time.time() - t < 1:  # Wait until reaching one second
                time.sleep(0.01)
            self.duration_sec += 1
            msg.sec = self.duration_sec
            self.pub.publish(msg)
            t = time.time()

    '''
    def user_input_publish_loop(self):
        def next_time():
            self.duration_sec +=1
            msg = Duration()
            msg.sec = self.duration_sec
            self.pub.publish(msg)

        def on_press(key):
            try:
                if key.char == 'a':
                    next_time()
                    print('Next move')
            except AttributeError:
                print('special key pressed: {0}'.format(key))

        # Collect events until released
        with keyboard.Listener(
                on_press=on_press) as listener:
            listener.join()
    '''

def main(args=None):
    rclpy.init(args=args)

    ts = timestep()

    rclpy.spin(ts)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ts.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
