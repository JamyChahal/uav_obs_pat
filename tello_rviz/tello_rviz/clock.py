from rclpy.clock import Clock
from rosgraph_msgs.msg import Clock as Clock_msg
from rclpy.clock import ClockType
from rclpy.clock import JumpThreshold
from rclpy.clock import ROSClock
from rclpy.duration import Duration
from rclpy.time import Time
import rclpy
from rclpy.node import Node
import time

class TestClock(Node):

    def __init__(self):
        super().__init__('clock')
        print("New clock built")

        # Direct instantiation of a ROSClock is also possible.
        self.clock = ROSClock()
        assert self.clock.clock_type == ClockType.ROS_TIME

        self.clock_msg = Clock_msg()
        self.rate = 1/10 #10Hz
        self.rate_check = self.rate/10 #Check 10 times more
        #self.sim_speed_multiplier = 1 #By default
        self.declare_parameter("sim_speed", 1.0)

        self.sim_speed_multiplier = self.get_parameter("sim_speed").get_parameter_value().double_value
        print("Sim speed x"+str(self.sim_speed_multiplier))

        self.zero_time = time.time()

        self.pub = self.create_publisher(Clock_msg, "/clock", 0)

        self.publish_loop()



    def publish_loop(self):
        t = time.time()
        while rclpy.ok():
            self.clock_msg.clock = Time(seconds = (self.sim_speed_multiplier*(time.time() - self.zero_time))).to_msg()
            self.pub.publish(self.clock_msg)
            while time.time() - t < self.rate:
                time.sleep(self.rate_check)
            t = time.time()

def main(args=None):
    rclpy.init(args=args)

    clock = TestClock()

    rclpy.spin(clock)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    clock.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()