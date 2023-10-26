from rclpy.node import Node

from tello_msgs.srv import TelloAction

class ActionManager(Node):
    def __init__(self):
        super().__init__('controller_manager')

        self.declare_parameter('namespace', 'drone1')
        namespace = self.get_parameter('namespace').value

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