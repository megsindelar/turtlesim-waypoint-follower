import re
from urllib import request
from enum import Enum, auto
import std_srvs
from std_srvs.srv import Empty
import rclpy
from rclpy.node import Node

class State(Enum):
    """States of program"""
    MOVING = auto(),
    STOPPING = auto()

class Waypoint(Node):
    def __init__(self):
        super().__init__("waypoint")
        """Timer that runs at 100 Hz"""
        self.state = State.STOPPING
        self.stopped = 1
        self.declare_parameter("frequency", 0.01)
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        self.toggle = self.create_service(Empty, "toggle", self.toggle_callback)
        self.timer = self.create_timer(self.frequency, self.timer_callback)
        #print(f'State: {self.state}')
    
    def timer_callback(self):    
        if self.state == State.MOVING:
            self.get_logger().debug("Issuing Command!")
            self.stopped = 0
        elif (self.state == State.STOPPING) and self.stopped == 0:
            self.get_logger().debug("Stopping")
            self.stopped = 1
        else:
            self.get_logger().debug("")
        print(f'State: {self.state}')

    def toggle_callback(self, request, response):
        if self.state == State.STOPPING:
            self.state = State.MOVING
        else:
            self.state = State.STOPPING
        return response

def main(args=None):
    """ The main() function. """
    rclpy.init(args=args)
    waypoint = Waypoint()
    rclpy.spin(waypoint)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        