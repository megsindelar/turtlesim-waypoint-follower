import re
from urllib import request
from enum import Enum, auto
import std_srvs
from std_srvs.srv import Empty
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill, TeleportAbsolute
from turtle_interfaces.srv import Waypoints

class State(Enum):
    """States of program"""
    MOVING = auto(),
    STOPPED = auto()

class Turtle(Enum):
    """States for turtlesim reset and turtle teleporting"""
    RESET = auto(),
    TELEPORT = auto()

class Waypoint(Node):
    def __init__(self):
        super().__init__("waypoint")
        """Initialize state to be stopping"""
        self.state = State.STOPPED
        self.stopped = 1
        self.turtle = Turtle.SPAWNING
        """Declare a frequency parameter for timer callback"""
        self.declare_parameter("frequency", 0.01)
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        """Create a service to toggle states"""
        self.toggle = self.create_service(Empty, "toggle", self.toggle_callback)
        """Create a service for turtle moving to different waypoints"""
        self.load = self.create_service(Waypoints, "load", self.load_callback)
        """Timer that runs at 100 Hz"""
        self.timer = self.create_timer(self.frequency, self.timer_callback)
        """Create a client to spawn turtle in new location"""
        self.spawn = self.create_client(Spawn, "spawn")
        """Create a client to kill a turtle"""
        self.kill = self.create_client(Kill, "kill")
        """Client for resetting the turtle in turtlesim"""
        self.reset = self.create_client(Empty, "reset")
        """Client for teleporting turtle in turtlesim"""
        self.teleport = self.create_client(TeleportAbsolute, "teleport")

        if not self.kill.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('Timeout waiting for "kill" service to become available')

        if not self.spawn.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('Timeout waiting for "spawn" service to become available')

        if not self.reset.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('Timeout waiting for "spawn" service to become available')

    
    """Timer function that runs at 100 Hz"""
    def timer_callback(self):    
        if self.state == State.MOVING:
            """Send to debug logger that you are issuing a command as long as it is moving"""
            self.get_logger().debug("Issuing Command!")
            self.stopped = 0
        elif (self.state == State.STOPPED) and self.stopped == 0:
            """When stopping, tell the debug logger once"""
            self.get_logger().debug("Stopping")
            self.stopped = 1
        else:
            """If already stopped and told debug logger, then print nothing"""
            self.get_logger().debug("")
        print(f'State: {self.state}')

    """Service callback function for toggling states"""
    def toggle_callback(self, request, response):
        if self.state == State.STOPPED:
            """If toggle is called, switch state from stopped to moving"""
            self.state = State.MOVING
        else:
            """If toggle is called, switch state from moving to stopped"""
            self.state = State.STOPPED
        return response

    def reset_turtlesim(self):
        self.reset_future = self.reset.call_async()

    def teleport(self):
        self.teleport_future = 

    def load_callback(self, request, response):
        """ Callback function for the /load service
        
            Resets the turtlesim node to its initial state and creates waypoints at those locations,
            then the turtle moves to each waypoint and calculates the total linear distance traveled
        """

        if self.state == State.STOPPED:
            self.reset_turtlesim()
            self.state = State.MOVING
        else:
            


        # self.get_logger().info("Killing turtle1")
        # self.kill_future = self.kill.call_async(Kill.Request(name="turtle1"))
        # self.turtle = Turtle.KILLING

        # #Calculate a new random position for the respawned turtle
        # self.x_new = request.mixer.y + request.mixer.angular_velocity
        # self.y_new = request.mixer.x * request.mixer.linear_velocity

        # response.x = self.x_new
        # response.y = self.y_new

        return response

def main(args=None):
    """ The main() function. """
    rclpy.init(args=args)
    waypoint = Waypoint()
    rclpy.spin(waypoint)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        