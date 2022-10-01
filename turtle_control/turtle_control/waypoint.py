from cgitb import reset
import re
from urllib import request
from enum import Enum, auto
import std_srvs
from std_srvs.srv import Empty
import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute, SetPen
from turtle_interfaces.srv import Waypoints
from geometry_msgs.msg import Twist
from random import uniform
from math import pi
import numpy as np

class State(Enum):
    """States of program"""
    MOVING = auto(),
    STOPPED = auto(),
    RESET = auto(),
    TELEPORT = auto()


class Waypoint(Node):
    def __init__(self):
        super().__init__("waypoint")
        """Initialize state to be stopping"""
        self.state = State.STOPPED
        """Declare a frequency parameter for timer callback"""
        self.declare_parameter("frequency", 0.01)
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        """Create a service to toggle states"""
        self.toggle = self.create_service(Empty, "toggle", self.toggle_callback)
        """Create a service for turtle moving to different waypoints"""
        self.load = self.create_service(Waypoints, "load", self.load_callback)
        """Timer that runs at 100 Hz"""
        self.timer = self.create_timer(self.frequency, self.timer_callback)
        """Client for resetting the turtle in turtlesim"""
        self.reset = self.create_client(Empty, "reset")
        """Client for teleporting turtle in turtlesim"""
        self.teleport = self.create_client(TeleportAbsolute, "TeleportAbsolute")
        """Client for turtlesim pen to draw an X at each waypoint"""
        self.pen = self.create_client(SetPen, "setpen")
        """Publish positions for turtle to move"""
       # self.move = self.create_publisher(Twist, "")

       # if not self.reset.wait_for_service(timeout_sec=1.0):
       #     raise RuntimeError('Timeout waiting for "spawn" service to become available')


    """Service callback function for toggling states"""
    def toggle_callback(self, request, response):
        if self.state == State.STOPPED:
            """If toggle is called, switch state from stopped to moving"""
            self.state = State.MOVING
        else:
            """If toggle is called, switch state from moving to stopped"""
            self.state = State.STOPPED
            self.get_logger().debug("Stopping")
        return response

    def load_callback(self, request, response):
        """ Callback function for the /load service
        
            Resets the turtlesim node to its initial state and creates waypoints at those locations,
            then the turtle moves to each waypoint and calculates the total linear distance traveled
        """
        self.state = State.RESET
        self.points = request.points
        self.reset_future = self.reset.call_async(Empty.Request())
        i = 0
        self.total_dist = 0.0
        while i < len(self.points):
            self.diff_x = self.points[i+1].x - self.points[i].x
            self.diff_y = self.points[i+1].y - self.points[i].y

            #teleport
            #self.teleport_future = self.teleport.call(TeleportAbsolute.Request(x = self.points[i].x, y = self.points[i].y, theta = uniform(-pi, pi)))
            #if self.teleport_future.done():
            #self.pen_future = self.pen.call(SetPen.Request(r = 255, g = 0, b = 0, width = 0.5, off = False))
            #self.x = 


            self.total_dist += np.sqrt(self.diff_x**2 + self.diff_y**2)
            i += 1

        response.total_dist = self.total_dist
        return response

    """Timer function that runs at 100 Hz"""
    def timer_callback(self):    
        if self.state == State.MOVING:
            """Send to debug logger that you are issuing a command as long as it is moving"""
            self.get_logger().debug("Issuing Command!")
            
        elif self.state == State.RESET:
            print("waiting")
            if self.reset_future.done():
                self.get_logger().info("Turtlesim Reset!")
                self.state == State.MOVING
                self.get_logger().info("Teleport turtle")
                self.state = State.TELEPORT

        #elif self.state == State.TELEPORT:
            

def main(args=None):
    """ The main() function. """
    rclpy.init(args=args)
    waypoint = Waypoint()
    rclpy.spin(waypoint)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        