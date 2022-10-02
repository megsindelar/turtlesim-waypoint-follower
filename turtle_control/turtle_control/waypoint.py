from cgitb import reset
import re
from telnetlib import TELNET_PORT
from urllib import request
from enum import Enum, auto
import std_srvs
from std_srvs.srv import Empty
import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute, SetPen
from turtle_interfaces.srv import Waypoints
from geometry_msgs.msg import Twist, Vector3
from turtlesim.msg import Pose
from random import uniform
from math import pi
import numpy as np
from time import sleep

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
        """Declare a tolerance parameter"""
        self.declare_parameter("tolerance",0.05)
        self.tolerance = self.get_parameter("tolerance").get_parameter_value().double_value
        """Create a service to toggle states"""
        self.toggle = self.create_service(Empty, "toggle", self.toggle_callback)
        """Create a service for turtle moving to different waypoints"""
        self.load = self.create_service(Waypoints, "load", self.load_callback)
        """Timer that runs at 100 Hz"""
        self.timer = self.create_timer(self.frequency, self.timer_callback)
        """Client for resetting the turtle in turtlesim"""
        self.reset = self.create_client(Empty, "reset")
        """Client for teleporting turtle in turtlesim"""
        self.teleport = self.create_client(TeleportAbsolute, "turtle1/teleport_absolute")
        """Client for turtlesim pen to draw an X at each waypoint"""
        self.pen = self.create_client(SetPen, "turtle1/set_pen")
        """Publish positions for turtle to move"""
        self.pub = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        """Subscriber to current turtle position"""
        self.sub = self.create_subscription(Pose, "turtle1/pose", self.turtle_pose_callback, 10)
        self.count = 0
        """Count variable for keeping track of indices for moving turtle"""
        self.count2 = 1


    def turtle_pose_callback(self, msg):    #referenced http://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal to create subsriber callback
        self.pose = msg
        self.get_logger().debug(f"Subsciber data: {self.pose}")

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
        self.reset_future = self.reset.call_async(Empty.Request())

        self.points = request.points
        i = 0
        self.total_dist = 0.0
        print("waypoints")
        while i < (len(self.points))-1:
            self.diff_x = self.points[i+1].x - self.points[i].x
            self.diff_y = self.points[i+1].y - self.points[i].y
            #teleport

            self.total_dist += np.sqrt(self.diff_x**2 + self.diff_y**2)
            i += 1

        response.total_dist = self.total_dist
        return response


    def draw_x(self,i):
        #while i < len(self.points):
        self.pen_future = self.pen.call_async(SetPen.Request(off = 1))
        self.teleport_future = self.teleport.call_async(TeleportAbsolute.Request(x = self.points[i].x, y = self.points[i].y, theta = uniform(-pi, pi)))
        print("hello")
        #draw x
        self.pen_future = self.pen.call_async(SetPen.Request(r = 255, g = 170, b = 175, width = 3, off = 0))
        self.teleport_future = self.teleport.call_async(TeleportAbsolute.Request(x = (self.points[i].x + 0.25), y = (self.points[i].y + 0.25)))
        self.teleport_future = self.teleport.call_async(TeleportAbsolute.Request(x = (self.points[i].x - 0.25), y = (self.points[i].y - 0.25)))
        self.teleport_future = self.teleport.call_async(TeleportAbsolute.Request(x = (self.points[i].x), y = (self.points[i].y)))
        self.teleport_future = self.teleport.call_async(TeleportAbsolute.Request(x = (self.points[i].x + 0.25), y = (self.points[i].y - 0.25)))
        self.teleport_future = self.teleport.call_async(TeleportAbsolute.Request(x = (self.points[i].x - 0.25), y = (self.points[i].y + 0.25)))
        self.teleport_future = self.teleport.call_async(TeleportAbsolute.Request(x = (self.points[i].x), y = (self.points[i].y)))
        
        i += 1
        sleep(1)
        
        
    def move_turtle(self):
        self.pen_future = self.pen.call_async(SetPen.Request(r = 255, g = 255, b = 255, width = 3, off = 0))
        print(self.pose)
        self.dist = np.sqrt((self.points[self.count2].x - self.pose.x)**2 + (self.points[self.count2].y - self.pose.y)**2)
        if (self.dist > self.tolerance):
            self.count2 = self.count2
        elif self.count2 == (len(self.points) - 1):
            self.count2 = 0
        else:
            self.count2 += 1

        print(f"count: ", self.count2)

        self.dx = self.points[self.count2].x - self.pose.x
        self.dy = self.points[self.count2].y - self.pose.y
        print(f"dx: {self.dx}")
        print(f"dy: {self.dy}")
        
        self.dtheta = np.arctan2(self.dy, self.dx)
        print(f"dtheta: {self.dtheta}")

        print(f"pose_theta: {self.pose.theta}")
        self.theta_diff = abs(self.pose.theta - self.dtheta)
        print(f"diff {self.theta_diff}")

        if self.pose.theta < self.dtheta and self.theta_diff > 0.1:
            move = Twist(linear = Vector3(x = 0.0, y = 0.0, z = 0.0),
                    angular = Vector3(x = 0.0, y = 0.0, z = 1.0))
        elif self.pose.theta < self.dtheta and self.theta_diff < 0.1:
            move = Twist(linear = Vector3(x = 2.0, y = 0.0, z = 0.0),
                    angular = Vector3(x = 0.0, y = 0.0, z = 1.5))
        elif self.pose.theta > self.dtheta and self.theta_diff > 0.1:
            move = Twist(linear = Vector3(x = 0.0, y = 0.0, z = 0.0),
                    angular = Vector3(x = 0.0, y = 0.0, z = -1.0))
        elif self.pose.theta > self.dtheta and self.theta_diff < 0.1:
            move = Twist(linear = Vector3(x = 2.0, y = 0.0, z = 0.0),
                    angular = Vector3(x = 0.0, y = 0.0, z = -1.5))
        self.pub.publish(move)

    def turtle_firstpt(self):
        self.pen_future = self.pen.call_async(SetPen.Request(off = 1))
        self.teleport_future = self.teleport.call_async(TeleportAbsolute.Request(x = self.points[0].x, y = self.points[0].y, theta = uniform(-pi, pi)))


    """Timer function that runs at 100 Hz"""
    def timer_callback(self):    
        if self.state == State.MOVING:
            """Send to debug logger that you are issuing a command as long as it is moving"""
            try:
                self.get_logger().debug("Issuing Command!")
                self.move_turtle()
            except:
                self.get_logger().debug("Need to input waypoints first! Try again")
                self.state = State.STOPPED
        
        elif self.state == State.RESET:
            if self.reset_future.done():
                self.get_logger().info("Turtlesim Reset!")
                if len(self.points) >= 1:
                    print(self.points)
                    if self.count < len(self.points):
                        self.draw_x(self.count)
                        self.count += 1
                        #sleep(2)
                    else:
                        print("teleport")
                        self.state = State.TELEPORT
                else:
                    self.get_logger().debug("No waypoints! Try again!")
                    self.state = State.STOPPED

        elif self.state == State.TELEPORT:
            print("hi")
            self.turtle_firstpt()
            #if self.teleport_future.done():
            #    self.state = State.MOVING
            self.state = State.STOPPED
            print(self.state)
            

def main(args=None):
    """ The main() function. """
    rclpy.init(args=args)
    waypoint = Waypoint()
    rclpy.spin(waypoint)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        