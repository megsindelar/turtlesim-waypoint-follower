# ME495 Embedded Systems Homework 1
# Author: Megan Sindelar

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
    """ Different possible states of the program
        Purpose: determines what functions get called in the main timer
    """
    MOVING = auto(),
    STOPPED = auto(),
    RESET = auto(),
    TELEPORT = auto()


class Waypoint(Node):
    """ Publishes turtle1/cmd_vel (a movement command with type geometry_msgs/Twist) at a fixed rate
        Subscribes to turtle1/pose (the current turtle position with type turtlesim.msg/Pose) at a fixed rate
        Services:
            toggle: service created to toggle turtle moving states from stopping to moving
            load: service created to load user input waypoints to generate a turtle path
        Clients:
            reset: reset the turtlesim node back to its initial state (type is std_srvs.srv/Empty)
            teleport_absolute: teleport the turtle to an absolute location (type is turtlesim.srv/TeleportAbsolute)
            set_pen: turn on and off the pen of the turtle, which draws a path wherever the turtle moves (type is turtlesim.srv/SetPen)
        Parameters:
            frequency: set the frequency of the timer (default is 100 Hz)
            tolerance: set the tolerance of how close the turtle can get to the waypoint before being "at" the waypoint (default is 0.05)
    """
    def __init__(self):
        super().__init__("waypoint")
        #Initialize state to be stopping
        self.state = State.STOPPED
        #Declare a frequency parameter for timer callback
        self.declare_parameter("frequency", 100.0)
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        #Declare a tolerance parameter
        self.declare_parameter("tolerance",0.05)
        self.tolerance = self.get_parameter("tolerance").get_parameter_value().double_value
        #Create a service to toggle states
        self.toggle = self.create_service(Empty, "toggle", self.toggle_callback)
        #Create a service for turtle moving to different waypoints
        self.load = self.create_service(Waypoints, "load", self.load_callback)
        #Timer that runs at 100 Hz
        self.timer = self.create_timer((1/self.frequency), self.timer_callback)
        #Client for resetting the turtle in turtlesim
        self.reset = self.create_client(Empty, "reset")
        #Client for teleporting turtle in turtlesim
        self.teleport = self.create_client(TeleportAbsolute, "turtle1/teleport_absolute")
        #Client for turtlesim pen to draw an X at each waypoint
        self.pen = self.create_client(SetPen, "turtle1/set_pen")
        #Publish positions for turtle to move
        self.pub = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        #Subscriber to current turtle position
        self.sub = self.create_subscription(Pose, "turtle1/pose", self.turtle_pose_callback, 10)
        #Count used for drawing waypoints
        self.count = 0
        #Count used to keep track of which waypoint to move turtle to
        self.count2 = 1


    def turtle_pose_callback(self, msg):   
        """ Subscriber callback function for turtle1/pose subscriber
            Receives turtle position data and logs it

            Args:
                msg: the data from the subscribed topic turtle1/pose

            Returns:
                no returns

            referenced http://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal to create subsriber callback
        """ 
        self.pose = msg
        self.get_logger().debug(f"Subsciber data: {self.pose}")


    def toggle_callback(self, request, response):
        """ Callback function for /toggle service

            Toggles between turtle moving and stopping states

            Args:
                request (EmptyRequest): no data
             
                response (EmptyResponse): no data

            Returns: 
                An EmptyResponse, contains no data
        """
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
        
            Resets the turtlesim node to its initial state and creates waypoints at each location
            specified by the user

            Args:
                request (WaypointsRequest): the points field has x, y, 
                                            linear velocity, and angular 
                                            velocity components
                                            (the waypoints specified by
                                            the user)

                response (WaypointsResponse): the total_dist response
                                              (returns the total distance
                                              that the turtle traveled
                                              for one path)
            Returns:
                A WaypointsResponse, containing the total distance the turtle travels in one loop
        """
        self.state = State.RESET
        self.reset_future = self.reset.call_async(Empty.Request())

        self.points = request.points
        i = 0
        self.total_dist = 0.0
        while i < (len(self.points))-1:
            self.diff_x = self.points[i+1].x - self.points[i].x
            self.diff_y = self.points[i+1].y - self.points[i].y

            self.total_dist += np.sqrt(self.diff_x**2 + self.diff_y**2)
            i += 1

        response.total_dist = self.total_dist

        return response


    def draw_x(self,i):
        """ Function to draw waypoint X's, specified by the user input for the /load service
            Calls the TeleportAbsolute and SetPen clients to teleport to different waypoints and draw X's at each waypoint location

            Args:
                index of the dictionary of waypoints specified by the user (the input is the self.count variable)

            Returns:
                no returns
        """
        self.pen_future = self.pen.call_async(SetPen.Request(off = 1))
        self.teleport_future = self.teleport.call_async(TeleportAbsolute.Request(x = self.points[i].x, y = self.points[i].y, theta = uniform(-pi, pi)))

        #draw waypoint
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
        """ Function to move the turtle in a looping path to each waypoint
            Publishes turtle1/cmd_vel (type geometry_msgs/Twist) to move the turtle

            Args:
                no arguments

            Returns:
                no returns
        """
        self.pen_future = self.pen.call_async(SetPen.Request(r = 255, g = 255, b = 255, width = 3, off = 0))
        self.dist = np.sqrt((self.points[self.count2].x - self.pose.x)**2 + (self.points[self.count2].y - self.pose.y)**2)
        if (self.dist > self.tolerance):
            self.count2 = self.count2
        elif self.count2 == (len(self.points) - 1):
            self.count2 = 0
        else:
            self.count2 += 1

        self.dx = self.points[self.count2].x - self.pose.x
        self.dy = self.points[self.count2].y - self.pose.y
        
        self.dtheta = np.arctan2(self.dy, self.dx)
        self.theta_diff = abs(self.pose.theta - self.dtheta)

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
        """ Function to teleport turtle back to the first waypoint after drawing all of the waypoints
            
            Args:
                no arguments
            
            Returns:
                no returns
        """
        self.pen_future = self.pen.call_async(SetPen.Request(off = 1))
        self.teleport_future = self.teleport.call_async(TeleportAbsolute.Request(x = self.points[0].x, y = self.points[0].y, theta = uniform(-pi, pi)))


    def timer_callback(self):
        """ Timer callback function that runs at a specified frequency (default is 100 Hz)
            Publishes turtle1/cmd_vel (type geometry_msgs/Twist) to stop turtle when system state is "STOPPED"

            Args:
                no arguments

            Returns:
                no returns
        """    
        if self.state == State.MOVING:
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
                    if self.count < len(self.points):
                        self.draw_x(self.count)
                        self.count += 1
                    else:
                        self.state = State.TELEPORT
                else:
                    self.get_logger().debug("No waypoints! Try again!")
                    self.state = State.STOPPED

        elif self.state == State.TELEPORT:
            self.turtle_firstpt()
            self.state = State.STOPPED
        
        elif self.state == State.STOPPED:
            move = Twist(linear = Vector3(x = 0.0, y = 0.0, z = 0.0),
                    angular = Vector3(x = 0.0, y = 0.0, z = 0.0))
            self.pub.publish(move)


def main(args=None):
    """ The main function"""
    rclpy.init(args=args)
    waypoint = Waypoint()
    rclpy.spin(waypoint)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        