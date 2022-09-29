import re
from urllib import request
import std_srvs
from std_srvs.srv import Empty
import rclpy
from rclpy.node import Node

#class State(Enum):

class Waypoint(Node):
    def __init__(self):
        super().__init__("waypoint")
        """Timer that runs at 100 Hz"""
        self.declare_parameter("frequency", 0.01)
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
       # self.toggle = self.create_service(Empty, "toggle", self.toggle_callback)
        self.timer = self.create_timer(self.frequency, self.timer_callback)
    
    def timer_callback(self):    
        self.get_logger().debug("Issuing Command!")


        #def toggle_callback(self, request, response):
            # if request.toggle == 0:
            #      self.get_logger().debug("Stopping.")
                    #!!!!!!!!!!!once!!!!!!!!!! command
                #stopped = 1   OR is there a way to check last debug log (start with prev log)
                #if stopped == 1, don't print anything
            # else:
            #      self.get_logger().debug("Issuing Command!")
                #stopped = 0

    # def toggle_callback(self, request, response):
    #     def check_new_request(self):
    #         if request.toggle != response.toggle_past:
    #             new_request = 1
    #         else:
    #             new_request = 0
    #         return new_request

    #     if request.toggle == 0:
    #         self.get_logger().debug("Stopping.")
    #     while request.toggle == 1 and not new_request:
    #         self.get_logger().debug("Issuing Command!")
    #         new_request = check_new_request()

    #     response.toggle_past = request.toggle
    #     return response

def main(args=None):
    """ The main() function. """
    rclpy.init(args=args)
    waypoint = Waypoint()
    #waypoint.stop = 1
    #waypoint.new_request = 0
    #waypoint.toggle_past = 0
    rclpy.spin(waypoint)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        