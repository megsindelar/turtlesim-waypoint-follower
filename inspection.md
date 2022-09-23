# CRAZY TURTLE
Demonstration package for ME495.
This README is intentionally vague.
Figuring out how this package works and filling in the details is part of the
exercise. Replace the blanks marked with `${ITEM}` with your answer.
Unless otherwise specified, list the command and all arguments that you passed to it.

## Repository Configuration
1. The `crazy_turtle` git repository consists of the ROS 2 packages `crazy_turtle` and `crazy_turtle_interfaces`
2. The package crazy_turtle is a `ament_python` package because `this package is only implementing python code.`
2. The package crazy_turtle_interfaces is a `ament_cmake` package because `this package is only implementing C/C++ code.`


## Setup Instructions
1. Build the workspace using `colcon build --symlink-install` so that it is unnecessary to rebuild when python files change.
2. Initialize the ROS environment (i.e., set the necessary ROS environment variables) by executing:
      `echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc`
      `. install/local_setup.bash`
3. Make sure no other ROS nodes are running prior to starting.
3. Run the launchfile `go_crazy_turtle.launch.xml` by executing `ros2 launch crazy_turtle go_crazy_turtle.launch.xml`
4. When running you can see a visual depiction of the ROS graph using the ros2 run rqt_graph rqt_graph command.
   The ROS graph, including all topics and node labels, looks like:
   
   ![The ROS Graph](https://github.com/ME495-EmbeddedSystems/homework1-megsindelar/blob/main/rosgraph_part1_setup.svg)

## Runtime Information
The `launchfile` from above should be running at all times when executing these commands.
If the nodes launched from the `launchfile` are not running, you will get incorrect results.

5. Use the ROS command `ros2 node list` to list all the nodes that are running.
   The output of the command looks like
   ```
   /mover
   /roving_turtle
   ```
6. Use the ROS command `ros2 topic list` to list the topics
   The output of the command looks like
   ```
   /parameter_events
   /rosout
   /turtle1/cmd_vel
   /turtle1/color_sensor
   /turtle1/pose

   ```

7. Use the ROS command `ros2 topic hz /turtle1/cmd_vel` to verify that the frequency of
   the `/turtle1/cmd_vel` topic is `119.923 Hz`

8. Use the ROS command `ros2 service list` to list the services.
   The output of the command looks like
   ```
   /clear
   /kill
   /mover/describe_parameters
   /mover/get_parameter_types
   /mover/get_parameters
   /mover/list_parameters
   /mover/set_parameters
   /mover/set_parameters_atomically
   /reset
   /roving_turtle/describe_parameters
   /roving_turtle/get_parameter_types
   /roving_turtle/get_parameters
   /roving_turtle/list_parameters
   /roving_turtle/set_parameters
   /roving_turtle/set_parameters_atomically
   /spawn
   /switch
   /turtle1/set_pen
   /turtle1/teleport_absolute
   /turtle1/teleport_relative
   ```

9. Use the ROS command `ros2 service type /switch` to determine the type of the `/switch` service, which is `crazy_turtle_interfaces/srv/Switch`.

10. Use the ROS command `ros2 param list` to list the parameters of all running nodes
    ```
    /mover:
      use_sim_time
      velocity
   /roving_turtle:
      background_b
      background_g
      background_r
      qos_overrides./parameter_events.publisher.depth
      qos_overrides./parameter_events.publisher.durability
      qos_overrides./parameter_events.publisher.history
      qos_overrides./parameter_events.publisher.reliability
      use_sim_time

    ```

11. Use the ROS command `ros2 param describe /mover velocity` to get information about the `/mover` `velocity` parameter, including its type, description, and constraints
    ```
    Parameter name: velocity
      Type: double
      Description: The velocity of the turtle
      Constraints:

    ```

12. Use the ROS command `ros2 interface proto crazy_turtle_interfaces/srv/Switch` to retrieve a template/prototype for entering parameters for the `/switch` service on the command line.
    ```
   "mixer:
      x: 0.0
      y: 0.0
      theta: 0.0
      linear_velocity: 0.0
      angular_velocity: 0.0
   "
    ```

## Package Exploration
1. Use the ROS command `ros2 interface package crazy_turtle_interfaces` to list the interface types defined by `crazy_turtle_interfaces`
   The output of the command looks like
   ```
   crazy_turtle_interfaces/srv/Switch
   ```
2. Use the ROS command `ros2 pkg executables crazy_turtle` to list the executables included with the `crazy_turtle` package
   The output of the command looks like
   ```
   crazy_turtle mover
   ```

## Live Interaction
1. Use the command `ros2 param get /mover velocity` to retrieve the value of the `/mover velocity` parameter, which is `4.5`.
2. The ROS command to call the `/switch` service, and it's output is listed below:
    ```
    COMMAND:

    ros2 service call /switch crazy_turtle_interfaces/srv/Switch "mixer:
      x: 1.0
      y: 2.0
      theta: 0.0
      linear_velocity: 4.0
      angular_velocity: 3.0" 
    
    

    OUTPUT:

    requester: making request: crazy_turtle_interfaces.srv.Switch_Request(mixer=turtlesim.msg.Pose(x=1.0, y=2.0, theta=0.0, linear_velocity=4.0, angular_velocity=3.0))

    response:
    crazy_turtle_interfaces.srv.Switch_Response(x=5.0, y=4.0)
    ```
3. The `switch` service performs the following actions (in sequence):
    1. It `deletes` the current turtle
    2. It then respawns a new turtle at `(x=(y+angular_velocity), y=(x*linear_velocity)`
4. What happens to the turtle's motion if you use `ros2 param set /mover velocity 10` to change `/mover velocity` to 10? same
5. Use the Linux command `${command and args}` to kill the `/mover` node.
6. Use the ROS command `${command and args}` to start the `/mover` node with a velocity of 10. 
    - Be sure to remap `cmd_vel` to `/turtle1/cmd_vel`.
7. What happened to the turtle's velocity after relaunching `mover`? `${faster | slower | same}`
