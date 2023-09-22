# Mobile Manipulator Motion Planning

This repository contains ROS packages for a mobile manipulator industrial use case. The project was developed as part of a lecture in the 2nd Semester of the Master's Degree Programm Robotics Engineering at the UAS Technikum Wien.

The repository contains the following packages:
- [butler_robot](butler_robot)
    
    The butler_robot contains all the necessary urdf files and bringup files to launch the butler_robot, a mobile manipulator consisting of a [MIR100](https://www.mobile-industrial-robots.com/solutions/robots/mir100/) base and a [UR5](https://www.universal-robots.com/products/ur5-robot/) manipulator, into a gazebo environment.

    <br></br>
- [tdf_gazebo](tdf_gazebo)
    
    The tdf_gazebo package contains the necessary world, objects, and maps to launch a gazebo simulation of the [Technikum Digital Factory](https://academy.technikum-wien.at/digitalefabrik/).

    <br></br>
- [tdf_simulation](tdf_simulation)

    The tdf_simulation package contains the ROS nodes for image processing, as well as, the robot control node to simulate the industrial use case.

    <br></br>
- [ur_butler_moveit](ur_butler_moveit)

    The ur_butler_moveit package contains the config and launch files for the [MoveIt](https://moveit.ros.org/) setup to control the UR5 manipulator. The package was automatically generated using the MoveIt Setup Assistant. 

    <br></br>

## Requirements

- ROS general packages

    The simulation requires some basic ROS packages to be installed for the robots and the control of moveit and move_base.
    ```BASH
    sudo apt-get install -y ros-noetic-navigation       # for move base
    sudo apt-get install -y ros-noetic-moveit*          # for moveit
    sudo apt-get install -y ros-noetic-universal-robots # for the UR5
    sudo apt-get install -y ros-noetic-mir-robot        # for the MIR100
    sudo apt-get install -y ros-noetic-cv-bridge        # for image conversion
    ```

- IRA Laser Tools

    Merging the two LiDAR sensors of the MIR100 together is done using the [IRA Laser Tools](http://wiki.ros.org/ira_laser_tools)

- STOMP Motion Planner

    In order to use the STOMP motion planner the Prerequisits and required packages have to be installed as described [here](https://ros-planning.github.io/moveit_tutorials/doc/stomp_planner/stomp_planner_tutorial.html#prerequisites).

- OpenCV
    For image processing OpenCV is required which can be installed as described [here](https://www.geeksforgeeks.org/how-to-install-opencv-in-c-on-linux/).

<br></br>

## Usage

First to simulate the robot control, the absolute path in the [control_node](tdf_simulation/src/control_node.cpp#L99) has to be adapted to the absolute path of the [butler moveit urdf file](ur_butler_moveit/config//gazebo_butler_robot.urdf).

In order to simulate the industrial use case the two launch files
- [tdf_butler_control](tdf_gazebo/launch/tdf_butler_control.launch)
- [spz_sim](tdf_simulation/launch/spz_sim.launch)

have to be launched in the given order by running these commands:
```BASH
roslaunch tdf_gazebo tdf_butler_control.launch
roslaunch tdf_simulation spz_sim.launch
```

The motion planning algorithm used for trajectory planning can be edited in the [tdf_butler_control](tdf_gazebo/launch/tdf_butler_control.launch) launch file as followed:

- RRT Connect

    Set the argument `motion_planner_pipeline` at the top of the launch file to `"ompl"`.
    Set the default ompl planner to `"RRTConnect"` in the [ompl_planning](ur_butler_moveit/config/ompl_planning.yaml#L168) config file.

- PRM

    Set the argument `motion_planner_pipeline` at the top of the launch file to `"ompl"`.
    Set the default ompl planner to `"PRM"` in the [ompl_planning](ur_butler_moveit/config/ompl_planning.yaml#L168) config file.

- STOMP

    Set the argument `motion_planner_pipeline` at the top of the launch file to `"stomp"`.

- CHOMP

    Set the argument `motion_planner_pipeline` at the top of the launch file to `"chomp"`.
