# ROS_vehicle
ENPM662 - Introduction to Robot Modeling Project 1 CAD Modeling &amp; Simulation using Gazebo

## Key Contributions of Each Member

Jonathan - Created CAD models of the chassis, wheel shafts, steering links, and assembled all the parts into the final truck & trailer model. Kept track of progress and testing of every step of the project. Helped fix wheel alignment issues. Wrote most of the report, Created & organized github repository. 

Kent- Made CAD models of the truck & lidar mount. Helped identify & fix wheel misalignments. Constructed a proper URDF from Solidworks, and made the initial ros package with it showing straight truck motion in gazebo.

Robens- Built some of the CAD file of the trailer & hitch. Built upon initial ros package by adding Lidar Sensor & controllers, coding of teleop script and testing it on the competition track.

Hamsa - Created CAD files for rims & tires and trailer and coded the close loop controller script.

## Steps to build and run project.
Build and source odometry and project1_pkg:

`colcon build --packages-select project1_pkg`

`colcon build --packages-select odometry`

`source install/setup.bash`

Odometry is needed to track robot position and orientation

Then launch package using command 

`ros2 launch project1_pkg gazebo.launch.py`

This will display an empty world with the robot at position (0,0)

## RVIZ

To launch rviz
open new terminal and run: 

`ros2 launch project1_pkg display.launch.py`

## Closed Loop Controller
To launch the closed loop controller:
Open new terminal and run:

`ros2 launch project1_pkg autonomous_control.py`

The robot will now move from (0,0) to (10, 10) in a straight line

To launch subscriber to monitor robot position:
Open new terminal and run:

`ros2 launch project1_pkg odom_subscriber.py`

## Teleop:
There are two maps for teleop: competition_arena and competition_track. 

By default our package opens with the empty world. To launch the competition arena,
open the gazebo.launch.py in project1_pkg/launch. Uncomment the map for teleop section
in line 24 to 29. And comment out the empty world section from line 17 to line 22.

Close any previously running gazebo terminals then
Build and source project1_pkg then launch gazebo with the command:

`colcon build --packages-select project1_pkg`

`source install/setup.bash`

`ros2 launch project1_pkg gazebo.launch.py`

Open new terminal and run:

`ros2 launch project1_pkg teleop.py`

Robot can now be controlled with W, A, S, D with W and S increasing and decreasing velocity 
respectively and A and D turning the wheel left and right respectively. 

To run the competition track, first close any running gazebo terminal then navigate to 
spawn_robot_ros2.launch.py in project1_pkg/launch. We need change the position so that the robot
spawns at the checkpoint on the track. Uncomment the position and orientation for competition lines
then comment the position and orientation for autonomous mode lines. 

Save, build, and source project1_pkg. 

`colcon build --packages-select project1_pkg`

`source install/setup.bash`

The competition track can then be ran with the command:

`ros2 launch project1_pkg competition.launch.py`

Then open new terminal and run:

`ros2 launch project1_pkg teleop.py`

You should now be able to control the robot around the track with W, A, S, D
