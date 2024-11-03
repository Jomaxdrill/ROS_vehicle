# ROS_vehicle
ENPM662 - Introduction to Robot Modeling Project 1 CAD Modeling &amp; Simulation using Gazebo

The following project made use of the CAD software Solidworks and software framework ROS for robotics to simulate the following vehicle in a 3d World space. 

![Screenshot 2024-10-31 163808](https://github.com/user-attachments/assets/8b9e9bfb-49f3-466e-ac1b-93e1b9a83171)

## Key Contributions of Each Member

Jonathan - Created CAD models of the chassis, wheel shafts, steering links, and assembled all the parts into the final truck & trailer model. Kept track of progress and testing of every step of the project. Helped fix wheel alignment issues. Wrote most of the report, Created & organized github repository. 

Kent- Made CAD models of the truck & lidar mount. Helped identify & fix wheel misalignments. Constructed a proper URDF from Solidworks, and made the initial ros package with it showing straight truck motion in gazebo.

Robens- Built some of the CAD file of the trailer & hitch. Built upon initial ros package by adding Lidar Sensor & controllers, coding of teleop script and testing it on the competition track.

Hamsa - Created CAD files for rims & tires and trailer and coded the close loop controller script.
## GENERAL SETUP
-Verify you have ROS2 galatic distribution installed and also CMAKE necessary installations. Refer to: https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html
On command line run:
```sh
echo $ROS_DISTRO
```
-Install previously the following packages and any additional in your linux distribution running on the terminal the command:
```sh 
sudo apt install python3-colcon-common-extensions
```
-Install all necessary dependencies and libraries with pip for insrtance. Recommended to run in your own python environment.

## Steps to build and run project.

### Create the workspace and install all dependencies priorly

Copy the folder workspace **ROS_SRC_PKG** in your linux system, locate them via the terminal using cd syntax and run 
```sh
cd ~\PATH TO ROS_SRC_PKG
rosdep install -i --from-path src --rosdistro galactic -y
```
![image](https://github.com/user-attachments/assets/d473813b-e6a5-4e99-9526-162e8f6c44b9)

Build and source odometry and project1_pkg:

```sh
colcon build --packages-select project1_pkg
```
project1_pkg contains the scripts for do teleoperation and for establish trajectories to follow with
In case you got the following error create the include folder as follows 

![image](https://github.com/user-attachments/assets/08e6578f-bfaf-40e6-b556-0381eb981d41)

```sh
mkdir -p ~/src/project1_pkg/include/project1_pkg
```
And run again the build command

```sh
colcon build --packages-select odometry
```
Odometry package is needed to track robot position and orientation as it contains plugin features.

Source ROS (Package will be identified) However you can do make this default when opening the terminal by modifying the .bashrc file. <ins>Don't forget your user password to give permissions </ins>
```sh
sudo nano ~/.bashrc
```
![image](https://github.com/user-attachments/assets/56625fea-d3f4-4354-8d2e-7433444ea24b)

```sh
 source /opt/ros/galactic/setup.bash
```
**Source package to be identified**. Do this for every new terminal you open locating the Workspace folder:

```sh
source install/setup.bash
```
![image](https://github.com/user-attachments/assets/a2567ff8-e8f2-4170-8662-526f9e996bc6)


Before launch also run the following commands to install the controller dependencies:

```sh
sudo apt install ros-galactic-ros2-control ros-galactic-ros2-controllers ros-galactic-gazebo-ros2-control
sudo apt-get install ros-galactic-controller-manager
```
### Gazebo

Then launch package using command 

```sh
ros2 launch project1_pkg gazebo.launch.py
```

This will display an empty world with the robot at position (0,0,0)

![Screenshot 2024-11-02 005130](https://github.com/user-attachments/assets/a49a3164-baac-4654-b04e-39a9e7d9c5c8)

### RVIZ

To launch rviz open a new terminal, source the package and run: 

```sh
ros2 launch project1_pkg display.launch.py
```
![image](https://github.com/user-attachments/assets/e9b2be9f-8e9b-4574-9cf5-dc51c4454369)

To visualize lidar scanner run before in other terminal the gazebo simulation and after launch RVIZ again and use the ** Add** button and look for select the ** laser scanner** plugin 

![Screenshot 2024-11-03 003739](https://github.com/user-attachments/assets/a8808ea6-1277-43b1-a6c6-f9343e9e5ea2)

After check for the scan topic and it should be visible the lidar.

![image](https://github.com/user-attachments/assets/06a107e9-b038-4e6e-a073-657293a8ef4a)


## Closed Loop Controller
To launch the closed loop controller:

Be sure to open the gazebo empty world by following the previous steps to open it. 

Open new terminal,source package and run:

```sh 
ros2 run project1_pkg autonomous_control.py
```

The robot will now move from (0,0) to (10, 10) in a straight line

![image](https://github.com/user-attachments/assets/ecd65f1b-7c26-4be5-bf60-42fad1c6312f)


To launch subscriber to monitor robot position:
Open new terminal,source package and run:

```sh 
ros2 run project1_pkg odom_subscriber.py
```
This will give information of the current position of the robot 

If odometry was settled correctly you can visualize it via the topic **/odom**, using programs as rqt_plot or plotjugger. For more information refer to the following useful link tutorial .All credits to the author https://www.youtube.com/watch?v=MnMGjvYxlUk
You can also run if installed
```sh 
ros2 run rqt_plot rqt_plot
```
![image](https://github.com/user-attachments/assets/e4f99df3-3217-4aff-919c-cc4ecce84ab1)

## Teleop:
There are two maps for teleop: competition_arena and competition_track. 

By default our package opens with the empty world. To launch the competition arena,
open the gazebo.launch.py in project1_pkg/launch. Uncomment the map for teleop section
in line 24 to 29. And comment out the empty world section from line 17 to line 22.

![image](https://github.com/user-attachments/assets/5e0b0c11-1796-448f-8890-f291092abf39)

Close any previously running gazebo terminals then

Build and source project1_pkg then launch gazebo with the command:

```sh
  colcon build --packages-select project1_pkg
```

```sh
  ros2 launch project1_pkg gazebo.launch.py
```

The following map should be displayed 
![Screenshot 2024-11-02 220423](https://github.com/user-attachments/assets/c0ef5b14-86e0-4e7c-9343-124d5dfa01fa)

Open new terminal and run:

```sh
  ros2 run project1_pkg teleop.py
```

Robot can now be controlled with W, A, S, D with W and S increasing and decreasing velocity 
respectively and A and D turning the wheel left and right respectively. 

To run the competition track, first close any running gazebo terminal then navigate to 
spawn_robot_ros2.launch.py in project1_pkg/launch. 

We need change the position so that the robot
spawns at the checkpoint on the track. Uncomment the position and orientation for competition lines
then comment the position and orientation for autonomous mode lines. 

![image](https://github.com/user-attachments/assets/ba658518-ba93-40c5-a2db-02c158fc7065)


Save, build, and source project1_pkg. 

```sh
colcon build --packages-select project1_pkg
```

Source the package 
```sh
source install/setup.bash
```

The competition track can then be run with the command:

```sh
ros2 launch project1_pkg competition.launch.py
```
![Screenshot 2024-11-02 012844](https://github.com/user-attachments/assets/f9b8efd5-510a-4f71-8e3a-ecff5c393554)

Then open new terminal,source package and run:

```sh
ros2 run project1_pkg teleop.py
```

![image](https://github.com/user-attachments/assets/b706db00-428b-44a5-9907-552f7effa192)


You should now be able to control the robot around the track with W, A, S, D

# Video links
## Teleoperation demo
https://drive.google.com/file/d/1k04x8Emz_ELX13NNIat-rTmemxfT8USf/view?usp=drive_link
## Autonomous mode demo
https://drive.google.com/file/d/1rhHgwKZt6jbfY2HQvarJ4jpp7cACh6Jj/view?usp=drive_link
