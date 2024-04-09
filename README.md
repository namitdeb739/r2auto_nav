# The 'Elevator' Problem: S/Ue

This repository was developed for NUS AY23/24 S2 EG2310. It includes navigation & communications code written for a TurtleBot 3 operating on ROS2, called **S/Ue**, because this module gave us so much emotional trauma that S/U-ing is the only logical choice.

**To anyone taking this module, good luck.**

## Description

The mission requires developing an integrated robot system which can autonomously navigate through a randomised maze, up until an 'elevator lobby' right after the exit. 

In the lobby, the robot is to send an HTTP request to a web server, which will trigger a random 1 of 2 doors top open.The server will also send a JSON response to the robot ID-ing the correct door. 

The robot will then proceed to the door and locate a bucked in a randomised location on the other side. From there, it will release 5 ping pong balls into the bucket. 

Finally, the robot will return to the maze develop a full map of the area using SLAM.

**NOTE**: Use of this repository requires the phsyical assembly of S/Ue. Installation instructions below are specific for the Remote PC. Full hardware & technical documentation can be found in the [documentation](https://github.com/namitdeb739/r2auto_nav/tree/main/documentation) folder.

## Getting Started

### Software Dependencies

* Ubuntu 20.04

### Installing

#### Install ROS2 Foxy on Remote PC
```
wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_foxy.sh
sudo chmod 755 ./install_ros2_foxy.sh
bash ./install_ros2_foxy.sh
```

#### Install Dependent ROS2 packages
1. Install Gazebo
```
sudo apt-get install ros-foxy-gazebo-*
```
2. Install Cartogrpaher
```
sudo apt install ros-foxy-cartographer
sudo apt install ros-foxy-cartographer-ros
```
3. Install Navigation2
```
sudo apt install ros-foxy-navigation2
sudo apt install ros-foxy-nav2-bringup
```
4. Install Slam Toolbox
```
sudo apt install ros-foxy-slam-toolbox
```

#### Install TurtleBot3 Packages
```
source ~/.bashrc
sudo apt install ros-foxy-dynamixel-sdk
sudo apt install ros-foxy-turtlebot3-msgs
sudo apt install ros-foxy-turtlebot3
```

#### Configure Environment for ROS2 Develoopment
```
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc
```

#### Create Aliases
For ease of use, we have created the following aliases which we have added to the `~/.bashrc` of our Remote PC and RPi4. We have listed them below.

**Note**: For our mission we had setup an AWS server which allows us to fetch the RPi4's IP-address whenever `sshrp` was called.

| Remote PC Alias | Command |
| :---------------- | :------ | 
| `sshrp` | `ssh ubuntu@<RPi4 IP-address>` | 
| `toolbox` | `ros2 launch slam_toolbox online_async_launch.py`|
| `rvm` | `cd ~/.rviz2 && rviz --display-config mission.rviz`|
| `ctl` | `cd ~/colcon_ws && colcon build --packages-select auto_nav && cd ~/colcon_ws/src/auto_nav/auto_nav && ros2 run auto_nav control` |
| `rteleop` | `ros2 run turtlebot3_teleop teleop_keyboard` |


| RPi4 Alias | Command |
| :---------------- | :------ | 
| `rosbu`        |   `ros2 launch turtlebot3_bringup robot.launch.py`    | 
| `line` |  |

#### Setting Up RViz Environment
Run Slam Toolbox
```
toolbox
```
Open RViz
```
rviz
```
Add the following Nodes:
* Map
* LaserScan
* Marker Array
* Interactive Markers

`File`>`Save Config` and save to the default directory, `~/.rviz`, named as `mission.rviz` (So that this configuration launches upon calling `rvm`)


### Executing program

Place S/Ue on the starting line.

1. Terminal 1: Bringup the TurtleBot
```
sshrp
rosbu
```

2. Terminal 2: Launch slam
```
toolbox
```

3. Terminal 3: Launch RViz
```
rvm
```
4. Terminal 4: Run `control.py`
```
ctl
```
5. Terminal 5: Run `lineFollower.py`
```
sshrp
cd turtlebot3_ws/src/r2auto_nav/r2auto_nav/
python3 lineFollower.py
```

S/Ue should then proceed to execute the mission.

## Troubleshooting

* To test S/Ue's movement independently, use `rteleop` to manually control movement.
* To test payload execution, run `python3 payload.py`.

## Authors

Contributors names and contact info

Namit Deb
[namitdeb739@gmail.com](namitdeb739@gmail.com)

Wong  Weng Hong [wongwh@u.nus.edu](wongwh@u.nus.edu)

Anitej Datta [anitej.datta@yahoo.in](anitej.datta@yahoo.in)

Vishwanath Annanya [1annanya3@gmail.com](1annanya3@gmail.com)

Anupama Sriram [anupamasriram000@gmail.com](1annanya3@gmail.com)

## Acknowledgments

The navigation algorithm developed in `control.py` was developed by 
* [awesome-readme](https://github.com/matiassingers/awesome-readme)
* [PurpleBooth](https://gist.github.com/PurpleBooth/109311bb0361f32d87a2)
* [dbader](https://github.com/dbader/readme-template)
* [zenorocha](https://gist.github.com/zenorocha/4526327)
* [fvcproductions](https://gist.github.com/fvcproductions/1bfc2d4aecb01a834b46)

