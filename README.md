# How does it work?

1 - To get started with autonomous exploration, first launch the Map Node 

by running the following command:

`ros2 launch slam_toolbox online_async_launch.py`

2 - Then, launch the Gazebo simulation environment by setting the TurtleBot3 

model, for example, using the following command. Note
`turtlebot3_world.launch.py` can be replaced with any saved Gazebo world:

`ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`

3 - Open rViz:
`rviz2`
At the bottom left, add the nodes by pressing the `Add` button
Select the tab to add nodes by topic and add the Scan, Odometry, nodes, and all
3x nodes under `slam_toolbox`

4 - Once the simulation environment is running, run the `auto_nav` 

package using the following command:

`ros2 run auto_nav control`

This will start the robot's autonomous exploration.
