**Intro**:
This is a piece of coursework part of my robotics course during my master's at the university of birmingham.

**Disclosure**:
This was a solo coursework

**Outcome of file**:
This file was run successfully in ros1 noetic.

**Task**:
The task is given in CW.md
This package implements the particle filter localisation using sensor and motion update from the Pioneer P3-DX robot.
A file of skeleton code has been given; this skelton code setups the map, the sensors and publisher/subsciber topics.
The main addition to the code by me was done in the pf.py file.

**Solution**:
A particle filter was created by creating particles and then ressampling then using round robin sampling with the probabilty being calculated using laser scan data.


**How to run** :
roscore
rosrun map_server map_server catkin_ws/src/pf_localisation/data/sim_data/meeting.yaml
rosrun stage_ros stageros catkin_ws/src/socspioneer/data/meeting.world
rosrun stage_ros stageros catkin_ws/src/pf_localisation/data/sim_data/meeting.world

roslaunch socspioneer keyboard_teleop.launch
rosrun pf_localisation node.py

setup rviz with map; laserscan and pose
set 2D estimated pose in rviz

**Results**:
The outcome was an algorithm that couyld localise the robot in a given map using a lser scan. The code also adapts well to the kidnapped robot problem.
