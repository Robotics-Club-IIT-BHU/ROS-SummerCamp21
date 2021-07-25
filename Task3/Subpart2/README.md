# Subpart-2

PHEWW!!!!
![](https://www.meme-arsenal.com/memes/83fdea9ed14a5e75791ae74d8d1f3d95.jpg)

That was a lot of work by you there pat yourself to make it till here.

Now its mostly gonna be a relaxing journey from now on And grab yourself a beverage. Below I would be explaining how to make your robot from scratch then what is the point of just learning everything what you learnt in pybullet camp again. Well now you will see the real power no how you can use packages from others and use them with just a few clicks and commands. Below I will be demonstrating how to download and install **TurtleBot3**. For other robots aswell you can follow similar process or the prescribed process given in the respective websites.

### Contribution
I would be very happy to see pull requests ( which I will merge on master ) with Instructions for installation of Other robots(that you had choosen) given in a similar fashion as shown for turtlebot3. As this will help others to get started quickly.


## Installing packages

### TurtleBot3

This is just a gist of the Turtlebot3 documentation [[link]](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation)

```bash
user@master :~$ cd catkin_ws/src
user@master :~/catkin_ws/src$ git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
user@master :~/catkin_ws/src$ cd ~/catkin_ws
user@master :~/catkin_ws$ catkin build #or catkin_make
user@master :~/catkin_ws$ source devel/setup.bash
user@master :~/catkin_ws$ echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
user@master :~/catkin_ws$ source ~/.bashrc
user@master :~/catkin_ws$ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```
In another terminal open teleop
```bash
user@master :~$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
with this you can control the turtle bot and move it around.

#### Additional Feature

Navigation and slam on turtlebot.
follow along
```bash
user@master :~$ cd catkin_ws/src
user@master :~/catkin_ws/src$ git clone https://github.com/ros-perception/slam_gmapping.git
user@master :~/catkin_ws/src$ cd ..
user@master :~/catkin_ws$ catkin build # or catkin_make
user@master :~/catkin_ws$ source devel/setup.bash
```

After Installing running the navigation and slam nodes for turtlebot.

Run the simulator in one terminal
```bash
user@master :~$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Run the SLAM node.
```bash
user@master :~$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```
Run teleop and take the robot around.
```bash
user@master :~$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
Then save the map
```bash
user@master :~$ rosrun map_server map_saver -f ~/map
```

Run Navigation node.
```bash
user@master :~$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=~/map.yaml
```

This is as easy to get a **STATE-OF-THE-ART** Algorithm working with your robot.

### Other robots.





## Task
Run your simulation with your choosen robot platform and run a planning node with a planning for either Manipulation, or Path Planning or any kind of Algorithm associated with your robot platform. Again talk to a mentor on discord at `@ROS-Team` for deciding your algorithm with your Robot Platform(Its important for you to discuss on Robocamp discord server so that we dont talk about the same platform again and again).

#### Submission
Simple submission of the task by submitting video of your robot simulation with a terminal open with your username visible.

**Deadline** : 29 July EOD ( Non Negotiable ).
