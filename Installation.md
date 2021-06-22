# Installation
For Installation the ROS wiki's page is more than sufficient.

For Users with **Ubuntu 18.04** or **Arch Linux**:<br/>
1. Install ROS Melodic from [ROS wiki Installation](http://wiki.ros.org/melodic/Installation/)

For Users with **Ubuntu 16.04** or **Mac OS X**
1. Install ROS Kinetic from [ROS wiki Installation](http://wiki.ros.org/kinetic/Installation/)

For Users with **Ubuntu 20.04**
1. Install ROS Noetic from [ROS wiki Installation](http://wiki.ros.org/noetic/Installation/)

Next :
2. Setup a catkin Environment from [ROS WiKi Managing Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
3. Follow Along our guide to shift to catkin build from [here](#catkinbuild)

## Catkin Build
If you followed along till here you will mostly have a file structure as follows



         ├── catkin_ws
             ├── build/
             |   ├── bin/
             |   ├── catkin/
             |   |   ...
             |   └── MakeFile
             ├── devel/
             |   ├── lib/
             |   ├── include/
             |   |   ...
             |   └── setup.bash
             └── src/
                 ├── genpy/
                 ├── packages1/
                 |   ...
                 └── CMakeLists.txt

But you have used `catkin_make` but the standard is to use `catkin build` This is more robust and supports mutltiple package type to be built together ( orocos, non-ros packages).

so first
```bash
$ rm -r build/*
$ rm -r devel/*
```

Then Install catkin tools
For Ubuntu
```bash
$ sudo apt-get update
$ sudo apt-get install python-catkin-tools
```
For others (depreciated)
```zsh
$ sudo pip install -U catkin_tools
--or--
$ git clone https://github.com/catkin/catkin_tools.git
$ cd catkin_tools
$ pip install -r requirements.txt --upgrade
$ python setup.py install --record install_manifest.txt
```


**Finally**
```bash
user@master: ~/catkin_ws$ catkin build
```
You will see `build/` and `devel/` are filled with new files built from your files from src

## Confirm Installation

```bash
user@master: ~/$ roscore
... logging to /home/user/.ros/log/49745fac-d334-11eb-a534-809133531cbd/roslaunch-user-27006.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://localhost:33949/
ros_comm version 1.14.10


SUMMARY
========

PARAMETERS
 * /rosdistro: melodic
 * /rosversion: 1.14.10

NODES

auto-starting new master
process[master]: started with pid [27016]
ROS_MASTER_URI=http://localhost:11311/

setting /run_id to 49745fac-d334-11eb-a534-809133531cbd
process[rosout-1]: started with pid [27027]
started core service [/rosout]

```
If this worked Hurrayyy!! :tada::tada:
