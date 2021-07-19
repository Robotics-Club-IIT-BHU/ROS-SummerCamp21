# Subpart-2

In this subpart we will learn about ROS control. 

In the previous subpart we simulated a bot in gazebo. Now we need control to our robot and make it move, here's when ROS control comes into picture. ROS control is a set of packages that include controller interfaces, controller managers, transmissions and hardware_interfaces. You can know more about it [here](https://wiki.ros.org/ros_control).

To use ros_control with out robot, you need to add some additional elements to your urdf. The `<transmission>` element is used to link actuators to joints, see the [spec](http://ros.org/wiki/urdf/XML/Transmission) for exact XML format. Now lets add transmission to the rear wheels in our bot leaving front wheels free to move. Add these lines to urdf (you need to add it in the robot tag).

```xml
<transmission name="tran1">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint_left_rear_wheel">
        <hardwareInterface>EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="left_rear_motor">
        <hardwareInterface>EffortJointInterface</hardwareInterface>
        <mechanicalReduction>1</mechanicalReduction>
    </actuator>
</transmission>

<transmission name="tran2">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint_right_rear_wheel">
        <hardwareInterface>EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="right_rear_motor">
        <hardwareInterface>EffortJointInterface</hardwareInterface>
        <mechanicalReduction>1</mechanicalReduction>
    </actuator>
</transmission>
```

In addition to the transmission tags, a Gazebo plugin needs to be added to your URDF that actually parses the transmission tags and loads the appropriate hardware interfaces and controller manager. By default the gazebo_ros_control plugin is very simple, though it is also extensible via an additional plugin architecture to allow power users to create their own custom robot hardware interfaces between ros_control and Gazebo. Add these lines to the urdf file (you need to add it in the robot tag).

```xml
<gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/bot</robotNamespace>
    </plugin>
</gazebo>  
```

We'll next need to create a configuration file and launch file for our ros_control controllers that interface with Gazebo. Create a file `bot_control.yaml` in config directory which we created previously and paste the following in it

```yaml
bot:
  # Publish all joint states -----------------------------------
  joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 50  

  # Position Controllers ---------------------------------------
  left_wheel_controller:
    type: effort_controllers/JointPositionController
    joint: joint_left_rear_wheel
    pid: {p: 100.0, i: 0.01, d: 10.0}
  right_wheel_controller:
    type: effort_controllers/JointPositionController
    joint: joint_right_rear_wheel
    pid: {p: 100.0, i: 0.01, d: 10.0}
```

Create a launch file `bot_control.launch` in the launch directory and paste following code in it

```xml
<launch>

  <!-- Load joint controller configurations from YAML file to parameter server -->
  <rosparam file="$(find gazebo_demo)/config/bot_control.yaml" command="load"/>

  <!-- load the controllers -->
  <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
    output="screen" ns="/bot" args="left_wheel_controller right_wheel_controller joint_state_controller"/>

  <!-- convert joint states to TF transforms for rviz, etc -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"
    respawn="false" output="screen">
    <remap from="/joint_states" to="/bot/joint_states" />
  </node>

</launch>
```

Now launch the two files in two terminal

Terminal 1:

```bash
~/catkin_ws $ roslaunch gazebo_demo fire_up.launch
```

Terminal 2:

```bash
~/catkin_ws $ roslaunch gazebo_demo bot_control.launch
```

Now to see whether controllers are spawned use `rostopic list` command to list out all the topics, you should see output something similar to this

```bash
~/catkin_ws $ rostopic list
/bot/joint1_position_controller/command
/bot/joint1_position_controller/pid/parameter_descriptions
/bot/joint1_position_controller/pid/parameter_updates
/bot/joint1_position_controller/state
/bot/joint2_position_controller/command
/bot/joint2_position_controller/pid/parameter_descriptions
/bot/joint2_position_controller/pid/parameter_updates
/bot/joint2_position_controller/state
/bot/joint_states
/clock
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/performance_metrics
/gazebo/set_link_state
/gazebo/set_model_state
/rosout
/rosout_agg
/tf
/tf_static
```

so we need to publish data to `/bot/joint1_position_controller/command` topic to move left wheel. Run this command to publish data to above topic.

```bash
rostopic pub -1 /bot/joint1_position_controller/command std_msgs/Float64 "data: 1.5"
```

Now you can see robot move but the movement wont last so long. In order to continuously move bot we need to publish data continuously. Lets do that in python

Create a python file `move.py` in scripts directory and make it executable using following command.

```bash
~/catkin_ws/src/gazebo_demo/scripts$ sudo chmod +x move.py
```

Copy the following code into `move.py` file

```python
#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import random
import math
   
def main():
    rate = rospy.Rate(10) # 10hz
    lpos = 0
    rpos = 0
    left_pub = rospy.Publisher("/bot/joint1_position_controller/command", 
                                            Float64, queue_size=10)
    right_pub = rospy.Publisher("/bot/joint2_position_controller/command", 
                                            Float64, queue_size=10)
    while not rospy.is_shutdown():
        
        lpos += random.random()
        rpos += random.random()
        if random.randint(0, 4) != 0:
            left_pub.publish(lpos)
        if random.randint(0, 4) != 0:
            right_pub.publish(rpos) 

        rate.sleep()

if __name__ == '__main__':
    rospy.init_node("move_node", anonymous=True)
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

Now fire up three terminals and run following commands in it

Terminal 1:

```bash
~/catkin_ws$ roslaunch gazebo_demo fire_up.launch 
```

Terminal 2:

```bash
~/catkin_ws$ roslaunch gazebo_demo bot_control.launch
```

Terminal 3:

```bash
~/catkin_ws$ rosrun gazebo_demo move.py
```

Now you can see bot moving randomly. Something like this

https://user-images.githubusercontent.com/67263028/126157846-32fedd5e-2d97-4638-bdab-a30dbb9cf340.mp4

