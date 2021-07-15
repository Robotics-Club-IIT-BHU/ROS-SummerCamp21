# Subpart-1 

<h2 align="center">RViz</h2>

**Visualisation** is the key which make robotics fun, Generally we want to see what the robot is seeing, how it joints, links are behaving while interacting with environment etc.... so for these purposes ROS have a package RViz(known as ROS Visualiser) to allow us to see and visualise all these things.

It allows us to....
 - See the image from the camera mounted on robot.
 - Visulaise and debug URDFs.
 - Odometry and Mapping of the environment.
 - set Markers
 - visulaise transformations 
 - and so on.......

Before moving further, see these Videos for Rviz Introduction and feel



[Rviz Introduction by The Construct](https://www.youtube.com/watch?v=yLwr5Zhr_t8&t=137s)<br/>
[More indepth usage](https://www.youtube.com/watch?v=6pep5xB4pEU)


Don't Worry if you don't understand few terms, you learn about most of them as the camp progress.
<br/>

Before moving further with RViz, let's first learn about **tf or transforms(transformations)**.

<h2 align="center">tf</h2>

In pybullet camp, you found an example of transformation i.e., **euler2quaternion** or **quaternion2euler**, but robotics is not just limited to it. To learn more about transformations in real world with robotics see [This video from Maryland University](https://www.youtube.com/watch?v=olmAyg_mrdI&list=PLZgpos4wVnCYhf5jsl2HcsCl_Pql6Kigk&index=2&t=837s)(Although this video is on quadrotors but nearly same math applies in all robotics field.)

In ROS, there is a seprate package which handles transformations, but one might be thinking...

<p align="center">
<img width = 500 height=300 src = "../../assests/ask.jpeg">
</p>

I will left the answer on [Kostas Danillidis-Proff at University of Pennsylvania](https://www.coursera.org/lecture/robotics-perception/rotations-and-translations-eTSMz)

In the video you found how different transformations need to be consider just for a single camera. But real robotics applications not just limits to a camera but include sensors like [IMU](https://en.wikipedia.org/wiki/Inertial_measurement_unit), [GPS](https://en.wikipedia.org/wiki/Global_Positioning_System), [LaserScan](https://en.wikipedia.org/wiki/Laser_scanning), [Encoders](https://www.encoder.com/article-what-is-an-encoder), and many more like this can be included, Hence to make all sensor synchronized ROS reduces the overhead of managing transformations differently but made a compact way of handling these through tf package. Additionally, it can be extended for joints, links transformations as well and allows you to view your entire robot urdf and all its parent and child frames using TF_ViewFrames. 

To learn more thoroughly about it Refer [ROS-Wiki/tf](http://wiki.ros.org/tf).

Also you can look about RViz from [ROS-Wiki/RViz](http://wiki.ros.org/rviz)(Although i think it will be hard for some of you to understand it directly from wiki for now..).

## HANDS ON

Only seeing something don't make you master in it. so let's try some things by yourselves...

Run below Packages Step by Step to get the flavour of Rviz and tf.

P.S. -> We are not going indepth of the underlying code(As our aim is to give you a headstart on how these are useful and not telling about every knitty-gritty details), if one is willing he/she can go through it for better understanding

 - [Learning Urdf Step by Step](http://wiki.ros.org/urdf/Tutorials) - Although these concepts were taught in pybullet camp but here you will actually going to visulaise it in RViz-ROS. **Just Follow the steps as mentioned there and try to Run locally-> Learning URDF Step by Step Tutorial for first three section till now(Upto- Adding Physical and Collision Properties to a URDF Model).**
   
 - 2
 - 3 
       

## Task for this Subpart

From Pybullet camp, you are familiar with building urdfs(assuming only static models). In this task your aim is to build the geometrical tags for the below robot(Original Version developed by RoboREG-adversarial_cars team [here](https://github.com/Robotics-Club-IIT-BHU/gym-adversarial-cars)) and visulaise it in RViz and debug there.
<p align ="center">
<img width = 400 height=300 src = "https://github.com/Robotics-Club-IIT-BHU/gym-adversarial-cars/blob/main/media/auto.png"><br/>
<b>Credits for this URDF- gym-adversarial_cars team - RoboREG</b>
</p>

Although the model was completely build and exported from **Blender** as URDF, so we won't expect you to mimic the same but just make a rough geometry of it and provide the visulaisations in RVIZ as snapshot for the final bot. You can use the [Visulaising urdf tutorial from ROS for your urdf](http://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch).

Happy Learning!!