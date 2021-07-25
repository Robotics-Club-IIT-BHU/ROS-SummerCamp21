<p align="center"><img src="https://answers.ros.org/upfiles/14554624266871161.png" width="30" height="30"> <b>R</b>obotic<b>O</b>perating<b>S</b>ystem <h1>ROS-SummerCamp</h1> </p>

Welcome to the ROS specialization offered by Robotics Club, IIT(BHU) Varanasi. We Aim to deliver a Beginner Level of Understanding to ROS to get you started with projects with it.

What we plan to Teach over the course of 2.5-3 weeks.
- **Part 1**: **ROS Communication** <br/> We would be explaining the basics of ROS in which we would be explaining a basic structure of a ROS package/application.

- **Part 2**: **RViz and Auxilary Tools** <br/> Introduction to Debugging/Visualizing Elements and softwares that ROS offers

- **Part 3**: **Simulator and Gazebo** <br/> Brief Introduction of Simulator i.e., and few packages to be used along side for controlling a simple robot.
- **Part 4**: **Use of Packages** <br/> Finally we would introduce you to the world of Packages and the ease of using it along side your project.

**NOTE**: We would always prefer you to look up [**ros wiki**](http://wiki.ros.org/Documentation) and [**ros answers**](https://answers.ros.org/questions/) as they containing literally everything  from where most of us have learnt ROS.

## What is ROS
<img src="assests/you_asked.jpeg" width="150" height="150" align="right" />
ROS stands for Robot Operating System. Even if it says so, ROS is not a real operating system since it goes on top of operating systems like Linux. ROS is a framework on top of the O.S. that allows it to abstract the hardware from the software. This means you can think in terms of software for all the hardware of the robot. And that’s good news for you because this implies that you can actually create programs for robots without having to deal with the hardware. Cool right!
<br/><br/><br/><br/><br/><br/>
And And And

https://user-images.githubusercontent.com/56990337/122926836-e3bf2c80-d385-11eb-9b17-c65d81f06f87.mp4

Just Look at all the robots and also more [ROBOTs](https://robots.ros.org/) can be found which are developed by **ROS** from scratch.

<h4><p align="center">And are very modular for you to be able to use there code directly in your project.</p></h4>


## Why ROS?

ROS makes it easy to get a robot running and do the required task.
> These are the lines from the [original paper](http://robotics.stanford.edu/~ang/papers/icraoss09-ROS.pdf) :

>  Writing software for robots is difficult, particularly as the scale and scope of robotics continues to grow. Different types of robots can have wildly varying hardware, making code reuse nontrivial. On top of this, the sheer size of the required code can be daunting, as it must contain a deep stack starting from driver-level software and continuing up through perception, abstract reasoning, and beyond. Since the required breadth of expertise is well beyond the capabilities of any single researcher, robotics software architectures must also support large-scale software integration efforts.  

> To meet these challenges, many robotics researchers, including ourselves, have previously created a wide variety of frameworks to manage complexity and facilitate rapid prototyping of software for experiments, resulting in the many robotic software systems currently used in academia and industry. Each of these frameworks was designed for a particular purpose, perhaps in response to perceived weaknesses of other available frameworks, or to place emphasis on aspects which were seen as most important in the design process.

> ROS is also the product of tradeoffs and prioritizations made during its design cycle. We believe its emphasis on large-scale integrative robotics research will be useful in a wide variety of situations as robotic systems grow ever more complex

Because next time you have to use a hand manipulator to grasp your smiley ball, or make a Bayesian Map using depth Camera and IMU you need not heatup the battery of your calculator nor ...

<img src="https://user-images.githubusercontent.com/56990337/122928448-81ffc200-d387-11eb-8cd0-1c96bd0737b3.png" width="300"> <img src="https://user-images.githubusercontent.com/56990337/122928478-8a57fd00-d387-11eb-84bb-2c3b72f8d91c.png" width="300"/>

Just simply search for the ROS package and it will do the trick for you.
```bash
$ sudo apt-get install ros-openslam-gmapping
$ roslaunch gmapping gmapping.launch
...
....
......[process-started] Node launched
```
![slam](https://user-images.githubusercontent.com/56990337/122931433-84afe680-d38a-11eb-82b4-67c0014d80a3.gif)


## Who are we?

We are your mentors for the specilization for contacting us you can use the discord server and contact us at `@ROS-Team` we can also be contacted with personal details which are shared on the server. We are a group of 3 mentors
<p align="center">

<h4 align="center">Team ROS</h4>

<table align="center">
 <td align="center">
     <a href="https://github.com/hex-plex">
    <img src="https://hex-plex.github.io/authors/admin/avatar_hu33d8f2710ea4928d295bd08cdc05f6eb_346396_270x270_fill_q75_lanczos_center.jpg?s=460&v=4" width="115px;" alt=""/><br /><sub><b>Somnath Sendhil Kumar </b></sub></a><br />
    </td>
    <td align="center">
     <a href="https://github.com/vstark21">
    <img src="https://vstark21.github.io/assets/img/profile-img.jpg?s=460&v=4" width="100px;" alt=""/><br /><sub><b>Chepuri Vishwas</b></sub></a><br />
	</td>
	<td align="center">
     <a href="https://github.com/Amshra267">
    <img src="https://amshra267.github.io/Amshra267_Website/static/media/avatar.440eace9.svg?s=460&v=4" width="100px;" alt=""/><br /><sub><b>Aman Mishra</b></sub></a><br />
	</td>

</table>

</p>

Enough Chit Chatting lets get to the real business.

## What should I do Next!
Lets now move to installation.<br/>
Before that there are a few requirements.

- **Ubuntu 18.04.5** (Linux is a must): We prefer this specific variant as its well tried and tested. other set of OS that are supported in order of high to low stability and support.
    - Ubuntu 16.04 ( equivalent to 18.04).
    - Arch Linux (with any desktop environment).
    - Ubuntu 20.04 (not well supported for now).
    - Mac OS X (highly unstable but can be your last resort).
    - For everyone else [VirtualBox](https://www.virtualbox.org/)

For Installation Jump to [Installation.md](/Installation.md) which has a Detailed description on the Installation process.
