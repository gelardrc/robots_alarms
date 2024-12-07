# Robots_alarms

That is just a package that creates alarms for a specific project on ROS. It's a part of a bigger project also hosted on github.

# How to install

> git clone https://github.com/gelardrc/robots_alarms.git

Dependencies:

> git clone https://github.com/PX4/PX4-Autopilot.git

🚀️*Obs* :  Px4 is a kind of tricky package to setup. But once you install and set things up you just need to remenber to source this :

> source <your_catikin>/devel/setup.bash
>
> cd <PX4-Autopilot\_clone>
> DONT\_RUN=1 make px4\_sitl\_default gazebo-classic # only one time during install
>
> source Tools/simulation/gazebo-classic/setup\_gazebo.bash \$(pwd) \$(pwd)/build/px4\_sitl\_default
>
> export ROS\_PACKAGE\_PATH=\$ROS\_PACKAGE\_PATH:\$(pwd)
>
> export ROS\_PACKAGE\_PATH=\$ROS\_PACKAGE\_PATH:\$(pwd)/Tools/simulation/gazebo-classic/sitl\_gazebo-classic

> 🚀️ <a href="https://docs.px4.io/main/en/simulation/ros_interface.html">PX4 manual</a>

# How to run

> rosrun robot_alarms battery_alert.py <robot_id> <_battery_percent>

# Example

> roslaunch robot_alarms example.launch

*If everything is ok, you should see a empty world in gazebo, with an uav, and another window with Rviz.*

![](https://github.com/gelardrc/robots_alarms/blob/main/img/default_gzclient_camera(1)-2024-12-07T12_03_41.383000.jpg)
![](https://github.com/gelardrc/robots_alarms/blob/main/img/rviz.png)

>🚀️If you want to have a better uav's view on rviz, you could use this package. Just add it in example.launch

*And that is a rqt image showing how topics comunicate to each other:*

![](https://github.com/gelardrc/robots_alarms/blob/main/img/rosgraph.png)

# Topics

Subscriber - <name_space>/mavros/battery

Publisher -  /battery_status

Publisher - /battery_marker

👀️ In this package we also have px4_example.py - you can ignore it. I only wrote it to drop UAV's battery.

# To do list

* [ ]  Improve battery status (px4 has many parameters about battery)
* [ ]  Understand why battery never goes under 50% ( I guess parameters again )
* [ ]  Add another alert about nonmap objects. ( working )
