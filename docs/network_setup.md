# Real-World Cars - Software

## Network Setup

In order to run one of the real FreiCARs, setting up the correct environment is crucial.
This involves setting the correct ROS_DOMAIN_ID to create a capsulated environment for your team, as well as starting a ROS2 Middleware service called Zenoh.
**IMPORTANT** Keep in mind that while you are using the tracking all teams and cars share the same ROS_DOMAIN_ID. That means that in order to keep everything working, make sure that all your publishers, subscribers, topics, services and frames you might want to provide are prefixed with your car name. Overview:

* The **freicar-base**: This is the machine that runs the Vive tracking and publishes the transforms in regards to the real-world cars.
* Any **student computer**: For connecting to the car via ssh and running `rviz`. Other nodes may run here as well, but beware of network congestion if transmitting camera data.
* Any **FreiCAR**: The agent and any other real-time critical control code or code that processes images and sensor data should be run directly on the car. Otherwise, network lag may cause nondeterministic bugs and other issues.

Once Vive tracking is running, please proceed with the following steps to set up each of the machines:

### *freicar-base*
The following describes setting up the correct ROS_DOMAIN_ID.

0. source the ROS2 environment: ```source /opt/ros/jazzy/setup.bash```
1. Optionally, check that the ROS_DOMAIN_ID is set to 70: ```echo $ROS_DOMAIN_ID``` (it should be 70 by default)
2. TODO: Add in Zenoh setup
2. If not, set it: ```export ROS_DOMAIN_ID=70```
3. Start the Vive tracking script, after you have started SteamVR: ```ros2 run freicar_vr_tracking vive_tracking.py```
4. To visualize the trackers, run rviz: ```rviz2```

### *Student Computers* (To be updated soon)   
This lets you view published topics wrt. the ROS master that is running on *freicar-base*.

1. Start/attach to docker: ```fcc``` / ```fct```
2. ```export ROS_MASTER_URI=http://192.168.140.70:11311```
3. ```export ROS_IP=IP_OF_STUD_COMPUTER``` (check ```ifconfig```)
4. Check with ```rostopic list``` that you can connect to the master running on *freicar-base*.
5. To visualize the trackers and other data, run rviz with the default config: ```rviz```

### *FreiCARs* (To be updated soon)
Finally, [start](../starting_charging_etc/) one of the cars and:

1. SSH from a student computer to the car via ```ssh freicar@freicarX (name of car)```.
If no SSH connection is possible, please check that the car is connected to the wifi network freicar_5g (to do so connect a monitor, keyboard, and mouse)
2. Start/attach to docker: ```fcc``` / ```fct``` (optionally start a [tmux](https://en.wikipedia.org/wiki/Tmux) session before)
3. ```export ROS_MASTER_URI=http://192.168.140.70:11311```
4. ```export ROS_IP=192.168.140.XX``` (different for every car, e.g. .36 for freicar6 -> use ```ifconfig``` on car)
(*ROS_MASTER_URI* is necessary for listening, *ROS_IP* for publishing a topic)

## Test Setup (To be updated soon)
1. Always remember to use a terminal where *ROS_MASTER_URI* and *ROS_IP* have been exported before for the following. Use `env | grep 'ROS'` to quickly check all ROS environment variables.
2. Launch sensors on car ```roslaunch freicar_agent freicar_agent_hw.launch agent_name:=freicar_X``` (X is the number of your car)
3. Start ```rviz``` on student computer, add respective topics and check for validity.
4. Optionally start an agent `roslaunch freicar_agent freicar_minimal_agent.launch agent_name:=freicar_X`. See the launch file for other parameters.

    **WARNING:** always place the car on a wood block and turn on the RC remote before launching untested agents. Please observe the [safety rules](../safety_rules).
