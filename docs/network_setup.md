# Real-World Cars - Software

## Network Setup

In order to run one of the real FreiCARs, setting up three machines is required. Overview:

* **freicar-base**: Run the `roscore` and vive tracking. All nodes communicate via this machine.
* Any **student computer**: For connecting to the car via ssh and running `rviz`. Other nodes may run here as well, but beware of network congestion if transmitting camera data.
* Any **FreiCAR**: The agent and any other real-time critical control code or code that processes images and sensor data should be run directly on the car. Otherwise, network lag may cause nondeterministic bugs and other issues.

Once Vive tracking is running, please proceed with the following steps to set up each of the machines:

### *freicar-base*
The following describes sets up the ROS master and explains how to publish tracked cars.

1. Optionally, check that the *ROS_IP* and *ROS_MASTER_URI* are set:
2. ```echo $ROS_IP → 192.168.140.70```
3. ```echo $ROS_MASTER_URI → http://192.168.140.70:11311```
4. Start a ```roscore```
5. In a different terminal, start GT tracking: ```rosrun freicar_vr_tracking vive_tracking.py```
    - Restart if a new tracker was added
    - IDs of trackers are set in: ```freicar_vr_tracking/param/reserved_trackers.yaml```
8. To visualize the trackers, run rviz: ```rviz```

### *Student Computers*    
This lets you view published topics wrt. the ROS master that is running on *freicar-base*.

1. Start/attach to docker: ```fcc``` / ```fct```
2. ```export ROS_MASTER_URI=http://192.168.140.70:11311```
3. ```export ROS_IP=IP_OF_STUD_COMPUTER``` (check ```ifconfig```)
4. Check with ```rostopic list``` that you can connect to the master running on *freicar-base*.
5. To visualize the trackers and other data, run rviz with the default config: ```rviz```

### *FreiCARs*
Finally, [start](../starting_charging_etc/) one of the cars and:

1. SSH from a student computer to the car via ```ssh freicar@freicarX (name of car)```.
If no SSH connection is possible, please check that the car is connected to the wifi network freicar_5g (to do so connect a monitor, keyboard, and mouse)
2. Start/attach to docker: ```fcc``` / ```fct``` (optionally start a [tmux](https://en.wikipedia.org/wiki/Tmux) session before)
3. ```export ROS_MASTER_URI=http://192.168.140.70:11311```
4. ```export ROS_IP=192.168.140.XX``` (different for every car, e.g. .36 for freicar6 -> use ```ifconfig``` on car)
(*ROS_MASTER_URI* is necessary for listening, *ROS_IP* for publishing a topic)

## Test Setup
1. Always remember to use a terminal where *ROS_MASTER_URI* and *ROS_IP* have been exported before for the following. Use `env | grep 'ROS'` to quickly check all ROS environment variables.
2. Launch sensors on car ```roslaunch freicar_agent freicar_agent_hw.launch agent_name:=freicar_X``` (X is the number of your car)
3. Start ```rviz``` on student computer, add respective topics and check for validity.
4. Optionally start an agent `roslaunch freicar_agent freicar_agent.launch agent_name:=freicar_X`. See the launch file for other parameters.

    **WARNING:** always place the car on a wood block and turn on the RC remote before launching untested agents. Please observe the [safety rules](../safety_rules).
