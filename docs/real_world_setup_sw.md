# Real-World Cars - Software

## Vive Tracking
1. Turn on at least 4 **lighthouses** at the power adapter (check green LED) 
<img src="https://github.com/vniclas/freicar_docs/raw/master/images/vive_lighthouse2.jpg" alt="lighthouse" width="400"/>
2. Turn on fixed **reference tracker** and then plug in power (after green LED)
<img src="https://github.com/vniclas/freicar_docs/raw/master/images/vive_tracker.jpg" alt="lighthouse" width="200"/>
3. Place **glasses** in center on some box, facing the ceiling (press blue button on connector box if no green light) 
<img src="https://github.com/vniclas/freicar_docs/raw/master/images/vive_headset.png" alt="lighthouse" width="400"/>
4. Turn on **tracker on the car (or the mapping device)**: On the car, first unplug its power connection (either tracking OR charging), then press its button until green LED lights up. A flashing red LED denotes the need for charging.
<img src="https://github.com/vniclas/freicar_docs/raw/master/images/vive_tracker.jpg" alt="lighthouse" width="200"/>
5. Login on *freicar-base* 
6. Start standalone Steam as a background process
7. Please ignore all warnings and errors issued by Steam. 
8. Start SteamVR → check 4 lighthouses, glasses, reference tracker and the number of car-wise trackers

## Network Setup

### *freicar-base*
The following describes sets up the ROS master and explains how to publish tracked cars.

1. Optionally, check that the *ROS_IP* and *ROS_MASTER_URI* are set:
2. ```echo $ROS_IP → 192.168.140.70```
3. ```echo $ROS_MASTER_URI → http://192.168.140.70:11311```
4. Start a ```roscore```
5. Start GT tracking: ```rosrun freicar_vr_tracking vive_tracking.py```
6. Restart if a new car was added
7. IDs of trackers are set in: ```freicar_vr_tracking/param/reserved_trackers.yaml```
8. To visualize the trackers, run rviz in the docker with the default config: ```fcc``` → ```rviz```

### *Student Computers*    
This lets you view published topics wrt. the ROS master that is running on *freicar-base*.

1. Start/attach to docker: ```fcc``` / ```fct```
2. ```export ROS_MASTER_URI=http://192.168.140.70:11311```
3. ```export ROS_IP=IP_OF_STUD_COMPUTER``` (check ```ifconfig```)
4. Check with ```rostopic list``` that you can connect to the master running on *freicar-base*
5. To visualize the trackers, run rviz with the default config: ```rviz```

### *FreiCARs*
The following details how to authenticate cars in the ROS network for listening and publishing.

1. SSH from a student computer to a car via ```ssh freicar@freicarX (name of car)```.
If no SSH connection is possible, please check that the car is connected to the SSID freicar_5g (connect a monitor, keyboard, and mouse)
- Wifi: *freicar_5g*, password: *******, Router IP: *192.168.140.10* (admin/admin)
2. Start/attach to docker: ```fcc``` / ```fct``` (optionally start a tmux session before)
3. ```export ROS_MASTER_URI=http://192.168.140.70:11311```
4. ```export ROS_IP=192.168.140.XX``` (different for every car, e.g. .36 for freicar6 -> use ```ifconfig``` on car)
(*ROS_MASTER_URI* is necessary for listening, *ROS_IP* for publishing a topic)

## Test Setup
1. Always remember to use a terminal where *ROS_MASTER_URI* and *ROS_IP* have been exported before for the following.
2. Launch sensors on car ```roslaunch freicar_launch start_sensors.launch name:=freicar_X``` (X is the number of your car)
3. Start ```rviz``` on student computer, add respective topics and check for validity.



