# Vive Tracking (Indoor Localization)

We use an HTC Vive tracking setup to obtain indoor ground-truth localization for recording maps and steering the cars. Follow this procedure to activate the tracking hardware:

## 1. Setup Hardware
1. Turn on at least 4 **lighthouses** at the power adapter (check green LED) 

    <img src="https://github.com/vniclas/freicar_docs/raw/master/images/vive_lighthouse2.jpg" alt="lighthouse" width="400"/>

2. Turn on fixed **reference tracker** and then plug in power (after green LED)

    <img src="https://github.com/vniclas/freicar_docs/raw/master/images/vive_tracker.jpg" alt="lighthouse" width="200"/>

3. Place **glasses** in center on some box, facing the ceiling (press blue button on connector box if no green light) 

    <img src="https://github.com/vniclas/freicar_docs/raw/master/images/vive_headset.png" alt="lighthouse" width="400"/>

4. Turn on **tracker on the car (or the mapping device)**: On the car, first unplug its power connection (either tracking OR charging), then press its button until green LED lights up. A flashing red LED denotes the need for charging.

    <img src="https://github.com/vniclas/freicar_docs/raw/master/images/vive_tracker.jpg" alt="lighthouse" width="200"/>

## 2. Start SteamVR
1. Login on *freicar-base* 
2. Check that the machine is connected to the network. If not, unplug and re-plug the ethernet cable.
3. Start standalone Steam as a background process
4. Please ignore all warnings and errors issued by Steam. 
5. Start SteamVR → check that 4 lighthouses, glasses, reference tracker and the number of car-wise trackers appear green. A little flash icon tells you that one of the trackers needs charging.
   
If one of the steps fail, first try restarting Steam and SteamVR. If the issue is still there, try rebooting freicar-base.

## 3. Publish to ROS
Note: different from the other computers, on freicar-base you do not need to run ROS commands inside the docker (**no** `fcc` or `fct` required!). So just open a regular terminal and:
1. Start a ROS Core server: `roscore`
2. In a seperate terminal, start the Vive tracking script: `rosrun freicar_vr_tracking vive_tracking.py`
3. Run `rostopic list` to check that the odometry topic(s) are available. The poses are published to the `/tf` topic (if in doubt, check Rviz or [tf_echo](https://wiki.ros.org/tf/Debugging%20tools#tf_echo)).

Now the poses and odometry are available from every machine in the network. Just remember to do the [network setup](network_setup.md).