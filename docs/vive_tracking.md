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
1. Log in on *freicar-base*
2. Check that the machine is connected to the network. If not, unplug and re-plug the ethernet cable.
3. Start standalone Steam as a background process
4. Please ignore all warnings and errors issued by Steam. 
5. Start SteamVR → check that 4 lighthouses, glasses, reference tracker and the number of car-wise trackers (or the mapping tool tracker) appear green. A little flash icon tells you that one of the trackers needs charging.
   
If one of the steps fails, first try restarting Steam and SteamVR. If the issue is still there, try rebooting freicar-base.

## 3. Publish to ROS2
Note: different from the other computers, on freicar-base you do not need to run ROS commands inside the docker (**no** `rcc` or `rct` required!). So just open a regular terminal and:

1. source the ROS2 environment: `source /opt/ros/jazzy/setup.bash`
2. You should not need to manually set the ROS_DOMAIN_ID, or start the Zenoh router, as this should be done by the tracking startup script.
3. start the Vive tracking script in `~/freicar/freicar_vive/`: `./start_tracking.sh`. This will enable communication between the Vive tracking system and ROS2. You should see a lot of output in the terminal, including the published transforms and odometry information. You may see some steps regarding 'burn-in' of the Vive trackers, simply wait until the 2000 iterations seem to be done. This step will also show you all recognized Vive trackers and their IDs and names.
4. Now you can start `freicar_docker` on your student PC and run `ros2 topic list` to see all available topics. You can also run `ros2 topic echo /tf` to see the published transforms. And in `rviz2` you can visualize the poses of the Vive trackers.
5. Step 4 also works from `freicar-base` itself.

If after these steps your student PCs still don't have access to the relevant topics, simply follow the [network setup](network_setup.md).
