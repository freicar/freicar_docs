# Simulator
Our simulator is capable of simulating all sensors of the physical FreiCar. The simulator comes with the docker image and thus does need to be installed manually.
 
## Starting the Simulator
The simulator together with some necessary ROS-nodes can be started by running:

```roslaunch freicar_launch start_carla.launch```

It takes a few seconds to launch. Only once the line `waypoint_service active` appears in the terminal is the simulator ready.

The simulator needs always to run first if you want to start your own programs.
Your ROS-nodes must be started in a separate launch file.  In this launch file
also the map is specified. See down below how to change it.

### Launching a Simulated FreiCAR

Now the simulator should be started and you are ready to spawn a car. Run:

`roslaunch freicar_launch spawn_sim_car.launch agent_name:=freicar_X`

Now your car "freicar_X" is spawned in the world and all sensors of the respective car are running. The car-name should be changeable throughout the course, so do not hardcode the name in your future own programs but use always ros-parameters.

Now start the [Rviz](http://wiki.ros.org/rviz#Overview) tool to visualize the world and the cars sensors. You can add topics to rviz to visualize the data published on them. You can see which topics are available with `rostopic list`.

### Controlling a Simulated FreiCAR
The topic `/freicar_X/control` is especially interesting, since this is where you can publish throttle and steering commands to control the car. During the course, this will be done automatically by your ROS nodes, but for now you can experiment with the settings by entering:

`rostopic pub /freicar_X/control <TAB> <TAB>` (press TAB twice)

The Auto-Completion should provide you with a message template looking like this:

```
freicar@ovomaltine:~$ rostopic pub /freicar_X/control raiscar_msgs/ControlCommand "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
steering: 0.0
throttle: 0.0
brake: 0.0
throttle_mode: 0
hand_brake: false"
```

Try changing the throttle and steering values and sending the message by pressing Enter. You will see the car move.

**IMPORTANT:** please only do this in the simulation. If you send steering commands to the real-world cars manually, they might accelerate uncontrollably and cause damage! Always follow the [safety rules](/safety_rules) when handling the real cars.



## Updating the Simulator
We might update the simulator from time to time. When a new simulator version comes out you can upgrade the simulator by running ```update_simulator.bash``` from the docker directory (only accessible from the host system).

## Simulated Sensors/Actuators
### Throttle, Steering, Brake
All cars can be controlled over throttle, steering and brake. The corresponding ROS-message can be looked up [here](https://rlgit.informatik.uni-freiburg.de/freicar/base/-/blob/master/freicar_msgs/msg/ControlCommand.msg) and can be sent to `/AnyCarName/control`.

### Localization
If the competition mode is turned off (see explanation below), a ground-truth localization will be provided as tf-transform from `/map -> /AnyCarName/handle` (center of the car). You can use a [ROS tf-listener](http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29) to query this transform in your code.  

### Odometry
The odometry of the car is provided on the topic `/AnyCarName/odometry` using the  [odometry message](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html) message type.

### RGB Camera
The RGB image of the front facing camera is published as [ROS image message](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html) on the topic: `/AnyCarName/sim/camera/rgb/front/image`.

![rgb_sensor](https://github.com/JohanVer/freicar_docs/raw/master/images/rgb_sensor.png "") 

### Depth Camera
We also provide a depth image (also as [ROS image message](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html)) ,which is aligning with the RGB camera, on the topic : `/AnyCarName/sim/camera/depth/front/image_float`. The depth image is encoded as single channel floating point image and the unit is meters.

![depth_sensor](https://github.com/JohanVer/freicar_docs/raw/master/images/depth_sensor.png "")

### Lidar
We simulate the Sick lidar on the topic `/AnyCarName/sim/lidar` as  [PointCloud2 message](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html). After receiving the message in one of your nodes we recommend [converting the PointCloud2 message to a pcl](https://answers.ros.org/question/136916/conversion-from-sensor_msgspointcloud2-to-pclpointcloudt/) format for convenience.

## Maps
We have prepared various urban and race maps that can be loaded into the simulator. 

In order to change the map open the file ```start_carla.launch``` from the package ```freicar_launch``` and change the ```map_name``` to the corresponding map-name in the table below.

You can also provide the map_name as command-line argument like: ``` roslaunch freicar_launch start_carla.launch map_name:=freicar_race_1.aismap ```

Preview | ROS Map Name (start_carla.launch)
--- | ---
![alt text](https://github.com/JohanVer/freicar_docs/raw/master/images/maps/freicar1.png "") | freicar_1.aismap
![alt text](https://github.com/JohanVer/freicar_docs/raw/master/images/maps/freicar2.png "") | freicar_2.aismap
![alt text](https://github.com/JohanVer/freicar_docs/raw/master/images/maps/freicar_race_1.png "") | freicar_race_1.aismap
![alt text](https://github.com/JohanVer/freicar_docs/raw/master/images/maps/freicar_race_2.png "") | freicar_race_2.aismap

## Spawn Poses
There are two ways to define the initial spawn pose of the car:

1. The `spawn_sim_car.launch` launchfile provides the parameters ```spawn/x```, ```spawn/y```, ```spawn/z``` and ```spawn/heading``` that can be used to setup the initial pose.
 
2. Additionally, `spawn_sim_car.launch` launchfile provides the parameter ```use_yaml_spawn```.
 If ```use_yaml_spawn``` is set to true the spawn pose will be read from the files located in ``` base/freicar_launch/spawn_positions ``` depending on the map that is defined by the global parameter ```map_path``` (set in ```start_carla.launch```).
 If the spawn position is used from the yaml file also the ros-parameter ``` spawn/x ``` etc will be set accordingly.

The heading angle is in degree and defined as follows:

![angle_definition](https://github.com/JohanVer/freicar_docs/raw/master/images/angle_def.png "")  
   

## Reset/Set Car Positions Manually

After spawning a car two ROS services are created (by freicar_carla_proxy).
 - ```/FREICAR_NAME/set_position``` (e.g ```/freicar_1/set_position```)
 - ```/FREICAR_NAME/reset_position``` (e.g ```/freicar_1/reset_position```)

These services can be used to either reset the cars position to the spawn pose or to set a custom pose. In both cases the car will be beamed to the changed pose.

### Set a new pose
Example to set a new pose (in case the car is called "freicar_1"):

``` rosservice call /freicar_1/set_position "{x: -0.3, y: -0.3, z: 0.0, heading: -45.0}" ```

### Reset to spawn pose
Example how to reset the cars pose to the spawn pose (in case the car is called "freicar_1"):

``` rosservice call /freicar_1/reset_position true ```