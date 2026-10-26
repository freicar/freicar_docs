# Simulator
Our simulator comes with the docker image and thus does need to be installed manually.
 
## Starting the Simulator
The simulator together with some necessary ROS2-nodes can be started by running the following script from the docker directory:

```./scripts/start.sh```

It takes a few seconds to launch. Once the map has loaded in and you see a spawned car, you can check the integrity of the simulator by running:

```./scripts/check.sh```

If the check comes back with all "OK"'s, you are ready to go.
The simulator needs always to run first if you want to start your own programs.
There are some other maps you can load into the simulator. See the [Maps](#maps) section below for more information.

### Launching a Simulated FreiCAR (and how to reset it)

The car should already be spawned in the simulator. If you want to reset the car simply use the following script from the docker directory:

```./scripts/reset.sh```

after a few seconds the car should be reset to its spawn position.

Now start the [Rviz2](http://wiki.ros.org/rviz#Overview) tool to visualize the world and the cars sensors. You can add topics to rviz to visualize the data published on them. You can see which topics are available with `ros2 topic list`.
All the relevant topics for the car are prefixed with `/freicar_X/` where `X` is the number of the car (1,2,3,...).
CARLA sensor/state topics are prefixed with `/carla/freicar_X`.

### Controlling a Simulated FreiCAR
The topic `/freicar_X/control` is especially interesting, since this is where you can publish throttle and steering commands to control the car. During the course, this will be done automatically by your ROS nodes, but for now you can experiment with the settings by entering:

`ros2 topic pub /freicar_X/control <TAB> <TAB>` (press TAB twice)

The Auto-Completion should provide you with a message template looking like this:

```
ros2 topic pub /freicar_X/control raiscar_msgs/msg/ControlCommand \
"header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
steering: 0.0
throttle: 0.0
brake: 0.0
throttle_mode: 0
hand_brake: false"
```

Try changing the throttle and steering values and sending the message by pressing Enter. You will see the car move.

**IMPORTANT:** please only do this in the simulation. If you send throttling or steering commands to the real-world cars manually, they might accelerate uncontrollably and cause damage (The cars have quite a lot of power)! Always follow the [safety rules](../safety_rules) when handling the real cars.


## Simulated Sensors/Actuators
### Throttle, Steering, Brake
All cars can be controlled over throttle, steering and brake. The corresponding ROS2-message can be looked up [here](https://rlgit.informatik.uni-freiburg.de/freicar/base/-/blob/master/freicar_msgs/msg/ControlCommand.msg) and can be sent to `/AnyCarName/control`.

### Localization
If the competition mode is turned off (see explanation below), a ground-truth localization will be provided as tf-transform from `/map -> /AnyCarName/handle` (center of the car). You can use a [ROS tf-listener](http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29) to query this transform in your code.  

### Odometry
The odometry of the car is provided on the topic `/AnyCarName/odometry` using the  [odometry message](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html) message type.

## Maps
We have prepared various urban and race maps that can be loaded into the simulator. 

To list all availabe maps you can run the following command after starting the simulator:

```./scripts/list-maps.sh``` (some maps might not be working properly yet)

then if you want to load a specific map you can simply stop the simulator with:

```./scripts/stop.sh```

and then re-start it with the desired map as parameter (exactly as they appear in the list from `list_maps.sh`):

```./scripts/start.sh --map <map_name>```

Preview | Map Name
--- | ---
![alt text](https://github.com/JohanVer/freicar_docs/raw/master/images/maps/freicar1.png "") | freicar_1.aismap
![alt text](https://github.com/JohanVer/freicar_docs/raw/master/images/maps/freicar2.png "") | freicar_2.aismap
![alt text](https://github.com/JohanVer/freicar_docs/raw/master/images/maps/freicar_race_1.png "") | freicar_race_1.aismap
![alt text](https://github.com/JohanVer/freicar_docs/raw/master/images/maps/freicar_race_2.png "") | freicar_race_2.aismap

## Spawn Poses

In order to change the spawn pose of a car, you can simply edit `freicar.json` in the docker directory. The spawn pose is defined by the following parameters:
- x: x-coordinate of the spawn position in meters
- y: y-coordinate of the spawn position in meters
- z: z-coordinate of the spawn position in meters
- roll: roll angle of the spawn pose in degrees
- pitch: pitch angle of the spawn pose in degrees
- yaw: yaw angle of the spawn pose in degrees
![angle_definition](https://github.com/JohanVer/freicar_docs/raw/master/images/angle_def.png "")  
   
### Reset to spawn pose
After editing the spawn pose, you can reset the car to this position by running this script from the docker directory:

```./scripts/reset.sh```

Keep in mind that changes to `freicar.json` are persistent and will also be used when starting the simulator again. So maybe keep a backup of the original file if you want to revert to the default spawn pose later on.
