# Introduction to the FreiCar Nodes

In the following, we'll give you a short overview of the FreiCAR ROS nodes. For more detailed information, check the README files in each node. This page is split into three parts:

1. Nodes relevant for both the simulation and real-world environments.
2. Nodes that are relevant only for operating the real cars.
3. Nodes required for the carla simulation.

*Note:* it is not possible to exhaustively test all these nodes in an academical
*environment. If you run into any bugs, please send a message to one of your
*supervisors along with the the steps required to reproduce the it.

## 1 Base Nodes

These packages are relevant for both the simulation and real-world cars.

### 1.1 FreiCAR Launch
`base/freicar_launch` is a pseudo-package containing launch files, configs and scripts for launching the simulation and simulated cars as well as the sensors on the real cars.

### 1.2 FreiCAR Messages and raiscar_msgs
The package `freicar_msgs` contains message and service definitions for communication with real and simulated cars. `raiscar_msgs` is a legacy package containing duplicate definitions of those messages relevant to the real-world cars. This is required because the hardware controllers still expect the messages under the old name. As far as we have tested, the old and new messages are compatible if the definitions match exactly.

## 2 Nodes Relevant to Real-World Cars
These nodes are required for operating the real cars.
### 2.1 FreiCar Agent
The package in `base/freicar_agent` is the template node for all real-world agents. Currently, it:

- Sends a track request to the chaperone
- Tries to obtain a pose from Vive tracking
- Drives a pre-defined curve and terminates

The package also includes launch files for starting the sensor stack and the agent node itself. It is advisable to run the agent node on the car itself to avoid network congestion when transmitting large sensor data messages.

### 2.2 FreiCar Chaperone
The chaperone node is responsible for making sure the agents do not collide with each other. It also makes sure the agents remain inside a specified boundary polygon.

The chaperone node also serves `Track` requests. These help the chaperone node keep track of all the running agents and uses their tf information and (if available) planned corridors to try to predict and prevent collisions between agents. It does so by sending stop commands to the cars and force-stopping the hardware if the agent does not react.

## 3 Nodes for the Carla Simulation
### 3.1 FreiCar Carla Proxy
This node handles all the necessary tasks that are related to the simulator:

- Spawning the agent in CARLA
- Setting up the sensors and publishing their information
- Publishing the received pose on tf (simulated agents only)
- Removing the agent from simulation at shutdown

The sensor definition for each agent is a yaml file located at [`freicar_carla_proxy/param/sensors.yaml`](https://aisgit.informatik.uni-freiburg.de/vertensj/freicar_base/-/blob/master/freicar_carla_proxy/param/sensors.yaml). You can read more about carla sensors [here](https://carla.readthedocs.io/en/latest/ref_sensors/).
___
### 3.2 FreiCar Carla Agent
This node spawns an arbitrary number of scripted agents in the environment. These agents generate random plans and follow them while obeying right of way rules and avoiding collisions. They perform these tasks by using the `FreiCarAgentLocalization` messages from other agents. A pre-compiled version of this node is available in the repository `executables`.

### 3.3 FreiCar Carla Map
This node contains the core map structure and additional services that are built on top of it. You can find a more detailed explanation [here](/nodes/freicar_map).

### 3.4 FreiCar Setting
This node is responsible for applying the simulation settings and setting a couple of global ROS parameters.

#### 3.4.1 YAML File
The CARLA API exposes 3 settings for the simulation. They are read from `base/freicar_setting/param/carla_settings.yaml`.

##### no rendering
If `no-render-mode` is set to true, the simulator will stop rendering completely. This obviously uses less resources but also means the camera images will not be available.
**This is not the same as headless rendering**.

##### simulated steps per second
`sim-steps-per-second` determines the desired number of simulated steps per second. To achieve acceptable physics simulation, the number should be at least 10. If set to 0, the simulation will run at full speed. This will naturally lead to variable delta time between the steps.

**Note:** This does not determine the simulator's FPS, although it can affect it. It also doesn't determine the amount of data you get from the sensors. Those are set in the sensor description file.

##### synchronous mode
if `synchronous` is set to true, the simulation server will not proceed until a `Tick()` is received. Technically all CARLA clients can send it but this node is currently the only one responsible for the sake of consistency. Setting this to true will start a thread ([currently deactivated](https://aisgit.informatik.uni-freiburg.de/vertensj/freicar_base/-/blob/master/freicar_setting/src/main.cpp#L86)) that tries to tick the server according to `sim-steps-per-second`.

You can read more about [world settings](https://carla.readthedocs.io/en/latest/python_api/#carla.WorldSettings) and [client-server synchrony](https://carla.readthedocs.io/en/latest/adv_synchrony_timestep/).

#### 3.4.2 Parameters
The aforementioned settings are also set as global ROS parameters.

##### sim_sps
This defines the simulated steps per second. Other nodes (e.g. freicar_agent) use this as their thread's sleep value. If the simulator is trying to simulate the environment `x` times per seconds, it makes sense for us to update our agents with the same frequency.

##### sim_sync_mode
Indicates whether the synchronous mode has been activated. So far, no node uses this information.

##### sim_norender
Indicates whether the no-render mode has been activated. So far, no node uses this information.