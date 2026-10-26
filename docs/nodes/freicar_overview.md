# Introduction to the FreiCar ROS 2 Nodes

FreiCar has separate ROS 2 environments for the real cars and the CARLA
simulator. They share control message definitions, but have different drivers,
launch files, topic names, and ROS distributions. This overview describes the
`freicar_docker` hardware environment and `freicar_carla_docker` simulator environment.

For installation and container setup, see [Environment](../environment.md),
[First Steps](../first_steps.md), and [Simulator](../simulator.md).
The environment READMEs contain the detailed build and runtime instructions.

## 1. Environment split

| | Real cars: `freicar_docker` | Simulation: `freicar_carla_docker` |
|---|---|---|
| ROS distribution | ROS 2 Jazzy | ROS 2 Foxy |
| Runtime | Hardware development container | CARLA 0.9.11 container and a separate ROS 2 container |
| Source directory in the development checkout | `freicar_docker/ws/src/` | `freicar_carla_docker/ros2_ws/src/` |
| Workspace inside the ROS container | `/workspaces/ros2_ws` | `/opt/ros_ws` |
| Default agent | `freicar_4` | `freicar_1` |
| Command topic | `/<agent>/control_cmd` | `/<agent>/control` |
| Default ROS domain | `70` | `0` |
| Middleware | `rmw_zenoh_cpp`, with a router started by the container | DDS middleware configured in the ROS image |

Run each stack with its own sourced ROS environment. The defaults separate the
hardware and simulator ROS graphs; starting both containers does not connect
them. Matching message definitions alone does not make the two environments
interoperate. Controllers must use the appropriate command topic, sensor topics,
frames, and clock for their target environment.

The `freicar_docker` image includes the ZED SDK and wrapper.
The hardware launch leaves ZED disabled by default; enable it explicitly with
`use_zed:=true` and select the connected `zed_camera_model`.

## 2. Shared ROS 2 interfaces

### 2.1 Control messages

Both environments provide `raiscar_msgs`. Use its ROS 2 types for control:

| Purpose | ROS 2 type | Hardware topic | Simulator topic |
|---|---|---|---|
| Command | `raiscar_msgs/msg/ControlCommand` | `/freicar_4/control_cmd` | `/freicar_1/control` |
| Feedback | `raiscar_msgs/msg/ControlReport` | `/freicar_4/control_report` | `/freicar_1/control_report` |
| Enable ROS control | `std_msgs/msg/Bool` | `/freicar_4/control_mode` | `/freicar_1/control_mode` |

Replace the agent name to match your car. `freicar_msgs` also exists in the
hardware workspace for agent-level messages such as `AgentCommand`. Types from
`freicar_msgs` and `raiscar_msgs` are distinct ROS interfaces; do not substitute
one for the other because their fields look similar.

A `ControlCommand` contains `header`, `steering`, `throttle`, `brake`,
`throttle_mode`, and `hand_brake`. ROS 2 headers have no `seq` field; their
`stamp` uses `sec` and `nanosec`. Omitted fields receive their default values.
Inspect the installed definition with:

```bash
ros2 interface show raiscar_msgs/msg/ControlCommand
```

For example, publish a zero-throttle command to the simulator:

```bash
ros2 topic pub --rate 20 /freicar_1/control \
  raiscar_msgs/msg/ControlCommand \
  '{steering: 0.0, throttle: 0.0, brake: 0.0, throttle_mode: 0, hand_brake: false}'
```

For hardware, the corresponding topic is `/freicar_4/control_cmd`.
Use one controller or manual command publisher per car at a time.

### 2.2 ROS 2 launch and parameters

Use `ros2 launch`, `ros2 run`, `ros2 topic`, and `ros2 service` in a sourced
container shell. Parameters belong to individual nodes and are supplied through
launch files or `ros2 param`; there is no global ROS parameter server.
Simulator consumers that use timestamps should set `use_sim_time:=true` so that
they use the bridge's `/clock`.

## 3. Nodes for real cars (`freicar_docker`)

### 3.1 Hardware launch and sensors

`base/freicar_agent` contains the robot description and the hardware launch:

```bash
ros2 launch freicar_agent ros2_hw.launch.py agent_name:=freicar_4 use_lidar:=true
```

This starts:

- `raiscar_serial_bridge` for the BaseController connection;
- the RealSense D435 sensor launch from `freicar_launch`;
- the SICK scanner driver when `use_lidar:=true`, publishing
  `/freicar_4/lidar_scan`;
- `robot_state_publisher` for the robot's mounting transforms;
- optional ZED drivers (`use_zed:=true`) or RF2O (`use_rf2o:=true`).

The hardware launch does **not** start the example driving agent or Cartographer.
`freicar_launch` provides sensor configuration and the separate
mapping and odometry launches.

### 3.2 BaseController serial bridge

`base/raiscar_serial_bridge` exposes ROS 2 control topics and translates them to
and from the existing BaseController serial protocol. The firmware's rosserial
wire format is handled inside this bridge; no ROS 1 runtime is required.

The default serial device is `/dev/ttyACM0` at 115200 baud. To start the bridge
independently, for example with a different device:

```bash
ros2 launch raiscar_serial_bridge raiscar_serial_bridge.launch.py \
  agent_name:=freicar_4 port:=/dev/ttyACM0
```

Do not start a second bridge if the hardware launch already owns the device.
The bridge subscribes to namespaced `control_cmd`, `control_mode`, and `brake_cmd`
topics and publishes `control_report`. ROS control must be enabled before the
firmware accepts autonomous commands:

```bash
ros2 topic pub --once /freicar_4/control_mode std_msgs/msg/Bool '{data: true}'
ros2 topic echo /freicar_4/control_report
```

Check the reported mode. The hand remote can disable autonomous control.

### 3.3 LiDAR mapping and odometry

Start Cartographer separately after the hardware launch:

```bash
ros2 launch freicar_launch cartographer_slam.launch.py agent_name:=freicar_4
```

The current configuration uses LiDAR only, without an IMU or external odometry.
Cartographer owns `freicar_4/map -> freicar_4/odom` and
`freicar_4/odom -> freicar_4/base_link`. The `tf_to_odom_node` adapter publishes
`/freicar_4/odom` as `nav_msgs/msg/Odometry`; the occupancy grid is
`/freicar_4/map`. Use `freicar_4/map` as RViz's fixed frame.

RF2O is an alternative odometry source and is disabled by default. Do not enable
it alongside this Cartographer configuration: both would publish the odometry
transform. Each TF edge must have one publisher.

### 3.4 Example agent and migration status

The example driving node is in `base/freicar_agent`, rather than a separate
`freicar_minimal_agent` package. Its launch is:

```bash
ros2 launch freicar_agent ros2_minimal_agent.py agent_name:=freicar_4
```

It publishes hardware commands, handles agent stop/resume messages, and includes
TF pose lookup helpers. It is a template that sends a predefined steering
pattern, not a complete autonomous controller. In the current `Drive()`
implementation, localization checking is still a TODO and is bypassed.

The `freicar_chaperone` directory contains legacy ROS 1 code and is excluded
from colcon by `COLCON_IGNORE`. It is not an active ROS 2 safety service, and
the current agent does not register through the old `Track` service. Do not
assume the historical collision or boundary checks are running.

## 4. Nodes for simulation (`freicar_carla_docker`)

### 4.1 Runtime launch and CARLA bridge

The simulator uses two containers: CARLA 0.9.11 and a ROS 2 Foxy runtime matched
to the CARLA Python API and bridge version. From the `freicar_carla_docker` directory,
start and check the stack with:

```bash
./scripts/start.sh
./scripts/check.sh
./scripts/shell.sh
```

Inside the ROS container, `freicar_carla_ros2/freicar_carla.launch.py` starts:

- `carla_ros_bridge` for simulator communication, sensor data, vehicle state,
  TF, and simulation time;
- `carla_spawn_objects` for the configured vehicle and attached sensors;
- `freicar_control_adapter` to translate FreiCar commands and publish reports;
- `freicar_carla_map` when the selected world has a bundled semantic map.

The vehicle and sensor definitions are in
`ros2_ws/src/runtime/freicar_carla_ros2/config/freicar.json`. Keep the configured
object ID and launch `role_name` consistent. The default is `freicar_1`.
The current bridge launch uses asynchronous mode (`synchronous_mode: false`).
Simulation settings are node parameters in this launch, replacing the old
`freicar_setting` node and global `sim_*` parameters.

### 4.2 Control adapter and sensor topics

The adapter consumes `/freicar_1/control` and publishes
`/carla/freicar_1/vehicle_control_cmd`. Steering uses FreiCar degrees.
`throttle_mode: 0` selects duty control with throttle in `[-1, 1]`; negative
values select reverse. Mode `1` selects signed target velocity in m/s.
These are the simulator adapter's semantics; hardware behavior follows its
firmware and car configuration.

ROS control is enabled when the adapter starts. Sending `false` to
`/freicar_1/control_mode` applies the brake and disables ROS commands; `true`
enables them again. The watchdog stops the car after 0.5 seconds without a
command, so controllers must publish continuously.

| Output | Topic | ROS 2 type |
|---|---|---|
| RGB image | `/carla/freicar_1/camera_rgb_front/image` | `sensor_msgs/msg/Image` |
| RGB calibration | `/carla/freicar_1/camera_rgb_front/camera_info` | `sensor_msgs/msg/CameraInfo` |
| Depth in metres | `/carla/freicar_1/camera_depth_front/image` | `sensor_msgs/msg/Image` (`32FC1`) |
| LiDAR | `/carla/freicar_1/lidar` | `sensor_msgs/msg/PointCloud2` |
| Odometry | `/carla/freicar_1/odometry` | `nav_msgs/msg/Odometry` |

The simulated LiDAR output is a point cloud, while the hardware mapping setup
uses a `LaserScan`. Sensor topics are not interchangeable without adapting the
consumer or providing a conversion.

### 4.3 Student controllers and runtime ownership

The packages in `ros2_ws/src/runtime/` are infrastructure maintained by course
staff. Student controllers and milestone packages are distributed separately.
When the `freicar_control` exercise package is installed, its path publisher
provides `/freicar_1/path_segment` (`raiscar_msgs/msg/ControllerPath`) and
`/freicar_1/path_viz` (`geometry_msgs/msg/PoseArray`). Route subscriptions must
match the reliable, transient-local QoS used by the path publisher; keep that
publisher running to retain the route for restarted controllers.

The old `freicar_carla_proxy`, scripted CARLA agent executable, and
`freicar_setting` are not required by this runtime. Their responsibilities are
now covered by the bridge, object spawner, adapter, launch parameters, and
separately supplied controllers.

## 5. Inspect the running ROS 2 graph

Run these commands inside the appropriate sourced ROS container:

```bash
ros2 node list
ros2 topic list -t
ros2 service list -t
```

For hardware mapping, check:

```bash
ros2 topic hz /freicar_4/lidar_scan
ros2 topic hz /freicar_4/odom
ros2 run tf2_ros tf2_echo freicar_4/odom freicar_4/base_link
```

For simulation, use `./scripts/check.sh` from the host's `freicar_carla_docker`
directory. It checks the running simulator stack, including semantic map
markers and waypoint requests for supported maps.
