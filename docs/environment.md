# FreiCAR Environment

Complex robots always involve large software stacks and setup procedures. 

To ease the process as much as possible we provide a complete docker container that includes everything you need, such as the simulator, libraries and IDEs.

## Usage
Our scripts add a few aliases to your bashrc. So after building everything close every open terminal so that on the next start everything gets sourced correctly.

Our docker container will provide most of the tools you need in a single container.
Using the container feels like a virtual machine that does not have the performance limitations of traditional VMs.

* Everytime when you want to use the environment you need start our container:
Therefore type `rcc` in a new terminal. This container must run only in one terminal at a time!

* Everytime you want to open additional terminals that interact with the containerized environment just open a new terminal and type `rct` (freicar terminal).

* If you close the terminal where `rcc` is running, all sessions will terminate without warning! Using [tmux](https://en.wikipedia.org/wiki/Tmux) or [screen](https://en.wikipedia.org/wiki/GNU_Screen) is advised.

The directory freicar_deps and freicar_ws are shared with your host. So every data that you create inside the docker within this directory can be read by the host, or vice versa. Additionally, only the data in these folders are persistent if you restart the docker environment. 

### Structure of ROS Workspace
Within the FreiCar container a ROS2 Workspace `freicar_ws`  will already be setup. In the workspace two git submodules are placed that contain several ROS nodes: `base` and `drivers` (and optionally `executables`, which contains a closed-source carla demo agent).

The `base` submodule contains some ROS modules that you need during the course.

NOTE: If there are updates to any submodules in the `docker` repository (e.g. if there is new exercise code available) you need to pull the `docker` repository using : `git submodule update --init --recursive` . 


### Persistent Changes to Docker Container
Sometimes it becomes necessary to install additional libraries or packages if your program needs them. While you can install them in the docker container with `sudo apt-get install`, the changes would vanish as soon you stop the docker.

However, you can make these changes persistent by executing the script `commit.sh` before closing the docker container. This will keep all changes you made and the next time you start up the container (with `rcc`) you will have changed environment.

## Installation (Will be updated soon)

### Carla Simulator (Up to date)
To set up the Carla simulator, you need a PC with:
* Ubuntu 20.04 LTS (64-bit)
* NVIDIA GPU with at least 4GB of VRAM (e.g., GTX 1050 or better)
* NVIDIA driver version 450.80.02 or newer
* Docker

After you gain access to Repository (via a zip file or git clone), you only need the correct .egg file for the Carla Python API. You can find it on your student PC's in ~/freicar/freicar_carla_docker/carla_python_api/ . Otherwise you may have to build it from source.
Then you simply have to do `./scripts/build.sh` in the `freicar_carla_docker` folder. This will build the docker image and set up the environment for you.

To run the Simulator, simply use:

* `./scripts/start.sh` to start the simulator in a new terminal. This will start the Carla simulator in a separate window.
* `./scripts/stop.sh` to stop the simulator. This will close the simulator window and stop the Carla server.
* `./scripts/shell.sh` to open a shell in the docker container. This will give you access to the ROS workspace and all the tools you need.
* `./scripts/commit.sh` to commit changes to the docker container. This will save any changes you made to the container, so you can use them next time you start the container.
* `./scripts/list_maps.sh` to list all available maps in the Carla simulator. This will show you a list of maps that you can use in your experiments.
* `./scripts/reset.sh` to reset the car's position in the Carla simulator. This will reset the car to its initial position in the map.

### FreiCar Docker (Will follow soon)
PASS
