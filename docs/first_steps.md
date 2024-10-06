# First Steps

This section explains how to start with developing in the FreiCar framework.

1. The first time, you want to start the FreiCar framework, you have to build all ROS packages. Go to `~/freicar_ws` and run `catkin build`. If the build fails due to missing dependencies, try running `caktin build` again. Due to interdependencies between FreiCAR packages, multiple builds may be necessary the first time. Once everything is built successfully, you have to re-source by typing `source devel/setup.bash`.
2. As the first step you should start the simulator by starting ```start_carla.launch```. For this please read the [Simulator section](/simulator) section.
3. Now the simulator should be started and you are ready to spawn your car.
 Run ```roslaunch freicar_launch spawn_sim_car.launch agent_name:=freicar_anyname spawn/x:=0 spawn/y:=0 spawn/z:=0 spawn/heading:=20 use_yaml_spawn:=true ```.
  Now your car "freicar_anyname" is spawned in the world and all sensors of the respective car are running. The car-name should be changeable throughout the course, so do not hardcode the name in your future own programs but use always ros-parameters.
4. Now start the [Rviz](http://wiki.ros.org/rviz#Overview) tool to visualize the world and the cars sensors. You should explore what data is available with ```rostopic list```

# Start Programming your own Code

For the FreiCar Course you can freely program any ROS node you want to have.

We prepared a template node [Freicar Agent](/nodes/freicar_agent/) for controlling the real-world cars. It shows you how to send the "Track Request", get the localization pose from the [Vive tracking system](/real_world_setup_sw/#vive-tracking) and send control commands to the hardware. This node is a good starting point if you want to program in C++.

See the chapter on [real-world cars](/real_world_setup_sw/#test-setup) for how to start and connect to the hardware cars, start the sensor stack and finally the agent node.

# Code Style

Always make your own **private** repository for your software. No changes to the submodules ``` base```, ```drivers``` or ```executables``` are allowed.
 
If you want to use code from these submodules you are allowed to copy these nodes, improve them, rename them and push them in your repository. 
