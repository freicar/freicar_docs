# First Steps

This section explains how to start with developing in the FreiCar framework.

## Building

The first time you want to start the FreiCar framework, you have to build all ROS packages:

1. Go to `~/freicar_ws` and run `catkin build`.
2. If the build fails due to missing dependencies, try running `caktin build` again. Due to interdependencies between FreiCAR packages, multiple builds may be necessary the first time.
3. Once everything is built successfully, you have to re-source by typing `source devel/setup.bash`.
2. To test the setup, you can try to start the simulator by launching ```start_carla.launch```. For this please read the [Simulator section](../simulator) section.

Now that you have built everything, some tips on using `catkin` in the future:

- Run `catkin build --this` from any package directory to build only this package instead of the entire workspace.
- If a package is causing trouble with the build (e.g. name collisions) and you're sure you don't need it for now, run `touch CATKIN_IGNORE` inside the **package root directory** to exclude it from future builds. This creates an empty file called `CATKIN_IGNORE`, which you can delete if you want to build the package again.
- If a build is failing without apparent reason, try to rebuild the entire workspace:

        catkin clean
        catkin build
        source ~/freicar_ws/devel/setup.bash

More details can be found in the [Catkin Cheatsheet](https://catkin-tools.readthedocs.io/en/latest/cheat_sheet.html).

## Programming Your Own Code

We prepared a template node *Freicar Agent* (see `freicar_ws/src/base/freicar_agent/`) for controlling the real-world cars. It shows you how to send the "Track Request", get the localization pose from the [Vive tracking system](../vive_tracking) and send control commands to the hardware.

See the documentation on the *Real-World Cars* for how to start and connect to the hardware cars, start the sensor stack and finally the agent node.

# Code Style

Always make your own **private** repository for your software. No changes to the submodules ``` base```, ```drivers``` or ```executables``` are allowed.
 
If you want to use code from these submodules you are allowed to copy these nodes, improve them, rename them and push them in your repository. 
