# FreiCAR Chaperone

The chaperone is a special node that acts as a safety controller. While it is still under development, it offers two key capabilities:

- Coordinating multiple cars, stopping them if they are in danger of colliding. This is done by either checking their current motions and extrapolating, or receiving planned corridors from low-level planners running on the cars and comparing those.
- Acting as a *geo-fence*, stopping the cars when they exit the area of the map.

In order to be seen by the chaperone, each agent must send a track request before starting. This registers the car with the chaperone, and the chaperone will track the car via Vive tracking and subscribe to its intended future path messages. The agent node should also subscribe to the chaperone's stop commands and stop/continue if it receives these commands. However, the chaperone can also *force-stop* a misbehaving agent by sending commands directly to their hardware controller.

See the README in the chaperone package for more information. Please note that the chaperone is still being developed and **may contain bugs**, so always follow the [safety rules](../safety_rules), even if the chaperone is active.


## Usage
1. First perform the [Network setup](../network_setup) your car (and your workstation).
2. Start [Vive tracking](../vive_tracking) (**outside** the docker environment).
3. Launch the docker on `base` by running `fcc` or `fct`, then verify the [ROS network setup](../network_setup).
4. Start the chaperone **inside the docker** on `base`:

    `roslaunch freicar_chaperone freicar_chaperone.launch`

