# FreiCar hardware documentation

## I. Usage

### **Powering on**

*Power Panel*
![Side_panel.png](https://github.com/vniclas/freicar_docs/raw/master/images/hardware_documentation/Side_panel.png)

- Hold the motor button down for 1 ~ 2 seconds until it lights blue.
- Switch the PC switch to *I* state or alternatively connect the adapter cable. Note that *II* state is unused.
    
    ![adapter_cable.jpg](https://github.com/vniclas/freicar_docs/raw/master/images/hardware_documentation/adapter_cable.jpg)
    
- The panel should look like below after powering on.

![Powered_on_vehicle.jpg](https://github.com/vniclas/freicar_docs/raw/master/images/hardware_documentation/Powered_on_vehicle.jpg)

### **Charging**

*Charging Overview*
![Charger_overview_annotated.png](https://github.com/vniclas/freicar_docs/raw/master/images/hardware_documentation/Charger_overview_annotated.png)

*Balancer connection*  
![balancer_socket.png](https://github.com/vniclas/freicar_docs/raw/master/images/hardware_documentation/balancer_socket.png)

**Steps**
<ol>
<li>Connect the dean plugs and the balancer cables for both battery systems.</li>
<li>Click the task button.</li>
<li>Select channel 1 or 2.</li>
<li>Select a current of 2 A (recommended is: 25% of battery capacity).</li>
<li>Make sure the number of cells is 4 for the motor batteries and 6 for the PC battery.</li>
<li>Make sure the task is charge</li>
<li>Use the task scroller to navigate to 'Start' and select it.</li>
<li>Repeat with the second channel.</li>
<li>Click on the channel button again to get back to the two channels view.</li>
<li>There is a percentage indicator for the charging state on the top right.</li>
<li>The charger decreases the charging current gradually as the charging process approaches its end.</li>
</ol>

The charger menu should look like the following
![PC_channel_annotated.png](https://github.com/vniclas/freicar_docs/raw/master/images/hardware_documentation/PC_channel_annotated.png)
![motor_channel_annotated.png](https://github.com/vniclas/freicar_docs/raw/master/images/hardware_documentation/motor_channel_annotated.png)

**Stop Charging**

When the batteries are fully charged, the respective channel color turns green followed by a blue tick, a short beep sound, then the charging stops.

To willingly stop the charging process:
<ol>
<li>Select the target channel.</li>
<li>Use the task scroller to navigate to 'Stop' and select it.</li>
</ol>

**Background Information**

*Balancing*: Each lithium ion battery has a number of cells, that are connected in series. The balancer cable gives the charger information about the voltage levels of the individual cells. The charger makes sure that the cells are always approximately equal when in balancer charging mode.

*Nominal Voltage*: The *PC battery* has 6 cells with 3.7 V nominal voltage each, that sums to a total of 22.2 V. The 2 *motor batteries* combined have 4 cells with 3.7 V nominal voltage each, that sums to a total of 14.8 V.

*Maximum and minimum voltages*: The individual cells level shouldn’t fall below 3 V and shouldn’t exceed 4.2 V.

**<span style="color:red">Attention!</span>**

- If you connect an unbalanced battery directly to your system, there is a big risk of it dying beyond repair.
- Do not leave the car charging unattended.

**Troubleshooting**

- Check the layout of the balancer cables and the dean plugs.
- Check that the battery cables are solidly connected from both sides.
- Check that the individual battery cells didn’t drop too low.

### **PC Power Hotswapping**
- To switch the PC power source from the PC battery to the adapter:
    <ol>
     <li>Keep the PC switch on *I* state.</li>
     <li>Connect the adapter cable.</li>
     <li>Switch the PC to *O* state.</li>
    </ol>
- To switch the PC power source from the adapter to the PC battery:
    <ol>
    <li>Keep the adapter connected.</li>
    <li>Switch the PC to *I* state.</li>
    <li>Disconnect the adapter.</li>
    </ol>

### **Turn off Alarms**
- Missing communication between the PC and the motor (higher frequency alarm):
    The motor turns itself off automatically approximately every 20 minutes. This results in a lost communication between the PC and the motor. To stop the alarm, hold the motor button down for 1~2 seconds until it lights blue again.
- Low battery (lower frequency alarm):
    Power off the car and charge it.

### **Powering off**

**Steps**
<ol>
<li>Shut down the PC.</li>
<li>Switch the PC to *O* state or disconnect the adapter.</li>
<li>Press the motor button for 1~2 seconds until the blue light disappears.</li>
</ol>

## **II. Documentation**

### Powering System

- For switching check [*switches and buttons*](#switches-and-buttons).
- For more information about the batteries check the [*battery system*](#battery-system).

**PC**

*Power Path*: The PC power pass through the base board, which powers the PSU, which in turn powers the motherboard.

*Power Source*: The power comes from either the PC battery or an adapter connected to an ac plug. The adapter overrides the PC battery when connected and powers on the base board.

**Motor**

*Power Path:* The motor power pass through the anti-spark switch which powers the VESC, which in turns control the motor.

*Power Source:* Two li-po 7.4 V batteries connected in series.

### **Battery System**

There are two independent battery systems on the vehicle.

**PC battery**

The battery has 6 series li-po cells with a total nominal voltage of 22.2 V. The PC battery poles are to be connected directly to the respective poles of the charger through the charging dean socket.

**Motor batteries**

There are 2 motor batteries, each with 2 series li-po cells, connected in series. Their total nominal voltage is 14.8 V. The positive pole of one battery and the negative pole of the other are connected directly to the charger respective poles through the charging dean socket. The remaining poles are connecting the two batteries together.

### **Switches and Buttons**

There are two independent switches on the vehicle. One to turn on the PC and the other to turn on the motor.

**PC**

The PC switch connects or disconnects the PC battery +ve pole to or from the base board. Note that the negative pole is always connected to the ground of the board to close the circuit accordingly.

**Motor**

The motor button connects or disconnects the anti-spark switch device. The later device is directly connecting the external battery poles underlined in the *battery system* description to the VESC -Electronic speed controller- device. The VESC is in turn connected to the vehicle throttle motor.

### **Joystick Controller and Receiver**

![rf.png](https://github.com/vniclas/freicar_docs/raw/master/images/hardware_documentation/rf.png)

- The RF receiver has two channels. These are connected to the base board. The two channels are marked on the board as throttle and steering.

- The base board outputs the processed input signals to the throttle motor through communication with the VESC and directly to the servo motor.

**Joystick**

![joystick.png](https://github.com/vniclas/freicar_docs/raw/master/images/hardware_documentation//joystick.png)

- First turn the joystick on by sliding the switch to the on state.
- There exists three analog knobs on the joystick. These set an initial offset for steering, the steering sensitivity, and an initial offset for the throttle.

![joystick_side.png](https://github.com/vniclas/freicar_docs/raw/master/images/hardware_documentation/joystick_side.png)

- The large side analog knob controls the steering of the vehicle. Rotate it to left or right to rotate the wheel axle accordingly.
    
    
- The trigger sends throttle signals:
    - Pull the trigger to move the vehicle forward.
    - Push it to the front to move the vehicle in reverse.
    - Velocity is controlled by how far you push or pull.

### **Servo Motor**

![servo.png](https://github.com/vniclas/freicar_docs/raw/master/images/hardware_documentation/servo.png)

The servo motor has a rotating knob. A plastic flat rod is fitted on top of knob to connect the servo to the wheel axle mechanically.

![servo_rod.png](https://github.com/vniclas/freicar_docs/raw/master/images/hardware_documentation/servo_rod.png)

To set the neutral angle of the wheel axle:

- Remove the plastic rod from the knob.
- Turn on the car to know the neutral state of the knob
- Rotate the wheel axle to the desired position.
- Reconnect the plastic rod. The rod can be elongated or shortened by rotating it around its axis.

### **System Description**

The vehicle can be controlled by an RF joystick. It has a base board to route power, send and receive control signals. It additionally has an electronic speed controller that manages the higher power needed to run the motor by receiving the communication signal from the base board.

#### Components Functionalities

**Base board**

![board_annotated.png](https://github.com/vniclas/freicar_docs/raw/master/images/hardware_documentation/board_annotated.png)

- Processes and controls signals through a mounted itsy-bitsy arduino microcontroller.
- Raises different alarms through a mounted buzzer.
- Receives the input throttle and steering signals from the *RF* controller.
- Sends the output communication signal to the *electronic speed controller*.
- Sends the output steering signal to the *servo motor*.
- Acts as the first routing stage for the input power (Adapter or *PC battery through the switch*).
- Sends the ignition signal and power to the *PSU*.
- Measures current (not currently used)

**Arduino Microcontroller**

![microcontroller.png](https://github.com/vniclas/freicar_docs/raw/master/images/hardware_documentation/microcontroller.png)

Mounted on the base board and connected to the PC using USB.

- Responsible for processing the communication signal sent to the *electronic speed controller*.
- Controls the buzzer alarm system:
    - Raises an alarm with a certain frequency for a dropped communication with the *electronic speed controller*.
    - Raises a lower frequency alarm for warning against low battery charge state.

**Motherboard**

- Interfaces all the PC components and the GPU.

**PSU**

- Routes different levels of power to the motherboard through two groups of cables:
    - 24 pin power connector.
    - 4 pin 12 V connector for the fan.
- Switches on the motherboard through a two jumper switch.
