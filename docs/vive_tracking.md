# Vive Tracking (Indoor Localization)

We use an HTC Vive tracking setup to obtain indoor ground-truth localization for recording maps and steering the cars. Follow this procedure to activate the tracking hardware:

1. Turn on at least 4 **lighthouses** at the power adapter (check green LED) 

    <img src="https://github.com/vniclas/freicar_docs/raw/master/images/vive_lighthouse2.jpg" alt="lighthouse" width="400"/>

2. Turn on fixed **reference tracker** and then plug in power (after green LED)

    <img src="https://github.com/vniclas/freicar_docs/raw/master/images/vive_tracker.jpg" alt="lighthouse" width="200"/>

3. Place **glasses** in center on some box, facing the ceiling (press blue button on connector box if no green light) 

    <img src="https://github.com/vniclas/freicar_docs/raw/master/images/vive_headset.png" alt="lighthouse" width="400"/>

4. Turn on **tracker on the car (or the mapping device)**: On the car, first unplug its power connection (either tracking OR charging), then press its button until green LED lights up. A flashing red LED denotes the need for charging.

    <img src="https://github.com/vniclas/freicar_docs/raw/master/images/vive_tracker.jpg" alt="lighthouse" width="200"/>

5. Login on *freicar-base* 
6. Check that the machine is connected to the network. If not, unplug and re-plug the ethernet cable.
6. Start standalone Steam as a background process
7. Please ignore all warnings and errors issued by Steam. 
8. Start SteamVR → check that 4 lighthouses, glasses, reference tracker and the number of car-wise trackers appear green. A little flash icon tells you that one of the trackers needs charging.
