## Pallet_detection
Detect and generate postion of pallets

### Installation
- dependencies:
>sudo apt-get install ros-noetic-cv-bridge

### Launch
- Change the value of camera resolution:
Open file: forklift_simulator/urdf/RX20_16/d435_sensor.xacro <br />
Change all the values of width and height: <br />
    width: 848 <br />
    height: 480

- If use simulation: 
Set the value of the argument **"use_simulation"** to **true** in multi_pallet_detection/launch/pallet_detection.launch
>roslaunch multi_pallet_detection pallet_detection.launch

- If use rosbag data: 
Set the value of the argument **"use_simulation"** to **false** in multi_pallet_detection/launch/pallet_detection.launch
>roslaunch multi_pallet_detection pallet_detection.launch

- Launch the visualization: 
>roslaunch multi_pallet_detection visualize.launch
