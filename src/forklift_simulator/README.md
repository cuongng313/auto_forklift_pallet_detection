## Forklift_Simulator
Simulated forklift &amp; sensors (2D&amp;3D Lidars + RGBD-camera) in warehouse environment

### Installation
- dependencies:
>sudo apt install ros-noetic-velodyne-gazebo-plugins\
sudo apt install ros-noetic-gazebo-plugins\
sudo apt install ros-noetic-gazebo-ros-control\
sudo apt-get install ros-noetic-joint-state-controller\
sudo apt-get install ros-noetic-effort-controllers\
sudo apt-get install ros-noetic-position-controllers\
sudo apt-get install ros-noetic-velocity-controllers\
sudo apt-get install ros-noetic-gazebo-ros-pkgs

- add export gazebo path to your .bashrc
>export GAZEBO_MODEL_PATH=<path_to_simulator_package>/forklift_simulator/models:$GAZEBO_MODEL_PATH

- add realsense gazebo library to your workspace 
>git clone https://github.com/pal-robotics/realsense_gazebo_plugin.git

- build your workspace with catkin_make

### Launch
- To test simulation of **forklift EXV_14**, run:
>roslaunch forklift_simulator forklift_simulation.launch\
rosrun forklift_simulator keyboard_control_forklift

- To **spawn the container**, run:
>roslaunch forklift_simulator container_simulation.launch\
rosrun forklift_simulator keyboard_control_container

- To **visualize** on rviz, run:
>roslaunch forklift_simulator visualize.launch
