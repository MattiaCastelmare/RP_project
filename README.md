# RP_project
The aim of this project is to develop a ICP-Based Localizer system compliant with the ROS environment. 

## Requirements
Before running the project be sured to have installed the following packages:

1) Map Server
\
Install the `ros-${DISTRO}-map-server` package. In our case (valid for Lattinone VM) we are using _ROS Noetic_ running the following command:
   ```sh
    sudo apt install ros-noetic-map-server
   ```
2) Stage-ROS
\
Install the `ros-${DISTRO}-stage-ros` and `ros-${DISTRO}-teleop-twist-keyboard` package. As above, we are using _ROS Noetic_ running the following command:
     ```sh
     sudo apt install ros-noetic-stage-ros ros-noetic-teleop-twist-keyboard
     ```
## How to test
Run the following commands in a terminal:
```sh
cd ~
```
```sh
mkdir -p catkin_ws/src
```
```sh
cd catkin_ws/src
```
```sh
catkin_init_worskpace
```
```sh
https://github.com/MattiaCastelmare/RP_project.git
