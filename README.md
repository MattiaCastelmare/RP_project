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
## How to set up workspace and clone repository
Run the following commands in a terminal:
\
Go in the root folder and create the folder catkin_icp_ws/src
   ```sh
   cd ~
   
   mkdir -p catkin_icp_ws/src
   ```
Go inside the catkin_icp_ws/src and create your catkin workspace
   ```sh 
   cd catkin_icp_ws/src
   
   catkin_init_workspace
   ```

Clone this repository inside the catkin_icp_ws/src folder
   ```sh
   git clone https://github.com/MattiaCastelmare/RP_project.git
   ```
Go into the catkin_icp_ws folder and build the project
   ```sh
   cd ~/catkin_icp_ws/
   
   catkin build
   ```
## How to test the project
To test the project
