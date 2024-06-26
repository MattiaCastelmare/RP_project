# Robot Programming project "ICP localization"
The aim of this project is to develop a ICP-Based Localizer system compliant with the ROS environment. 

## Requirements
Before running the project be sured to have installed the following packages:

### 1)  Map Server
\
Install the `ros-${DISTRO}-map-server` package. In my case (valid for Lattinone VM) I am using _ROS Noetic_, run the following command:
   ```sh
    sudo apt install ros-noetic-map-server
   ```
 ###  2) Stage-ROS
\
Install the `ros-${DISTRO}-stage-ros` and `ros-${DISTRO}-teleop-twist-keyboard` package. As above, I am using _ROS Noetic_ run the following command:
  ```sh
  sudo apt install ros-noetic-stage-ros ros-noetic-teleop-twist-keyboard
  ```

## How to set up workspace and clone repository
In the root folder create the folder catkin_icp_ws/src:
   ```sh
   cd ~
   
   mkdir -p catkin_icp_ws/src
   ```
Go inside the catkin_icp_ws/src and create your catkin workspace:
   ```sh 
   cd catkin_icp_ws/src
   
   catkin_init_workspace
   ```

Clone this repository inside the catkin_icp_ws/src folder:
   ```sh
   git clone https://github.com/MattiaCastelmare/RP_project.git
   ```
Go into the ~/catkin_icp_ws/ folder and build the project:
   ```sh
   cd ~/catkin_icp_ws/
   
   catkin build
   ```
## How to test the project
Open a terminal and navigate inside the ~/catkin_icp_ws/ by typing:
```sh
cd ~/catkin_icp_ws/
```
Everytime a terminal is opened source the project by the command:
```sh
source devel/setup.bash
```
1) Open a terminal go into the ~/catkin_icp_ws/ and source it then run:
   ```sh
   roscore
   ```
2) Open a terminal go into the ~/catkin_icp_ws/ and source it then run:
   ```sh
   cd ~/catkin_icp_ws/src/RP_project/02_icp_localization/
   ```
   Run the map_server node:
   ```sh
   rosrun map_server map_server test_data/cappero_map.yaml
   ```
3) Open a terminal go into the ~/catkin_icp_ws/ and source it then run:
   ```sh
   cd ~/catkin_icp_ws/src/RP_project/02_icp_localization/
   ```
   Launch the rviz configuration
   ```sh
   rviz -d test_data/rviz.rviz
   ```
4) Open a terminal go into the ~/catkin_icp_ws/ and source it then run:
   ```sh
   cd ~/catkin_icp_ws/src/RP_project/02_icp_localization/
   ```
   Launch the simulator, launch the roscore and the 'stageros' node:
   ```sh
   rosrun stage_ros stageros test_data/cappero.world
   ```
5) Open a terminal go into the ~/catkin_icp_ws/ and source it the run the localizer node:
   ```sh
   rosrun icp_localization localizer_node
   ```
At this point, open **RViz** and set the initial position and orientation of the robot close to the real one. Then, open the simulator and move the robot with the mouse. In **RViz**, you should see the odometry of the robot and its scan of the obstacles as you move the robot.
\
The following is an example of the results obtained:

<p align="center">
  <img src="https://github.com/MattiaCastelmare/RP_project/assets/94857717/894dce46-0795-40b9-80a8-a875d903d10e" alt="ICP Localization Demo">
</p>

