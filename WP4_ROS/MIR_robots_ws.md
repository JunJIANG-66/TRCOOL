# Installation

In the GitHub repository of [`mir_robot`](https://github.com/DFKI-NI/mir_robot), two methods are provided for installing the ROS driver and ROS configuration for MiR robots. However, the binary installation is currently unavailable due to certain issues. Our installation process is inspired by the instructions provided in the above GitHub repository.

The instructions below use the ROS distro `noetic` and `Ubuntu 20.02` as an example; if you use a different distro (e.g. `melodic`), replace all occurrences of the string `noetic` by your distro name in the instructions.

## Package overview (Pending)

- `mir_actions`: Action definitions for the MiR robot
- `mir_description`: URDF description of the MiR robot
- `mir_dwb_critics`: Plugins for the dwb_local_planner used in Gazebo
- `mir_driver`: A reverse ROS bridge for the MiR robot
- `mir_gazebo`: Simulation specific launch and configuration files for the MiR robot
- `mir_msgs`: Message definitions for the MiR robot
- `mir_navigation`: move_base launch and configuration files

## Preliminaries
If you haven't already installed ROS on your PC, you need to add the ROS apt repository. This step is necessary for either binary or source install.

```ruby
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq
```

## Binary install
For a binary install, it suffices to run this command:
```ruby
sudo apt install ros-noetic-mir-robot
```

> [!CAUTION]
> Binary install unavailable

## Source install
For a source install, run the commands below instead of the command from the "binary install" section.
```ruby
# create a catkin workspace
mkdir -p ~/MIR_robots_ws/src
cd ~/MIR_robots_ws/src/

# clone mir_robot into the catkin workspace
git clone -b noetic https://github.com/DFKI-NI/mir_robot.git

# use rosdep to install all dependencies (including ROS itself)
sudo apt-get update
sudo apt-get install -y python3-rosdep
sudo rosdep init
rosdep update --include-eol-distros
rosdep install --from-paths ./ -i -y --rosdistro noetic

# build all packages in the catkin workspace
source /opt/ros/noetic/setup.bash
catkin_init_workspace
cd ~/MIR_robots_ws
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebugInfo
```
> [!WARNING]
> For `rosdep install --from-paths ./ -i -y --rosdistro noetic`  
> ERROR: Rosdep experienced an error: manifest [/home/jun/gazebo_mrt/models/sun/manifest.xml] must have a single 'package' element

> [!TIP]
> Ignore the ERROR.


Retry the `catkin_make` in the `MIR_robots_ws` folder.
```
cd ~/MIR_robots_ws
catkin_make
```
> [!WARNING]
> The ERROR information is
> 
> Could not find a package configuration file provided by "costmap_queue"
  with any of the following names:  
> costmap_queueConfig.cmake  
> costmap_queue-config.cmake

> [!NOTE]
> Run the following code to install `costmap_queue` in ROS NOETIC
```
cd ~/MIR_robots_ws/src
git clone https://github.com/locusrobotics/robot_navigation.git
```

And then, retry the  `catkin_make` in the `MIR_robots_ws` folder. You will get the following ERROR:

> [!WARNING]
> Could not find a package configuration file provided by "map_server" with any of the following names:  
> map_serverConfig.cmake  
> map_server-config.cmake

You can run the folowing code to install `map_server` in ROS NOETIC in Ubuntu 20.04:
```bash
sudo apt update
sudo apt install ros-noetic-map-server
```
The next ERROR is that 
> [!WARNING]
> Could not find a package configuration file provided by "move_base_msgs"
  with any of the following names:
> move_base_msgsConfig.cmake
> move_base_msgs-config.cmake

The following code can fix it:
```bash
sudo apt install ros-noetic-move-base-msgs
```

For convenience, I will no longer list all error messages, but only the code that needs to be run.
```bash
sudo apt-get install ros-noetic-rospy-message-converter
sudo apt-get install ros-noetic-costmap-2d
sudo apt-get install ros-noetic-navigation
```

After fixed all the ERRORs, run
```bash
cd ~/MIR_robots_ws
catkin_make
echo "source ~/MIR_robots_ws/devel/setup.bash" >> ~/.bashrc
```

> [!CAUTION]
> The ERROR above can be fixed by two steps:  
> RUN `sudo apt-get install ros-noetic-navigation`  
> RUN `sudo apt-get install ros-noetic-rospy-message-converter`



You should add the following line to the end of your `~/.bashrc`, and then close and reopen all terminals:
```ruby
source ~/MIR_robots_ws/devel/setup.bash
```
> [!NOTE]
> `MIR_robots_ws` is your workspace name; you can replace it with any name you prefer.


## Gazebo demo

You can launch the fellowing code to do the Demo in Gazebo and RViz.

```bash
### gazebo:
roslaunch mir_gazebo mir_maze_world.launch
rosservice call /gazebo/unpause_physics   # or click the "start" button in the Gazebo GUI

### localization:
roslaunch mir_navigation amcl.launch initial_pose_x:=10.0 initial_pose_y:=10.0
# or alternatively: roslaunch mir_gazebo fake_localization.launch delta_x:=-10.0 delta_y:=-10.0

# navigation:
roslaunch mir_navigation start_planner.launch \
    map_file:=$(rospack find mir_gazebo)/maps/maze.yaml \
    virtual_walls_map_file:=$(rospack find mir_gazebo)/maps/maze_virtual_walls.yaml
rviz -d $(rospack find mir_navigation)/rviz/navigation.rviz
```






