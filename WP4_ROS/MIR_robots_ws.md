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



You should add the following line to the end of your `~/.bashrc`, and then close and reopen all terminals:
```ruby
source ~/MIR_robots_ws/devel/setup.bash
```
> [!NOTE]
> `MIR_robots_ws` is your workspace name; you can replace it with any name you prefer.









