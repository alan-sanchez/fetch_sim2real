# fetch-sim2real project

## Install

### Install dependencies
Install the necessary packages to run a simulation environment with a fetch robot.

```
sudo apt-get update
sudo apt-get install ros-melodic-fetch*
```

You also need to install the rviz_visual_tools for the cone marker. Further information [here](https://github.com/PickNikRobotics/rviz_visual_tools/blob/melodic-devel).
```
sudo apt-get install ros-melodic-rviz-visual-tools
```

The octomap dependencies need to be installed.
```
sudo apt-get install ros-melodic-octomap
sudo apt-get install ros-melodic-octomap-server
sudo apt-get install ros-melodic-octomap-mapping
```

You also need to pip and pip3 install:
* rospkg
* scipy
* sympy
* planar
* pyvista
* shapely

### Build
Add the package to your src file in your workspace.

```bash
git clone https://github.com/alan-sanchez/fetch_sim2real.git
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
source ~/.bashrc
```

## Getting Started
Run the below launch files and python node into separate terminals to get things started.

```bash
# Terminal 1
roslaunch fetch_sim2real short_table_gazebo.launch
```

This should bringup Fetch in the gazebo environment with a table in front of it. The arm should also be in the up configuration.
In the second terminal you should run the rviz_setup.launch file to begin the UV disinfection.
```bash
# Terminal 2
roslaunch fetch_sim2real rviz_setup.launch
```

Below is a gif for reference.

![](images/reference.gif)
