# fetch-sim2real project

## Install

### Install dependencies
Make sure that you have [Ubuntu 18.04](https://releases.ubuntu.com/18.04/) and [Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) installed on your computer. Then install the necessary packages to run the simulation environment with the fetch robot.

```
sudo apt-get update
sudo apt-get install ros-melodic-fetch*
```
This will install all of the packages related to fetch. Also, the octomap dependencies need to be installed and this can be done running the following commands.
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
* octomap (pip install octomap-python --no-binary octomap-python & sudo apt-get install libdynamicedt3d*)

### Build
Once your system is set up, clone the [fetch_sim2real](https://github.com/alan-sanchez/fetch_sim2real.git) to your workspace and build the package in your workspace. Then source the workspace in your bash file. This can be done by copying the commands below and pasting them into your terminal.

```bash
cd ~/catkin_ws/src
git clone https://github.com/alan-sanchez/fetch_sim2real.git
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
source ~/.bashrc
```

## Getting
Run the below launch files and python node into separate terminals to get things started.

```bash
# Terminal 1
roslaunch fetch_sim2real short_table_gazebo.launch
```

This should bringup Fetch in the gazebo environment with a table in front of it. The arm should also be in the up configuration.
In the second terminal you should run the rviz_setup.launch file. This will open RViz which will visualize the disinfection region, depth map, and waypoints.
```bash
# Terminal 2
roslaunch fetch_sim2real rviz_setup.launch
```
Within the same terminal, press "Enter" on your keyboard and this will generate a random polygon (with a predefined range as to not to exceed the physical limitations of the Fetch robot) and initialize the robot to start disinfecting. Below is a gif for reference.

![](images/reference.gif)
