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
Install the pip3 modules:
```bash
pip3 install pyvista
pip3 install os
pip3 install sys
pip3 install pandas
pip3 install octomap-python --no-binary octomap-python

```
If you are having issues running octomap, it may needs other dependencies. If so, use this:

```bash
sudo apt-get install libdynamicedt3d*
```

Install pip module:
```bash
pip install scipy
```

### Build
Once your system is set up, clone the [fetch_sim2real](https://github.com/alan-sanchez/fetch_sim2real.git) to your workspace and build the package. Then source the workspace in your bash file. This can be done by copying the commands below and pasting them into your terminal.

```bash
cd ~/catkin_ws/src
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

# If you want to run the simulation and visualizer headless, then use this command below.
roslaunch fetch_sim2real short_table_gazebo.launch

# If you want only want RViz, use the command below.
roslaunch fetch_sim2real short_table_gazebo.launch rviz:=true

# If you want the simulation and rviz, use the command below.
roslaunch fetch_sim2real short_table_gazebo.launch rviz:=true gui:=true headless:=false
```

This should bringup Fetch in the gazebo environment with a table in front of it. The arm should also be in the up configuration.
In the second terminal the nodes.launch file. This will activate the rest of the nodes that will do the simulation.
```bash
# Terminal 2
roslaunch fetch_sim2real nodes.launch
```
Within the same terminal, press "Enter" on your keyboard and this will generate a random polygon (with a predefined range as to not to exceed the physical limitations of the Fetch robot) and initialize the robot to start disinfecting. Below is a gif for reference.

![](images/reference.gif)
