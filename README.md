# Baxter Pick and Place

## Prerequisites
- A developer workstation meeting the minimum system requirements is available for use.
- The developer workstation has been set up successfully with Ubuntu and ROS - instructions here.
- You have configured your Development Workstation to access Github. Instructions here.

## Installation
- Download and Install the SDK dependencies
```bash
$ sudo apt-get install python-wstool python-rosdep
```

- Create a new catkin workspace.
```bash
$ mkdir -p ~/ros_ws/src
$ cd ~/ros_ws/src
```

- Download and copy the rosinstall file to the root of your Catkin workspace using wstool.
```bash
$ wstool init .
$ wstool merge https://raw.github.com/RethinkRobotics/baxter/master/baxter_sdk.rosinstall
$ wstool update
$ cd ..
```
This pulls down the development branches from all of our repositories into your source directory.

For Baxter Gazebo simulation, checkout the baxter_simulator repository:
```bash
$ cd ~/ros_ws/src
$ git clone https://github.com/RethinkRobotics/baxter_simulator.git
```

- Check out the Baxter PNP package into your ROS workspace
```bash
$ cd ~/ros_ws/src
$ git clone https://github.com/skumra/baxter-pnp.git
```

- Source ros setup if you haven't yet
```bash
$ source /opt/ros/kinetic/setup.bash
```

- Build and Install
```bash
$ cd ~/ros_ws
$ catkin_make
$ catkin_make install
```

- Use the baxter.sh script for proper environment setup
```bash
$ cp src/baxter/baxter.sh .
```

## Run PNP in Simulation
- Run the baxter.sh script with sim specified
```bash
$ cd ~/ros_ws
$ ./baxter.sh sim
```

- Start simulation with controllers
```bash
$ roslaunch baxter_gazebo baxter_world.launch
```

- Start pick and place
```bash
$ cd ~/ros_ws
$ source devel/setup.bash
$ rosrun baxter-pnp run_pnp.py
```

## Run PNP on Robot
- Run the baxter.sh
```bash
$ cd ~/ros_ws
$ ./baxter.sh
```

- Start pick and place
```bash
$ cd ~/ros_ws
$ source devel/setup.bash
$ rosrun baxter-pnp run_pnp.py
```
