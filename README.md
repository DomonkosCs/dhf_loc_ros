# DHF_LOC Package
ROS wrapper for the [DHF localization algorithm](https://github.com/DomonkosCs/dhf-localization). Acts as a ROS package, built for ROS noetic.

## Contact
See in the README of the [library](https://github.com/DomonkosCs/dhf-localization).

## Installation

The package is only tested inside the the workspace of [dhf_loc_ws](https://github.com/DomonkosCs/dhf_loc_ws). See [dhf_loc.rosinstall](https://github.com/DomonkosCs/dhf_loc_ws/blob/main/src/dhf_loc.rosinstall) for the required packages. 
After cloning the workspace, you can install them via
` vcs import < dhf_loc.rosinstall`, which requires the `python3-vcstool` package.

**Important**: This package contains the [DHF localization algorithm](https://github.com/DomonkosCs/dhf-localization) library as a git submodule. After cloning this repo (either directly, or from the `.rosinstall` file),
you have to run the following commands inside the root of this repository (where the `.gitmodules` file is located)
```
git submodule init
git submodule update
```

## Usage

### Demo
For a demo, launch the `dhf_loc/launch/dhf_localization_tb3.launch` file. This starts a Gazebo simulation, and localizes the TurtleBot3 robot in the TurtbleBot3 House environment, using the Mean Exact Particle Flow Daum--Huang filter (MEDH).
Details of the algorithm can be found in the [paper](https://github.com/DomonkosCs/dhf-localization/blob/main/paper.pdf). You can move the robot around by running
```
rosrun turtlebot3_teleop turtlebot3_teleop_key
```

The parameters of the package are documented inside `dhf_loc/src/dhf_localization_node.py`, and can be changed in `dhf_loc/config/dhf_localization_node/simu.yaml`.

### Reproducing the paper

The results in the paper are acquired by running the dhf_loc/launch/dhf_localization_<VAR>_bag.launch files, where <VAR> corresponds to the inspected localization algorithm. For details, see the comments inside the launch files.
