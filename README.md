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

