# JVRC-1 Controller

A C++ mc_rtc controller for the JVRC-1 humanoid robot.

## Overview

This controller:
- Keeps the left and right feet in contact with the ground,
- Enforces self-collision avoidance, joint limits, and equations of motion,
- Moves the left hand and right hand through a repeating sequence,
- Implements a lookAt task to follow hands trajectory.

## Repository layout

- `src/` — controller implementation
- `etc/` — mc_rtc configuration files
- `CMakeLists.txt` — build configuration
- `README.md` — this file

## Requirements

- A working mc_rtc installation
- JVRC-1 robot support available in your mc_rtc setup
- C++17-compatible compiler
- ROS support for RViz visualization if you want the graphical display

## Run with mc_rtc_superbuild

This is the recommended installation path if you already use `mc_rtc_superbuild`.

### 1) Place the controller in your workspace

Clone this repository inside the `workspace/` directory of your `mc_rtc_superbuild`, for example:

```bash
cd <path_to_your_workspace>
#For example cd ~/mc_rtc_superbuild/workspace
git clone <your-repo-url> jvrc1_controller
```
Adjust the path if your superbuild uses a different workspace layout.

### 2) Building and using the controller (Standalone setup)

After cloning the repository inside the `workspace` folder of `mc_rtc_superbuild`, build and install the controller:

```bash
cd <path_to_your_workspace>/jvrc1_controller
mkdir -p build
cd build
cmake ../ -DCMAKE_BUILD_TYPE=RelWithDebInfo
make
make install
#Note: if mc_rtc is installed in a privileged directory sudo is required
```

### 3) Source the environment

Source the setup file produced by your build, usually you already added this to your .zshrc file as recommended after building mc_rtc_superbuild, if not, you can do it manually running for example:

```bash
source <path_to_your_workspace>/install/setup_mc_rtc.sh
```


### 4) Start RViz

Use the launch command:

```bash
# ROS 2
ros2 launch mc_rtc_ticker display.launch
```

### 5) Run the controller with its custom config

```bash
mc_rtc_ticker -f <path_to_your_workspace>/install/lib/mc_controller/etc/JVRC1Controller.yaml
# For example mc_rtc_ticker -f /home/vscode/workspace/install/lib/mc_controller/etc/JVRC1Controller.yaml
# Or mc_rtc_ticker -f ~/mc_rtc_superbuild/workspace/install/lib/mc_controller/etc/JVRC1Controller.yaml
```

The `-f` option lets mc_rtc_ticker load a specific mc_rtc configuration file.

## Expected behavior

The controller repeats the following sequence:

1. move the left hand to the target pose,
2. bring the left hand back,
3. move the right hand to the target pose,
4. bring the right hand back,
5. move both hands to their targets,
6. bring both hands back,
7. repeat.

When one hand is moving, the robot looks at the moving hand. When both hands move, the robot looks forward.

## Notes

- The controller is written in C++ and uses mc_rtc tasks and constraints.
- Configuration values are loaded from the YAML file instead of being hardcoded.

## Video

