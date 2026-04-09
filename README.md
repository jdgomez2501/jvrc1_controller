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
git clone https://github.com/jdgomez2501/jvrc1_controller.git
```
Adjust the path if your superbuild uses a different workspace layout.

### 2) Building and using the controller (Standalone setup)

After cloning the repository inside the `workspace`, build and install the controller:

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

Source the setup file produced when building mc_rtc_superbuild, usually you already added this to your .zshrc file as recommended after building mc_rtc_superbuild, if not, you can do it manually running for example:

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

In another terminal properly sourced, run the controller with the path of the config file created previosuly when building and installing. It is a MUST to specify this config file, otherwise the controller will not be loaded

```bash
mc_rtc_ticker -f <path_to_your_workspace>/install/lib/mc_controller/etc/JVRC1Controller.yaml
# For example mc_rtc_ticker -f /home/vscode/workspace/install/lib/mc_controller/etc/JVRC1Controller.yaml
# Or mc_rtc_ticker -f ~/mc_rtc_superbuild/workspace/install/lib/mc_controller/etc/JVRC1Controller.yaml
```
The `-f` option lets mc_rtc_ticker load a specific mc_rtc configuration file.

In case of having problems finding your installed config file you can use directly the template included in the source file by running this:

```bash
mc_rtc_ticker -f <path_to_your_workspace>/jvrc1_controller/etc/JVRC1Controller.in.yaml
```
## Expected behavior

The controller repeats the following sequence:

1. The left-hand moves to (0.5, 0.25, 1.1) with orientation (0, 0.7, 0, 0.7) (w, x, y, z)
2. The left-hand moves back to its initial position
3. The right-hand moves to (0.5, -0.25, 1.1) with orientation (0, 0.7, 0, 0.7)
4. The right-hand moves back to its initial position
5. Both hands move to their respective specified position
6. Both hands go back to their initial position
7. Repeat

While a single hand is moving the robot should look at the moving hand, when both hands are moving the robot should look forward


## Notes

- The controller is written in C++ and uses mc_rtc tasks and constraints.
- Configuration values are loaded from the YAML file instead of being hardcoded.
- These values were tunned by experimentation to have an acceptable trade-off between good trade-off between target tracking and stability. These values can be further tuned to achieve better performance, but they are good enough for the current application.

## Video

