# Automodel-CIC-IPN
Welcome to the CIC's AutoModel Car gitHub. This repository contents the code for the next modules:

- Image processing and camera adjustment (C++ and Python).
- Lane follower (C++ and Pyhton).
- Intersection detector (C++ and Pyhton).
- Obstacke detection (C++ and Pyhton).
- Joystick (C++).

The ROS distro used is Indigo along with Ubuntu 14.10 LTS. All the C++ and Pyhton Modules are included in different ROS packages. The launch files are also included.

## Packages installation:

**IMPORTANT: Before starting, make sure you have ROS and a catkin_ws workspace already configured on your PC! Otherwise, visit the [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).**

TODO

## Cloning the repository

> git clone https://github.com/Conilo/automodelcar-cic-ipn.git

## Build the code:
First, it's necesarry to compile the code using (also after modifiying any file or node source). To compile the code:

> bash compile.bash

To compile the code en "release mode" add the `-s` flag:

> bash compile.bash -s

## Run the code
The cic package contains different launch files in order to execute specific tasks. The next subsecions explains deeply all the launch files intended fucntions.

### Camera adjustment

To run the camera adjustment launch, type:

> roslaunch cic camera_adjustment.launch

A debug window will be displayed with a chesboard layout (see figure 1) and using a chesboard pattern of 35x35 [cm], it's possible to modify the parameters in the launch in order to match the chesboard pattern with the one displayed:

- Pixel to cm ratio in the X-axis.
- Pixel to cm ratio in the Y-axis.
- Scaling factor for the X-axis.
- Scaling factor for the Y-axis.
- Four points to wrap the image in birdview.

![](img/calibration_window.png)
Figure 1: Chessboard pattern displayed on camera adjustmen mode.

### Autonomous mode
TODO

## Contact:
If you need more info about the code, please contact:

* Project Coordinator:
Erik Zamora Gómez (e-mail: ezamora1981@gmail.com).

* Project Manager: 
Cesar Gerardo Bravo Conejo  (e-mail: conilo@gmail.com).

Student assistants:
- Brenda Camacho Garcia (e-mail: brendacg616@gmail.com).
- Esteban Iván Rojas Hernández (e-mail: rojasesteban23@gmail.com).
