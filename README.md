# Kinova Arm Controller - OpenTeach
This repository contains the information to setup the ROS Noetic based controller part of [OpenTeach](https://open-teach.github.io/) to control the Kinova Arm. 

## Contents
1. [Requirements](#requirements)
2. [Launching the Controller](#launch-controller)

## Installation Requirements <a name="requirements"></a>
1. Kinova SDK
1. Kinova JACO Arm

## Installing the Kinova SDK
We need to setup the Kinova SDK before we make the C binaries for the controller. You can download the SDK from https://drive.google.com/u/0/uc?id=1UEQAow0XLcVcPCeQfHK9ERBihOCclkJ9&export=download. You should be able make the controller binaries and run the controller after this step.

## Launching the Controller <a name="launch-controller"></a>
After installing the Kinova SDK,  install this repo within the base controller package and run catkin_make from the [base](https://github.com/NYU-robot-learning/OpenTeach-Controllers) directory (where you have both - Kinova JACO arm controller and Allegro Hand controller). Do the following from this directory:
```
cd <base-controller-dir>
catkin_make
```
After you make the binaries, you need to source the setup files using the below command:
```
source <base-controller-dir>/devel/setup.bash
```
Then you can launch the Kinova Arm's roslaunch file to start controlling the robot.
```
roslaunch kinova_arm kinova_robot.launch
```

```
@misc{iyer2024open,
      title={OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation}, 
      author={Aadhithya Iyer and Zhuoran Peng and Yinlong Dai and Irmak Guzey and Siddhant Haldar and Soumith Chintala and Lerrel Pinto},
      year={2024},
      eprint={2403.07870},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
