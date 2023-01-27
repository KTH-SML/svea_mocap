# svea_mocap

This repository contains convenience launch files and python class for
integrating the Qualisys Motion Capture (mocap) System into the SVEA software
stack. The repository has two main examples to demonstrate:
1. Using mocap as localization for experiments
2. Using mocap to evaluate new localization methods (i.e. slam or via aruco markers)

## Installation

Please refer to [svea](https://github.com/KTH-SML/svea) on how to install and
run SVEA software.

Once you have set up your workspace based on [svea](https://github.com/KTH-SML/svea),
in addition to cloning this package, you also need to clone the
[motion\_capture\_system](https://github.com/KTH-SML/motion_capture_system) repository,
which is the main driver for ROS-ifying data from the motion capture system.

## Usage

For setting up the motion capture system, contact one of the lab's
research engineers who works with the system.

Once you have gotten help to set up your environment on the motion
capture system, you can proceed to one of the following examples.

**Note:** For these examples to work, you need to be on the same network as
the motion capture system.

### Testing connection to mocap

To test if you can connect to the mocap system in the first place, try
running

```bash
roslaunch mocap_qualisys qualisys.launch
```

If no errors or warnings pop-up, you can then try calling `rostopic list`
and you should see all of the subjects that are published by the mocap
system.

### Using mocap as localization for experiments

To try out using mocap as localization, try running the following launch
file with the <subject name> replaced:

```bash
roslaunch svea_mocap mocap_only mocap_name:=<subject name>
```

### Using mocap to evaluate new localization methods

To try out comparing indoor localization with mocap, try running the following
launch file with the <subject name> replaced:

```bash
roslaunch svea_mocap localization_comparison mocap_name:=<subject name>
```

**Note:** By default the example supports the use of the RC remote for driving the
vehicle around while recording the inputs from the RC remote.
