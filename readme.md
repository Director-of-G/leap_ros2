
# Leap ROS2 Interface

- This repository contains code for running the [LEAP Hand](http://leaphand.com/).


## Table of Contents

- [Dependencies](#dependencies)
- [Setup Instructions](#setup-instructions)
- [Running the Leap Hand](#running-the-leap-hand)
- [Debugging](#debugging)

## Dependencies
The table below lists the direct dependencies needed for this repository.

| **Dependency Name**                                                          | **Description**                                                                   |
|------------------------------------------------------------------------------|-----------------------------------------------------------------------------------|
| [dynamixel-sdk](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)                          | DYNAMIXEL SDK is a software development kit that provides DYNAMIXEL control functions using packet communication.       |

## Setup Instructions

Follow these steps to set up the Leap Hand:

1. **Clone the Repository**:
   ```bash
   git clone git@github.com:BiomechatronicsLab/leap_ros2.git
   cd your-repo-directory
   ```

2. **Install Dependencies**:
   Ensure you have Python packages required for the project:
   ```bash
   pip install -r requirements.txt
   ```
   
4. **Compile the Package**:
   Build the package using `colcon`:
   ```bash
   colcon build --packages-select leap_ros2 --symlink-install
   ```

## Running the Leap Hand

1. **Connect USB**
   Connect USB to Leap Hand to Computer
   
2. **Create and Edit Configuration File**:
   Copy the default parameters:
   ```bash
   cp path/to/default_params.yaml path/to/your/config.yaml
   ```
   Modify `config.yaml` as needed to set your desired parameters.

4. **Launch the Driver**:
   Execute the following command, specifying the path to your configuration file:
   ```bash
   ros2 launch leap_ros2 launch_leap.py config_file:=leaphandName.yaml
   ```

5. **Test Demo**:
   Execute the following command, to check if the leap hand will move each of its fingers!
   ```bash
   ros2 run leap_ros2 demo_repeat_joint_data.py
   ```

## Debugging

1. After plugging in the USB from the Leap into your computer, use the following if necessary to determine the serial ID associated with the appropriate hand: ```cd /dev/serial/by-id/ ```






## Welcome to the LEAP Hand SDK
- Please visit [our website](http://leaphand.com/) for more information about LEAP hand.
#### Software Setup
- Please see the [Python API](https://github.com/leap-hand/LEAP_Hand_API/tree/main/python), [ROS API](https://github.com/leap-hand/LEAP_Hand_API/tree/main/ros_module), [Useful Tools](https://github.com/leap-hand/LEAP_Hand_API/tree/main/useful_tools) folders for software specific details.

#### Hardware Setup
- Connect 5v power to the hand (the dynamixels should light up during boot up.)
- Connect the Micro USB cable to the hand (Do not use too many USB extensions)
- Open [Dynamixel Wizard](https://emanual.robotis.com/docs/en/software/rplus1/dynamixel_wizard/) and find the correct port using the options button and put that in main.py or ros_example.py.  Note, you cannot have Dynamixel wizard open while using the hand's API, the port will be "busy" with the other process.
- On Ubuntu you can find the hand by ID using `/dev/serial/by-id` The ID will stay persistent on reboots.
- We offically support Python and ROS, but other languages are supported by [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/).

#### Functionality
- Leap Node allows you to command joint angles in different scalings.
- You can read position, velocity and current from the hand.  
- Do not query reads too often, going past 90hz for one set of angles or 30hz for all three will slow down the USB communication.
- The default controller follows the PID control, up to the current limit cap. 
- Other controllers including velocity control or current control are supported as per the [motor manual](https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/)
- For Lite, keep the current limit around 300ma.
- For Full, you can raise the current limit up to 550ma.
- If facing a jittery hand, adjust the PID values down.
- If the hand is too weak, adjust the PID values up.

#### Troubleshooting
- If your motor is 90/180/270 Degrees off, the horn is mounted incorrectly on the motor.  Remount it.
- If no motors show up, check that your serial port permissions are correct. Try this command: `sudo usermod -aG dialout $USER`
- If some motors are missing, make sure they are IDed corrrectly and are connected to the U2D2.
- If you get "overload error" and the motors are flashing red, then they have overloaded (self-collision etc). It should clear on a power cycle.  If it happens often, lower the current limits in the control code so that it does not happen as often.
- If you get "jittery" motors, try lowering the P and D values, either in the roslaunch file or the python file.
- If you feel the motors are too inaccurate, you can also try raising the P and D values.
- To improve latency on Ubuntu try these tips.   Configure [USB Latency Settings in Ubuntu](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/) and the [Dynamixel Python SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK/issues/288)

#### Useful Tools:
- MANO to LEAP joint angle mapping.
- If you have useful tools you feel the community can benefit from, please make a pull request.
- I can also add tools to this upon request.  :)

#### Support:
- Please contact me at kshaw2@andrew.cmu.edu for any issues.
- This code is made available using an MIT License.
- The CAD files are provided with a CC BY-NC-SA Attribution-NonCommercial-ShareAlike license which allows you to use and build upon our work non-commercially.
- LEAP Hand is provided as-is and without warranty.
- If you use LEAP Hand in an academic setting, please cite our paper:
```
@article{shaw2023leaphand,
  title={LEAP Hand: Low-Cost, Efficient, and Anthropomorphic Hand for Robot Learning},
  author={Shaw, Kenneth and Agarwal, Ananye and Pathak, Deepak},
  journal={Robotics: Science and Systems (RSS)},
  year={2023}
}
```
