# create_robot

[ROS2](http://ros.org) driver for iRobot [Create 1 and 2](http://www.irobot.com/About-iRobot/STEM/Create-2.aspx).
This package wraps the C++ library [libcreate][libcreate], which uses iRobot's [Open Interface Specification][oi_spec].

This package was forked from the [create_robot](https://github.com/AutonomyLab/create_robot) from AutonomyLab. The objective is to make it work with ROS2.

<!--[](* Documentation: TODO)-->

## Build Status

It builds on ROS2 Humble but it doesn't work yet.
<!-- - ROS2 Humble ![](https://github.com/autonomylab/create_robot/workflows/Continuous%20Integration/badge.svg?branch=kinetic) -->

## Supported Robots

| Model     | Support    |
|-----------|------------|
| Create 1  |  Maybe *      |
| Create 2  _(firmware >= 3.2.6)_ |  Maybe *      |
| Roomba Original Series | Maybe * |
| Roomba 400 Series |  Maybe * |
| Roomba 500 Series |  Maybe *  |
| Roomba 600 Series |  Hopefully |
| Roomba 700 Series |  Maybe *  |
| Roomba 800 Series |  Maybe * |
| Roomba 900 Series |  Maybe *  |

_* I only have a roomba 600, if you want to try on other models knock yourself out and let me know how it goes :)_


## Features

|  Feature          |  Status       |
|-------------------|---------------|
|  Odometry         | Planned       |
|  Safe mode        | Planned       |
|  Clean demo       | Planned       |
|  Dock demo        | Planned       |
|  Drive wheels     | Planned       |
|  Drive (v,w)      | Planned       |
|  Brush motors     | Planned       |
|  LEDs             | Planned       | 
|  Digit LEDs       | N/A           |
|  Sound            | Planned       |
|  Wheeldrop        | Planned       |
|  Bumpers          | Planned       |
|  Cliff sensor     | Planned       |
|  Dirt detect      | N/A           |
|  Omni IR sensor   | Planned ?     |
|  Left IR sensor   | Planned ?     |
|  Right IR sensor  | Planned ?     |
|  Battery info     | Planned       |
|  Light sensors    | Planned ?     |
| **_Diagnostics_** |               |
|  Corrupt packets  | N/A           |
|  Physical tests   | N/A           |
|  Overcurrent info | N/A           |

## Install

#### Prerequisites

* Internet connection
* [ROS2](https://docs.ros.org/en/humble/Installation.html) _Humble_

#### Compiling

1. Create a catkin workspace  
    ``` bash
    $ cd ~
    $ mkdir -p ~/create_2_ws/src 
    ```

2. Clone this repo  
    ``` bash
    $ cd ~/create_2_ws/src
    $ git clone git@github.com:goowza/create_robot_2.git
    ```
  
3. Install dependencies  
    ``` bash
    $ cd ~/create_2_ws
    $ rosdep install -i --from-path src --rosdistro humble -y 
    ```

4. Build  
    ``` bash
    $ cd ~/create_2_ws
    $ colcon build
    ```
#### USB Permissions
5. In order to connect to Create over USB, ensure your user is in the dialout group
    ``` bash
    $ sudo usermod -a -G dialout $USER
    ```

6. Logout and login for permission to take effect

## Running the driver

### Setup

1. After compiling from source, don't forget to source your workspace:  
    ``` bash
    $ . install/local_setup.bash
    ```

2. Connect computer to the robot 7-pin serial port
  - If using Create 1, ensure that nothing is connected to Create's DB-25 port

3. Launch one of the existing launch files or adapt them to create your own.

### Launch files

For Create 2 (Roomba 600/700 series):
``` bash
$ ros2 launch create_bringup create_2_launch.py
```

For Create 1 (Roomba 500 series):
``` bash
$ ros2 launch create_bringup create_1_launch.py
```

For Roomba 400 series:
``` bash
$ ros2 launch create_bringup roomba_400_launch.py
```

#### Launch file arguments

* **config** - Absolute path to a configuration file (YAML). Default: `create_bringup/config/default.yaml`
* **desc** - Enable robot description (URDF/mesh). Default: `true`

For example, if you would like to disable the robot description and provide a custom configuration file:

```bash
$ ros2 launch create_bringup create_2_launch.py config:=/abs/path/to/config.yaml desc:=false
```

### Parameters

 Name         |  Description |  Default
--------------|--------------|----------
`dev`         |  Device path of robot |  `/dev/ttyUSB0`
`base_frame`  |  The robot's base frame ID | `base_footprint`
`odom_frame`  |  The robot's odometry frame ID | `odom`
`latch_cmd_duration` | If this many seconds passes without receiving a velocity command the robot stops | `0.2`
`loop_hz`     |  Frequency of internal update loop |  `10.0`
`publish_tf`  |  Publish the transform from `odom_frame` to `base_frame` | `true`  
`robot_model` |  The type of robot being controlled (supported values: `ROOMBA_400`, `CREATE_1` and `CREATE_2`) | `CREATE_2`
`baud`        |  Serial baud rate | Inferred based on robot model, but is overwritten upon providing a value

### Publishers

 Topic       | Description  | Type
-------------|--------------|------
 `battery/capacity` | The estimated charge capacity of the robot's battery (Ah) | [std_msgs/Float32][float32]
 `battery/charge` | The current charge of the robot's battery (Ah) | [std_msgs/Float32][float32]
 `battery/charge_ratio` | Charge / capacity | [std_msgs/Float32][float32]
 `battery/charging_state` | The chargins state of the battery | [create_msgs/ChargingState][chargingstate_msg]
 `battery/current` | Current flowing through the robot's battery (A). Positive current implies charging | [std_msgs/Float32][float32]
 `battery/temperature` | The temperature of the robot's battery (degrees Celsius) | [std_msgs/Int16][int16]
 `battery/voltage` | Voltage of the robot's battery (V) | [std_msgs/Float32][float32]
 `bumper` | Bumper state message (including light sensors on bumpers) | [create_msgs/Bumper][bumper_msg]
 `clean_button` | 'clean' button is pressed ('play' button for Create 1) | [std_msgs/Empty][empty]
 `day_button` |  'day' button is pressed | [std_msgs/Empty][empty]
 `hour_button` | 'hour' button is pressed | [std_msgs/Empty][empty]
 `minute_button` | 'minute' button is pressed | [std_msgs/Empty][empty]
 `dock_button` | 'dock' button is pressed ('advance' button for Create 1) | [std_msgs/Empty][empty]
 `spot_button` | 'spot' button is pressed | [std_msgs/Empty][empty]
 `ir_omni` | The IR character currently being read by the omnidirectional receiver. Value 0 means no character is being received | [std_msgs/UInt16][uint16]
 `joint_states` | The states (position, velocity) of the drive wheel joints | [sensor_msgs/JointState][jointstate_msg]
 `mode` | The current mode of the robot (See [OI Spec][oi_spec] for details)| [create_msgs/Mode][mode_msg]
 `odom` |  Robot odometry according to wheel encoders | [nav_msgs/Odometry][odometry]
 `wheeldrop` | At least one of the drive wheels has dropped | [std_msgs/Empty][empty]
 `/tf` | The transform from the `odom` frame to `base_footprint`. Only if the parameter `publish_tf` is `true` | [tf2_msgs/TFMessage](http://docs.ros.org/jade/api/tf2_msgs/html/msg/TFMessage.html)


### Subscribers

Topic       | Description   | Type
------------|---------------|------
`cmd_vel` | Drives the robot's wheels according to a forward and angular velocity | [geometry_msgs/Twist][twist]
`debris_led` | Enable / disable the blue 'debris' LED | [std_msgs/Bool][bool]
`spot_led`   | Enable / disable the 'spot' LED | [std_msgs/Bool][bool]
`dock_led`   | Enable / disable the 'dock' LED | [std_msgs/Bool][bool]
`check_led`  | Enable / disable the 'check robot` LED | [std_msgs/Bool][bool]
`power_led`  | Set the 'power' LED color and intensity. Accepts 1 or 2 bytes, the first represents the color between green (0) and red (255) and the second (optional) represents the intensity with brightest setting as default (255) | [std_msgs/UInt8MultiArray][uint8multiarray]
`set_ascii` | Sets the 4 digit LEDs. Accepts 1 to 4 bytes, each representing an ASCII character to be displayed from left to right | [std_msgs/UInt8MultiArray][uint8multiarray]
`dock` | Activates the demo docking behaviour. Robot enters _Passive_ mode meaning the user loses control (See [OI Spec][oi_spec]) | [std_msgs/Empty][empty]
`undock` | Switches robot to _Full_ mode giving control back to the user | [std_msgs/Empty][empty]
`define_song` | Define a song with up to 16 notes. Each note is described by a MIDI note number and a float32 duration in seconds. The longest duration is 255/64 seconds. You can define up to 4 songs (See [OI Spec][oi_spec]) | [create_msgs/DefineSong][definesong_msg]
`play_song` | Play a predefined song | [create_msgs/PlaySong][playsong_msg]
`side_brush_motor` | Set duty cycle for the side brush. Accepts -1.0 to 1.0 range | [create_msg/MotorSetpoint][motorsetpoint_msg]
`main_brush_motor` | Set duty cycle for the main brush. Accepts -1.0 to 1.0 range | [create_msg/MotorSetpoint][motorsetpoint_msg]
`vacuum_motor` | Set duty cycle for the vacuum. Accepts 0.0 to 1.0 range | [create_msg/MotorSetpoint][motorsetpoint_msg]

## Commanding your Create

If I can make this work, you will be able to move the robot around by sending [geometry_msgs/Twist][twist] messages to the topic `cmd_vel`:

```
linear.x  (+)     Move forward (m/s)
          (-)     Move backward (m/s)
angular.z (+)     Rotate counter-clockwise (rad/s)
          (-)     Rotate clockwise (rad/s)
```
#### Velocity limits

` -0.5 <= linear.x <= 0.5` and `-4.25 <= angular.z <= 4.25`

### Teleoperation

`create_bringup` comes with a launch file for teleoperating Create with a joystick.

``` bash
$ ros2 launch create_bringup joy_teleop_launch.py [joy_config:=xbox360]
```

There exists configuration files for the [Xbox 360 wired controller](https://www.amazon.ca/Microsoft-Xbox-360-Wired-Controller/dp/B003ZSN600) and the [Logitech F710 controller](http://gaming.logitech.com/en-ca/product/f710-wireless-gamepad). You can adapt these files for your preferred joystick configuration.

## Contributions

Contributing to the development and maintenance of _create\_robot\_2_ is encouraged. Feel free to open issues or create pull requests on [GitHub](https://github.com/goowza/create_robot_2).



[libcreate]:  https://github.com/AutonomyLab/libcreate
[oi_spec]:  https://www.adafruit.com/datasheets/create_2_Open_Interface_Spec.pdf
[odometry]:  http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html
[empty]:  http://docs.ros.org/api/std_msgs/html/msg/Empty.html
[uint16]:  http://docs.ros.org/api/std_msgs/html/msg/UInt16.html
[int16]:  http://docs.ros.org/api/std_msgs/html/msg/Int16.html
[twist]:  http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
[bool]:  http://docs.ros.org/api/std_msgs/html/msg/Bool.html
[uint8multiarray]:  http://docs.ros.org/api/std_msgs/html/msg/UInt8MultiArray.html
[float32]:  http://docs.ros.org/api/std_msgs/html/msg/Float32.html
[create_msgs]:  http://github.com/autonomylab/create_robot/tree/melodic
[bumper_msg]:  https://github.com/autonomylab/create_robot/blob/melodic/create_msgs/msg/Bumper.msg
[mode_msg]:  https://github.com/autonomylab/create_robot/blob/melodic/create_msgs/msg/Mode.msg
[chargingstate_msg]:  https://github.com/autonomylab/create_robot/blob/melodic/create_msgs/msg/ChargingState.msg
[jointstate_msg]:  http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html
[definesong_msg]:  https://github.com/autonomylab/create_robot/blob/melodic/create_msgs/msg/DefineSong.msg
[playsong_msg]:  https://github.com/autonomylab/create_robot/blob/melodic/create_msgs/msg/PlaySong.msg
[motorsetpoint_msg]:  https://github.com/autonomylab/create_robot/blob/melodic/create_msgs/msg/MotorSetpoint.msg
