# Rotating LiDAR Control for ROS2

# Overview

This Python script implements a ROS 2 node to control a stepper motor attached to a 2D LiDAR, using GPIO pins on a Raspberry Pi. The purpose of this node is to rotate the LiDAR in order to collect 3D point cloud data with another package. The node publishes the current position, velocity, and effort of the turret via the `/angle topic`, and controls the motor's rotation based on specified parameters.

# Dependencies

## Hardware

- Raspberry Pi 4
- Stepper motor (e.g., 28BYJ-48)
- Stepper motor driver (e.g., ULN2003)

## Software

- ROS2 Humble
- RPi.GPIO

# Run node

To run the node with the parameters stored in `config/config.yaml` run

```shell
ros2 launch rotating_lidar_ctrl conf0.launch.py
```

# Parameters

| Parameter             | Description                                               | Default Value |
|-----------------------|-----------------------------------------------------------|---------------|
| `pub_period_s`        | Period (in seconds) for publishing turret state           | 0.01          |
| `ctrl_period_s`       | Control loop period (in seconds)                          | 0.002         |
| `pins`                | List of GPIO pins for controlling the stepper motor       | [17, 18, 27, 22] |
| `lower_angle_deg`     | Lower angle limit in degrees                              | -90.0         |
| `upper_angle_deg`     | Upper angle limit in degrees                              | 90.0          |
| `step_count_per_rot`  | Number of steps per full rotation of the stepper motor    | 4096          |

# Topics

The node publishes to the following topic:

- `/angle` (`sensor_msgs/JointState`): 
    - **name**: Name of the joint (default: turret)
    - **position**: Current angle of the turret (in radians)
    - **velocity**: Angular velocity (in radians per second)
    - **effort**: Effort (always 0.0 in this implementation)