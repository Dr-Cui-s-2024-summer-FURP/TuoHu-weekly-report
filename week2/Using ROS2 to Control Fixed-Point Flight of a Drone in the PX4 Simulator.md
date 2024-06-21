# Using ROS2 to Control Fixed-Point Flight of a Drone in the PX4 Simulator

This guide is based on information from [PX4 documentation on ROS2 offboard control](https://docs.px4.io/main/zh/ros2/offboard_control.html).

**Note:** To control a drone in the PX4 simulator using ROS2, you need two essential repositories:

1. Clone the repository for drone-related variable information:

   ```bash
   git clone https://github.com/PX4/px4_msgs.git
   ```
2. Clone the repository for ROS2 commands to control drone variables:

   ```bash
   git clone https://github.com/PX4/px4_ros_com.git
   ```

Place both repositories in the `src` directory of your ROS2 workspace and compile them.

### Code for Fixed-Point Flight Using ROS2

The following code is inspired by the [offboard control example](https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard/offboard_control.cpp) in the `PX4/px4_ros_com` repository.

Here’s a breakdown of the steps and corresponding code:

#### 1. Control Drone Flight Mode

This code sets the drone to offboard control mode, which allows it to be controlled externally via ROS.

```cpp
this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2) {
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}
```

#### 2. Control Drone State

This code arms the drone, which starts the propellers and puts it in a ready state for takeoff.

```cpp
this->arm();

void OffboardControl::arm() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}
```

#### 3. Set Offboard Control Mode and Trajectory Setpoint

These functions publish the offboard control mode and set a trajectory point for the drone to hover at a specified location.

```cpp
// OffboardControlMode needs to be paired with TrajectorySetpoint
publish_offboard_control_mode();
publish_trajectory_setpoint();
```

```cpp
/**
 * @brief Publish the offboard control mode.
 * For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode() {
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint() {
    TrajectorySetpoint msg{};
    msg.position = {0.0, 0.0, -5.0};
    msg.yaw = -3.14; // [-PI:PI]
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}
```

This code sends the drone to hover at the specified point.

#### 4. Execute Multiple Waypoints

To execute multiple waypoints, set up an array of points and send the location information at different time intervals.

Run the node with:

```bash
ros2 run px4_ros_com offboard_control
```

**Note:** The `trajectory_waypoint` variable might allow setting multiple waypoints at once and sending them without intervals.

### Issues and Troubleshooting

- Ensure the simulator and uXRCE-DDS client are connected before running the code.
- Make sure the ground control station is open; otherwise, the drone won't receive signals and enter flight mode (the drone's wings spinning indicate it's in flight mode).
- Warnings in PX4 typically do not affect the virtual drone's flight.

By following these steps, you can control a drone's fixed-point flight using ROS2 in the PX4 simulator.
