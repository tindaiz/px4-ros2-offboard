# PX4 ROS 2 Offboard

This repository contains a ROS 2 example for offboard control with
[PX4](https://px4.io/).

The workspace contains the following packages:

- `px4_offboard`: Offboard control and vehicle visualization nodes
- `bringup`: ROS 2 launch files for running the control stack and Gazebo bridge
- `px4_msgs`: ROS 2 message definitions for communicating with PX4

The `px4_offboard` package contains the following nodes:

- `offboard_control`: Example of offboard position control using trajectory
  setpoints. The node waits for valid PX4 state data, requests Offboard mode,
  arms the vehicle, climbs 5 m above its initial position, and follows a
  circular trajectory.
- `px4_visualizer`: Publishes vehicle markers and paths for visualization in
  RViz.

The source code is released under the BSD 3-Clause license inherited by the
`px4_msgs` package. This repository is intended for simulation and research
use. Always test in SITL before using a real vehicle.

## Setup


## Running

### Software in the Loop

You will use three terminals to run the offboard demo.

On the first terminal, run a SITL instance from the PX4 Autopilot firmware:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

On the second terminal, run the Micro XRCE-DDS Agent so that ROS 2 nodes can
communicate with the PX4 client:

```bash
MicroXRCEAgent udp4 -p 8888
```

On the third terminal, source the ROS 2 workspace and start the launch file:

```bash
ros2 launch bringup offboard.launch.py
```

This starts both `offboard_control` and `px4_visualizer`. To run only the
offboard controller:

```bash
ros2 launch bringup offboard.launch.py start_visualizer:=false
```

The controller publishes setpoints at 50 Hz and only requests Offboard mode
and arming after receiving valid position, attitude, and vehicle status data
from PX4. Confirm that the simulated vehicle and its failsafe configuration
are ready before allowing it to arm.

## References

- [PX4 ROS 2 User Guide](https://docs.px4.io/main/en/ros/ros2_comm.html)
- [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot)
- [ROS 2 colcon tutorial](https://docs.ros.org/en/humble/Tutorials/Colcon-Tutorial.html)
