# MAV_INJECT

ROS2 package for editing PX4 drone configuration files during flight operations.

## Overview

MAV_INJECT enables dynamic modification of PX4 drone configuration files as part of an autonomous diagnostics and repair system. The system can:

- Stream raw data from the drone
- Process data through AI models to identify issues (reversed props, motor problems, etc.)
- Land the drone safely
- Edit core configuration files (motor direction, mixer files, params, etc.)
- Reboot the drone
- Resume flight with corrected configuration

## Project Structure

```
MavInject/
├── mav_inject/
│   ├── __init__.py
│   └── injection_test.py      # Main ROS2 node
├── resource/
│   └── mav_inject
├── test/
├── package.xml                 # ROS2 package manifest
├── setup.py                    # Python package setup
├── setup.cfg                   # Setup configuration
└── README.md
```

## Installation on VM

1. Transfer this package to your VM with ROS2 Kilted:
   ```bash
   # On your development machine
   git push origin main

   # On your VM
   git pull origin main
   ```

2. Build the package:
   ```bash
   cd /path/to/your/ros2_workspace
   colcon build --packages-select mav_inject
   source install/setup.bash
   ```

## Usage

### Basic Launch

Run the injection test node with the path to your PX4 build:

```bash
ros2 run mav_inject injection_test --ros-args -p px4_build_path:=/path/to/PX4-Autopilot
```

### With SITL Simulator

```bash
# Start PX4 SITL
cd /path/to/PX4-Autopilot
make px4_sitl gazebo

# In another terminal, run the injector node
ros2 run mav_inject injection_test --ros-args -p px4_build_path:=/path/to/PX4-Autopilot
```

## ROS2 Topics

### Published Topics

- `/px4_injector/status` (std_msgs/String): Status updates and health checks

### Subscribed Topics

- `/px4_injector/command` (std_msgs/String): JSON commands for config injection

## Command Format

Send commands as JSON strings to `/px4_injector/command`:

### Backup Configuration
```json
{"action": "backup"}
```

### Edit Configuration
```json
{
  "action": "edit",
  "file": "relative/path/to/config",
  "modifications": {
    "param_name": "new_value"
  }
}
```

### Restore Configuration
```json
{"action": "restore"}
```

## Example: Send Command

```bash
ros2 topic pub /px4_injector/command std_msgs/String "data: '{\"action\": \"backup\"}'"
```

## Development Workflow

1. Develop code locally on macOS
2. Push changes to git repository
3. Pull on VM with ROS2 Kilted and PX4 simulator
4. Build and test with simulator

## Configuration Files

The node automatically scans for common PX4 configuration files including:

- Parameter files (`.params`)
- Mixer files
- Startup scripts in `ROMFS/px4fmu_common`
- Build-specific configs in `build/px4_sitl_default/etc`

## Future Enhancements

- [ ] Implement actual file editing logic with validation
- [ ] Add safety checks before modifying critical files
- [ ] Implement reboot triggering via MAVLink
- [ ] Add support for specific config types (mixers, params, calibration)
- [ ] Integrate with AI diagnostics pipeline
- [ ] Add rollback mechanisms for failed modifications

## License

MIT

## Authors

MAV_INJECT Team
