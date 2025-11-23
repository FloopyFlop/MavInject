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

2. Build the PX4 Parameter Bridge C library (for direct parameter access):
   ```bash
   cd /path/to/MavInject
   ./build_bridge.sh /path/to/PX4-Autopilot
   ```

   This will create `libpx4_param_bridge.so` (or `.dylib` on macOS) that provides direct C API access to PX4's parameter system.

3. Build the ROS2 package:
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
4. Build the C bridge library with `./build_bridge.sh`
5. Build and test with simulator

## Architecture: Direct C API Access

MAV_INJECT uses **direct C API access** to PX4's parameter system, bypassing the MAVLink protocol entirely. This is the **lowest possible level** of parameter access.

### How It Works

1. **C Bridge Library** (`px4_param_bridge.c`):
   - Small C shared library that wraps PX4's `param_set()` and `param_get()` functions
   - Links directly against PX4's parameter system
   - Provides simple C interface callable from Python

2. **Python ctypes Wrapper** (`px4_param_api.py`):
   - Loads the C bridge library using ctypes
   - Provides Python methods that call C functions directly
   - No network overhead, no protocol encoding/decoding

3. **ROS2 Node** (`injection_test.py`):
   - Uses the Python API to read/write parameters
   - Automatic fallback to MAVLink if C API unavailable
   - Extensive logging of all operations

### Access Levels (from lowest to highest)

1. **Direct C API** (Current implementation): Calls `param_set()`/`param_get()` directly
2. **MAVLink Shell**: Execute shell commands via MAVLink
3. **MAVLink Parameter Protocol**: Standard PARAM_SET/PARAM_VALUE messages
4. **QGroundControl/GUI**: User interface tools

### Building the C Bridge

```bash
# Set PX4 directory and build
cd /path/to/MavInject
./build_bridge.sh /path/to/PX4-Autopilot

# The library will be created at:
# mav_inject/build/libpx4_param_bridge.so (Linux)
# mav_inject/build/libpx4_param_bridge.dylib (macOS)

# Set environment variable (optional, auto-detected if in build/)
export PX4_PARAM_BRIDGE_LIB=/path/to/MavInject/mav_inject/build/libpx4_param_bridge.so
```

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
